"""
phase4_gps.py
=============
Phase 4: GPS 誘導走行 (方位融合・スタック検知/回復・カメラでのゴール見張り)。
(main6.py から機械的に分割。動作は変更していない)
元の位置: main6.py 1750〜2179 行
"""

from gpiozero import LED
import math
import time
import threading
from BNO055 import BNO055
import goal_vision as gv
import heading_fusion as hf

from . import config as C
from . import state as st
from .state import mark_sensor_fail, mark_sensor_ok
from .logger import blink_led, log, log_data_status, log_sensor_row
from .hardware import MotorController, SonarSensor
from .navigation import calc_azimuth_with_front, calc_direction_diff, calc_distance, calc_target_bearing
from .camera import make_camera, make_goal_detector

# ─── Phase 4: GPS 誘導走行 + スタック検知・回避 ──────────────────────────────

def phase4_guided_run(bno: BNO055, sonar: SonarSensor,
                      motor: MotorController, led: LED, start_time: float,
                      baseline_horiz_acc: float, min_run_sec: float = 0.0) -> str:
    """
    GPS 誘導走行。目標まで PHASE4_TO_5_RADIUS 以内に入ったら Phase4 を終了し、
    戻り値 "NEAR_GOAL" を返して Phase5 (カメラ最終接近) へ引き継ぐ。
    ★main6: GPS で CAM_P4_ACTIVATE_RADIUS_M 以内に入ったらカメラでゴールを見張り、
    ゴールと確定できたら "CONE_FOUND" を返す (カメラは g_p4_camera で Phase5 へ引き継ぐ)。
    ★main6: 機体方位は地磁気と GPS COG の融合 (g_heading_est) を使う。
    タイムアウト時は "TIMEOUT"、中断時は "ABORT" を返す。

    ★変更: 機体の物理的な前方とセンサ方位のズレは、Phase0 で一度だけ
    実行する BNO055 の Axis Remap (set_bno_orientation, BNO_ORIENTATION)
    で補正済みであるため、Phase4 側で前方候補を探索する処理は行わない。
    方位計算は全て calc_azimuth_with_front() (EMA平滑化済みの地磁気から
    計算した方位) を使用する。
    """

    st.phase = 4
    st.g_p4_camera = None
    if st.g_heading_est is None:
        st.g_heading_est = hf.HeadingEstimator(
            cog_min_speed_mps=C.HEADING_COG_MIN_SPEED_MPS,
            straight_min_sec=C.HEADING_STRAIGHT_MIN_SEC,
            gyro_max_dps=C.HEADING_GYRO_MAX_DPS,
            cog_max_age_sec=C.HEADING_COG_MAX_AGE_SEC,
            offset_alpha=C.HEADING_OFFSET_ALPHA,
            min_offset_samples=C.HEADING_MIN_OFFSET_SAMPLES,
            outlier_deg=C.HEADING_OUTLIER_DEG)
    heading_est = st.g_heading_est
    blink_led(led)
    log("─" * 62)
    log(f"[Phase4] GPS 誘導走行開始")
    if min_run_sec > 0:
        # ★追加: Phase5 でゴールが見つからず戻ってきた場合
        log(f"[Phase4] Phase5 でゴールが見つからなかったため GPS 誘導に戻りました。"
            f"最低 {min_run_sec:.0f} 秒は GPS 誘導で走ってから Phase5 へ移ります")
    log(f"  目標: ({C.TARGET_LAT}, {C.TARGET_LNG})  Phase5移行半径: {C.PHASE4_TO_5_RADIUS} m")
    log(f"  BNO055 Axis Remap 設定: BNO_ORIENTATION={C.BNO_ORIENTATION} (Phase0 で適用済み)")
    log(f"  スタック閾値(★厳格化): 前進中の水平加速度 < {C.STUCK_HORIZON_ACCEL_THRESH} m/s² "
        f"かつ GPS速度 < {C.STUCK_GPS_SPEED_THRESH_MPS} m/s  or 超音波 < {C.STUCK_SONAR_DIST_THRESH} m")
    log(f"  スタック確定: {C.STUCK_COUNT_THRESHOLD} 回連続 ({C.STUCK_COUNT_THRESHOLD * C.LOOP_DT:.1f} s)")
    log(f"  方位: 直進中 (速度>={C.HEADING_COG_MIN_SPEED_MPS} m/s) は GPS の COG、それ以外は地磁気+学習したずれ  "
        f"[{heading_est.summary()}]")
    if C.CAM_P4_ENABLE:
        log(f"  カメラ: GPS {C.CAM_P4_ACTIVATE_RADIUS_M:.0f} m 以内で起動し、ゴールを "
            f"{C.CONE_LOCK_FRAMES} 枚続けて捉えたら Phase5 へ")
    log_data_status("Phase4開始時点")

    # ★削除: Phase4限定のモータ反転フラグ (PHASE4_MOTOR_REVERSED) は撤去。
    #        モータは常に素直に forward=前進 / backward=後退として動作する。

    # ★追加: Phase4 (誘導走行・スタック回復) では、いかなる場合も
    #        モータを後退方向へ回転させてはならない。backward() が万一
    #        呼ばれても物理的に後退しないよう、ここで安全ロックを有効化する。
    motor.set_forward_only(True)
    log("[Phase4] ★安全ロック: 後退禁止モードを有効化しました "
        "(以降、backward()を呼んでも実際には後退しません)")

    # コンソールヘッダー (横 1 行ログ)
    HDR = (f"{'T[s]':>7}  {'Lat':>11}  {'Lng':>12}  "
           f"{'Dist[m]':>8}  {'Bear[°]':>7}  {'Az[°]':>6}  "
           f"{'Diff[°]':>7}  {'Sonar[m]':>9}  "
           f"{'HorizAcc':>9}  {'StuckCnt':>8}  {'Motor':>14}  {'Note':>10}")
    DAT = (f"{{:>7.2f}}  {{:>11.6f}}  {{:>12.6f}}  "
           f"{{:>8.2f}}  {{:>7.2f}}  {{:>6.2f}}  "
           f"{{:>7.2f}}  {{:>9}}  "
           f"{{:>9.4f}}  {{:>8d}}  {{:>14}}  {{:>10}}")
    print()
    print("-" * len(HDR))
    print(HDR)
    print("-" * len(HDR))

    p4_start   = time.time()
    stuck_count = 0
    near_goal    = False   # ★変更: GOAL_RADIUS到達→ミッション終了 ではなく Phase5 への移行フラグ
    cone_found   = False   # ★main6: カメラでゴールを確定した
    watcher      = None    # ★main6: カメラの見張り (GPS で近づいたら起動)
    cam_p4_failed = False
    cone_lock    = gv.ConeLock(confirm_frames=C.CONE_LOCK_FRAMES,
                               consistency_deg=C.CONE_LOCK_CONSISTENCY_DEG,
                               gps_check_min_dist_m=C.CONE_LOCK_GPS_CHECK_MIN_M,
                               gps_bearing_tol_deg=C.CONE_LOCK_GPS_BEARING_TOL_DEG)
    last_cone_seq = 0
    cone_str = "-"
    timed_out    = False
    in_recovery  = False   # スタック回復中フラグ
    last_turn_dir = None   # 直前に旋回していた方向 ("L" or "R")。前進中などは更新しない
    last_nav_report_time = p4_start  # 目標距離・方位ずれのまとめログ用
    motor_cmd = "FORWARD"  # ★追加: スタック判定で「前進中か」を見るため、ループ開始前に初期化しておく

    def rotate_until_degrees(direction: str, target_deg: float, note_prefix: str) -> float:
        """
        機体前方基準の方位 (calc_azimuth_with_front) の変化量を積算し、
        指定方向へ target_deg 度回転するまで強旋回を続ける。
        メインループが継続的に更新しているグローバル g_mag を参照するだけで
        自らは BNO055 に触れない (メインループとの I2C 同時アクセスを回避するため)。
        方位が読めない/回転が進まない等の異常時に無限ループしないよう、
        STUCK_RECOVER_TURN_TIMEOUT_SEC で安全に打ち切る。
        戻り値: 実際に回転できた角度 (概算, 度)
        """
        if direction == "L":
            motor.turn_left_strong()
            note = f"{note_prefix}_L"
        else:
            motor.turn_right_strong()
            note = f"{note_prefix}_R"

        t_start   = time.time()
        prev_az   = calc_azimuth_with_front(st.g_mag)
        cumulative = 0.0
        while True:
            now = time.time()
            if now - t_start > C.STUCK_RECOVER_TURN_TIMEOUT_SEC:
                log(f"[Phase4][STUCK] {note}: 回転タイムアウト "
                    f"({C.STUCK_RECOVER_TURN_TIMEOUT_SEC:.0f}s, 約{cumulative:.1f}度で打ち切り)", "WARN")
                break
            cur_az = calc_azimuth_with_front(st.g_mag)
            delta  = calc_direction_diff(cur_az, prev_az)
            cumulative += delta
            prev_az = cur_az
            log_sensor_row(time.time() - start_time, note,
                           f"rot={cumulative:.1f}/{target_deg:.0f}deg")
            if abs(cumulative) >= target_deg:
                break
            time.sleep(0.1)

        motor.stop()
        log(f"[Phase4][STUCK] {note}: 回転完了 (目標{target_deg:.0f}度 / 実測約{cumulative:.1f}度)")
        return cumulative

    def forward_with_monitor(duration_sec: float, note_prefix: str) -> bool:
        """
        duration_sec 秒前進しつつ、その間も水平加速度・GPS速度・超音波による
        スタック判定を継続する (★通常走行時と同じ厳格化条件)。
        STUCK_COUNT_THRESHOLD 回連続でスタック条件を満たしたらその場で
        打ち切り True (再スタック) を返す。何も起きなければ False。
        グローバル g_acc / g_gps_speed / g_gps_valid / g_sonar_m
        (メインループが更新) を参照するだけで、自らはセンサに触れない。
        """
        motor.forward()
        t_end = time.time() + duration_sec
        local_stuck_count = 0
        restuck = False
        while time.time() < t_end:
            horiz_acc = math.sqrt(st.g_acc[0]**2 + st.g_acc[1]**2)
            gps_speed_mps = st.g_gps_speed * C.KNOTS_TO_MPS
            is_slow_gps    = (not st.g_gps_valid) or (gps_speed_mps < C.STUCK_GPS_SPEED_THRESH_MPS)
            is_stuck_acc   = is_slow_gps and (horiz_acc < (baseline_horiz_acc + C.STUCK_HORIZON_ACCEL_THRESH))
            is_stuck_sonar = (st.g_sonar_m is not None and st.g_sonar_m < C.STUCK_SONAR_DIST_THRESH)
            if is_stuck_acc or is_stuck_sonar:
                local_stuck_count += 1
            else:
                local_stuck_count = 0

            log_sensor_row(time.time() - start_time, f"{note_prefix}_FORWARD",
                           f"remain={t_end - time.time():.1f}s stuck_cnt={local_stuck_count}")

            if local_stuck_count >= C.STUCK_COUNT_THRESHOLD:
                restuck = True
                break
            time.sleep(0.1)

        motor.stop()
        return restuck

    def do_recovery():
        """
        スタック回復動作 (★改良版)。
        スタック検知の直前に旋回していた方向と逆方向へ、地磁気の方位変化から
        【ちょうど180°】回転したことを確認してから STUCK_RECOVER_FWD_SEC (10秒)
        前進する。この前進の最中も引き続きスタック判定を行い、再びスタックを
        検知した場合はさらに90°回転してから再度10秒前進する、という動作を
        STUCK_RECOVER_MAX_RETRIES 回まで繰り返す (無限ループ防止のため上限あり)。
        """
        nonlocal stuck_count, in_recovery
        in_recovery = True

        # 直前の旋回方向の反対を選択 (旋回していなければデフォルトで右旋回)
        if last_turn_dir == "L":
            recover_dir = "R"
        elif last_turn_dir == "R":
            recover_dir = "L"
        else:
            recover_dir = "R"

        log(f"[Phase4][STUCK] スタック検知！ 回復動作開始 (直前の旋回方向: {last_turn_dir or 'なし'})")
        log(f"         {'左' if recover_dir == 'L' else '右'}方向へ約{C.STUCK_RECOVER_TURN_DEG:.0f}° 回転"
            f" (直前と反対方向) → 前進 {C.STUCK_RECOVER_FWD_SEC:.0f} s")

        rotate_until_degrees(recover_dir, C.STUCK_RECOVER_TURN_DEG, "RECOVER180")

        cur_dir = recover_dir
        attempt = 0
        while True:
            attempt += 1
            restuck = forward_with_monitor(C.STUCK_RECOVER_FWD_SEC, f"RECOVER_ATT{attempt}")
            if not restuck:
                break
            if attempt >= C.STUCK_RECOVER_MAX_RETRIES:
                log(f"[Phase4][STUCK] 前進中に再スタックを検知しましたが、"
                    f"最大試行回数({C.STUCK_RECOVER_MAX_RETRIES})に達したため走行を再開します。", "WARN")
                break
            cur_dir = "R" if cur_dir == "L" else "L"
            log(f"[Phase4][STUCK] 前進中に再びスタックを検知 → "
                f"{'左' if cur_dir == 'L' else '右'}方向へ{C.STUCK_RECOVER_RETRY_TURN_DEG:.0f}° 回転して再度回避します "
                f"(試行 {attempt + 1}/{C.STUCK_RECOVER_MAX_RETRIES})")
            rotate_until_degrees(cur_dir, C.STUCK_RECOVER_RETRY_TURN_DEG, f"RECOVER90_{attempt}")

        stuck_count = 0
        in_recovery = False
        log("[Phase4][STUCK] 回復動作完了。誘導走行に戻ります。")

    try:
        while True:
            loop_start = time.time()
            elapsed    = loop_start - p4_start

            # ── タイムアウト判定 ──
            if elapsed > C.TIMEOUT_SEC:
                log(f"[Phase4] タイムアウト ({C.TIMEOUT_SEC} s)")
                timed_out = True
                break

            # ── BNO055 取得 ──
            try:
                st.g_acc   = bno.getAcc()
                st.g_mag   = bno.getMag()
                st.g_gyro  = bno.getGyro()
                st.g_calib = bno.getCalibrationStatus()
                mark_sensor_ok("BNO055", "航法演算用")
            except Exception as e:
                mark_sensor_fail("BNO055", f"読み取りエラー: {e}")
                log(f"BNO055 エラー: {e}", "WARN")
                st.g_acc = [0.0, 0.0, 0.0]
                st.g_mag = [0.0, 0.0, 0.0]

            # ── 超音波センサ取得 (失敗しても止めない) ──
            st.g_sonar_m = sonar.get_distance_m()
            sonar_str = f"{st.g_sonar_m:.3f}" if st.g_sonar_m is not None else "---"

            # ── 航法計算 ──
            # ★main6: 機体方位 = 地磁気 (EMA 平滑化) と GPS COG の融合
            #   motor_cmd はこの時点ではまだ「直前のループで出した指令」= 直前の
            #   0.1 秒間に実際に機体がしていた動き
            mag_azimuth    = calc_azimuth_with_front(st.g_mag)
            azimuth        = heading_est.update(
                loop_start, mag_azimuth,
                gps_valid=st.g_gps_valid, gps_speed_mps=st.g_gps_speed * C.KNOTS_TO_MPS,
                gps_course=st.g_gps_course, gps_fix_time=st.g_gps_fix_time,
                forward=(motor_cmd == "FORWARD"), gyro_z_dps=st.g_gyro[2])
            lat, lng       = st.g_gps_lat, st.g_gps_lng
            distance       = calc_distance(lat, lng)
            target_bearing = calc_target_bearing(lat, lng)
            diff           = calc_direction_diff(azimuth, target_bearing)

            # ── ★main6: GPS で近づいたらカメラでゴールを見張る ──
            if C.CAM_P4_ENABLE and st.g_gps_valid and not cam_p4_failed:
                if watcher is None and distance <= C.CAM_P4_ACTIVATE_RADIUS_M:
                    log(f"[Phase4] ゴールまで {distance:.1f} m → カメラでゴールの見張りを開始します")
                    try:
                        watcher = gv.ConeWatcher(make_camera(), make_goal_detector(),
                                                 interval=C.CAM_P4_INTERVAL_SEC,
                                                 threaded=C.CAM_P4_THREADED, clock=time.time)
                        if not watcher.start():
                            raise RuntimeError(watcher.camera.last_error)
                    except Exception as e:
                        log(f"[Phase4][WARN] カメラを起動できません ({e}) → GPS だけで誘導します", "WARN")
                        if watcher is not None:
                            watcher.close()
                        watcher = None
                        cam_p4_failed = True
                elif watcher is not None and distance > C.CAM_P4_DEACTIVATE_RADIUS_M:
                    log(f"[Phase4] ゴールから {distance:.1f} m 離れたのでカメラを止めます")
                    watcher.close()
                    watcher = None
                    cone_lock.reset()
            if watcher is not None:
                got = watcher.latest()
                if got is not None and got[2] != last_cone_seq:
                    det, _, last_cone_seq = got
                    locked = cone_lock.update(det, azimuth, target_bearing,
                                              distance if st.g_gps_valid else None)
                    cone_str = (f"cone{cone_lock.count}" if det.detected else "-")
                    if det.detected:
                        log_sensor_row(time.time() - start_time, "P4_CONE",
                                       f"{det.summary()} lock={cone_lock.last_reason}")
                    if locked:
                        log(f"[Phase4] カメラでゴールを確定しました ({cone_lock.last_reason}, "
                            f"GPS距離={distance:.1f} m) → Phase5 へ移行します")
                        cone_found = True
                        break

            # ── 水平加速度ノルム (スタック検知用) ──
            horiz_acc = math.sqrt(st.g_acc[0]**2 + st.g_acc[1]**2)

            # ── ★追加: 目標までの距離・方位ずれのまとめ表示 ──
            # 9軸センサ(地磁気, 前方補正済み)から算出した機体方位と目標方位の
            # ずれ (diff) を NAV_REPORT_INTERVAL_SEC ごとにログへはっきり表示する。
            if loop_start - last_nav_report_time >= C.NAV_REPORT_INTERVAL_SEC:
                last_nav_report_time = loop_start
                turn_side = "右" if diff < 0 else ("左" if diff > 0 else "正面")
                log(f"[Phase4] 目標までの距離={distance:.2f} m  "
                    f"機体方位={azimuth:.1f}° [{heading_est.source}] (地磁気のみ={mag_azimuth:.1f}°)  "
                    f"目標方位={target_bearing:.1f}°  "
                    f"ずれ={diff:+.1f}° ({turn_side})  {heading_est.summary()}"
                    + (f"  カメラ: {cone_lock.last_reason}" if watcher is not None else ""))
                log_data_status("GPS誘導走行演算 (航法計算+スタック判定)")

            # ── ゴール判定 (★変更: ここではミッション終了ではなく Phase5 へ移行) ──
            if st.g_gps_valid and distance <= C.PHASE4_TO_5_RADIUS and elapsed >= min_run_sec:
                log(f"[Phase4] 目標まで {C.PHASE4_TO_5_RADIUS} m 以内に到達！ distance={distance:.2f} m")
                log("[Phase4] Phase5 (カメラ最終接近) へ移行します。")
                near_goal = True
                break

            # ── スタック検知 (★厳格化) ────────────────────────────────────
            # 判定A: 水平加速度が基準より低い。ただし「前進」を指示している
            #        間のみ有効とする。旋回(その場スピン/弱旋回)中はもともと
            #        並進加速度が小さくなるため、以前はこれを誤って
            #        「動いていない=スタック」と判定していた (実運用で確認された誤検知)。
            is_forward_intent = (motor_cmd == "FORWARD")
            is_stuck_acc_raw  = horiz_acc < (baseline_horiz_acc + C.STUCK_HORIZON_ACCEL_THRESH)

            # 判定A': 加速度だけでなく GPS 対地速度も十分低いことを AND 条件で要求。
            #         振動などのノイズだけで加速度が一瞬下がっただけの誤検知を防ぐ。
            gps_speed_mps = st.g_gps_speed * C.KNOTS_TO_MPS
            is_slow_gps   = (not st.g_gps_valid) or (gps_speed_mps < C.STUCK_GPS_SPEED_THRESH_MPS)

            is_stuck_acc = is_forward_intent and is_stuck_acc_raw and is_slow_gps

            # 判定B: 超音波が近すぎる (壁に当たっている。旋回中でも常時有効)
            is_stuck_sonar = (st.g_sonar_m is not None and st.g_sonar_m < C.STUCK_SONAR_DIST_THRESH)

            is_stuck = is_stuck_acc or is_stuck_sonar

            note = ""
            if is_stuck and not in_recovery:
                stuck_count += 1
                if stuck_count >= C.STUCK_COUNT_THRESHOLD:
                    note = "STUCK!"
            else:
                if not in_recovery:
                    stuck_count = 0

            # ── モータ制御 ──
            if in_recovery:
                motor_cmd = "RECOVERING"
            elif not st.g_gps_valid:
                motor.stop()
                motor_cmd = "GPS_WAIT"
            elif stuck_count >= C.STUCK_COUNT_THRESHOLD:
                motor.stop()
                motor_cmd = "STUCK_STOP"
                # 別スレッドで回復動作 (メインループをブロックしないため)
                t_rec = threading.Thread(target=do_recovery, daemon=True)
                t_rec.start()
            else:
                motor_cmd = motor.apply_diff(diff)

            # ── 直前の旋回方向をトラッキング (スタック回復時に反対方向へ回すため) ──
            # ★変更: apply_diff() が返す文字列に "TURN_R_STRONG(180°反転優先)" の
            #        ように末尾補足が付く場合があるため、完全一致 (in) ではなく
            #        接頭辞 (startswith) で判定するように変更した。
            if motor_cmd.startswith("TURN_L"):
                last_turn_dir = "L"
            elif motor_cmd.startswith("TURN_R"):
                last_turn_dir = "R"
            # FORWARD / GPS_WAIT / STUCK_STOP / RECOVERING の場合は前回の値を維持する

            # ── コンソール 1 行表示 ──
            row = DAT.format(
                elapsed, lat, lng,
                distance, target_bearing, azimuth,
                diff, sonar_str,
                horiz_acc, stuck_count, motor_cmd, (note or f"{heading_est.source} {cone_str}"),
            )
            print(row, flush=True)

            # ── ログ蓄積 ──
            log_sensor_row(time.time() - start_time, motor_cmd,
                           f"{note} head={azimuth:.1f}[{heading_est.source}] mag={mag_azimuth:.1f} "
                           f"ofs={heading_est.offset_deg:+.1f} cone={cone_str}".strip())

            # ── ループ待機 ──
            wait = C.LOOP_DT - (time.time() - loop_start)
            if wait > 0:
                time.sleep(wait)

    except KeyboardInterrupt:
        log("[Phase4] Ctrl+C 受信 → 緊急停止")

    finally:
        motor.stop()
        # ★追加: Phase4限定の後退禁止ロックを解除 (Phase5などに影響させないため)
        motor.set_forward_only(False)
        log("[Phase4] 安全ロック: 後退禁止モードを解除しました")
        # ★main6: カメラは、ゴールを見つけたときだけ開いたまま Phase5 に引き継ぐ
        if watcher is not None:
            if cone_found:
                watcher.stop()
                st.g_p4_camera = watcher.camera
            else:
                watcher.close()
        print()
        log("─" * 62)
        log(f"[Phase4] 方位の融合: {heading_est.summary()}")
        if cone_found:
            log("[Phase4] 結果: カメラでゴールを発見 → Phase5 へ移行")
        elif near_goal:
            log("[Phase4] 結果: Phase5 へ移行")
        elif timed_out:
            log("[Phase4] 結果: タイムアウト")
        else:
            log("[Phase4] 結果: 中断")
        log(f"[Phase4] 最終距離: {calc_distance(st.g_gps_lat, st.g_gps_lng):.2f} m")

    if cone_found:
        return "CONE_FOUND"
    if near_goal:
        return "NEAR_GOAL"
    elif timed_out:
        return "TIMEOUT"
    else:
        return "ABORT"
