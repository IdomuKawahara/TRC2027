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
from .stuck import StuckDetector, StuckRecovery
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
    log(f"  スタック確定: {C.STUCK_COUNT_THRESHOLD} 回連続 ({C.STUCK_COUNT_THRESHOLD * C.LOOP_DT:.1f} s)  "
        f"追加: GPS が {C.STUCK_NOPROGRESS_SEC:.0f}s で {C.STUCK_NOPROGRESS_DIST_M:.1f}m 進まない / "
        f"前進中にヨー角速度 >= {C.STUCK_SPIN_GYRO_DPS:.0f}deg/s")
    log(f"  スタック回避: 後退 {C.STUCK_BACK_SEC:.1f}s → {C.STUCK_RECOVER_TURN_DEG:.0f}° 回転 (ジャイロで確認) → 前進 "
        f"{C.STUCK_RECOVER_FWD_SEC:.0f}s / 回れなければ轍型とみなしてツイスト+ロッキング (最大 {C.STUCK_MAX_ROUNDS} ラウンド)")
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

    # ★改善: スタック検知/回避は mission/stuck.py (段差型は後退→回転、轍型はツイスト→ロッキング)
    detector = StuckDetector(baseline_horiz_acc)
    recovery = StuckRecovery(motor, start_time, baseline_horiz_acc)
    recovery_thread = None
    stuck_reason = ""

    def do_recovery(reason: str, kind_hint):
        """別スレッドで回避動作を行い、終わったら検知器をリセットして誘導走行に戻す。"""
        nonlocal in_recovery
        try:
            recovery.run(last_turn_dir, reason, kind_hint)
        finally:
            detector.reset()
            in_recovery = False

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

            # ── スタック検知 (★改善: mission/stuck.py の StuckDetector) ──
            #   A: 前進中に加速度が静止時並み & GPS が遅い (従来)   B: 超音波が近い (従来)
            #   C: 前進指令が続くのに GPS 位置が進まない (追加)     D: 前進指令なのにその場で回る (追加)
            is_forward_intent = (motor_cmd == "FORWARD")
            gps_speed_mps = st.g_gps_speed * C.KNOTS_TO_MPS
            note = ""
            stuck_confirmed = False
            if not in_recovery:
                _, stuck_confirmed, stuck_reason = detector.update(
                    loop_start, is_forward_intent, horiz_acc, st.g_gps_valid, gps_speed_mps,
                    st.g_sonar_m, st.g_gyro[2], lat, lng,
                    driving=motor_cmd.startswith(("FORWARD", "TURN")))
                stuck_count = detector.count
                if stuck_confirmed:
                    note = f"STUCK!{stuck_reason}"

            # ── モータ制御 ──
            if in_recovery:
                motor_cmd = "RECOVERING"
            elif not st.g_gps_valid:
                motor.stop()
                motor_cmd = "GPS_WAIT"
            elif stuck_confirmed:
                motor.stop()
                motor_cmd = "STUCK_STOP"
                # 別スレッドで回復動作 (メインループをブロックしないため)。
                # ★改善: スレッドが動き出す前に次の周期が来ても二重に起動しないよう、
                #        フラグはここで立てる
                in_recovery = True
                recovery_thread = threading.Thread(target=do_recovery, daemon=True,
                                                   args=(stuck_reason, detector.kind_hint))
                recovery_thread.start()
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
        # ★改善: 回避動作のスレッドが残ったまま Phase5 に進むとモータを取り合うので、
        #        先に打ち切って終了を待つ
        if recovery_thread is not None and recovery_thread.is_alive():
            recovery.cancel()
            recovery_thread.join(timeout=3.0)
            if recovery_thread.is_alive():
                log("[Phase4][WARN] 回避動作スレッドが 3 秒以内に止まりませんでした", "WARN")
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
