"""
phase5_camera.py
================
Phase 5: カメラ色検知による最終接近 + 超音波による 0m 到達確認。
カメラが使えないときの超音波左右スキャン (phase5_sonar_fallback) も含む。
(main6.py から機械的に分割。動作は変更していない)
元の位置: main6.py 2180〜2398 行, 2429〜2807 行
"""

from gpiozero import LED
import time
import datetime
import goal_vision as gv

from . import config as C
from . import state as st
from .state import mark_sensor_fail, mark_sensor_ok
from .logger import blink_led, log, log_data_status, log_sensor_row
from .hardware import MotorController, SonarSensor
from .navigation import calc_azimuth, calc_azimuth_with_front, calc_direction_diff, calc_distance, calc_target_bearing
from .camera import make_camera, make_goal_check_config, make_goal_detector

# ─── Phase 5 フォールバック: 超音波センサによる最終接近 (左右スキャン方式) ────
# ★変更: Phase5 の主役はカメラ (phase5_camera_approach) になった。この関数は
#        カメラが初期化できない/撮影が続けて失敗する/長時間ゴールが見つからない
#        ときの予備として残している (中身は従来の main2.py の Phase5 のまま)。

def phase5_sonar_fallback(sonar: SonarSensor, motor: MotorController,
                          led: LED, start_time: float, blink: bool = True,
                          timeout_sec: float = None) -> bool:
    """
    ★変更: 旧 phase5_final_approach。カメラが使えない場合のフォールバック。
    blink=False ならフェーズ切り替えの LED 点滅を省略する (カメラ方式から
    切り替わった場合、既に点滅済みのため)。timeout_sec を渡すとその秒数で
    打ち切る (残り時間だけ実行するため)。

    Phase4 で目標まで PHASE4_TO_5_RADIUS 以内に入った後の最終接近フェーズ。

    GPS はこれ以上の精度で距離を詰められないため、前方の超音波センサで
    ゴールに設置された物体との距離を直接測りながら近づく。

    ★変更: 従来は超音波の値を判断に使わず、turn_left_weak/turn_right_weak
    を機械的に交互切り替えするだけの蛇行だったが、これを「超音波センサの
    値に基づいて能動的に対象物へ向かう」方式に変更した。1サイクルは
    以下の手順で構成される:

      1. turn_left_strong() で FINAL_SCAN_SEC 秒間左へ旋回しながら、
         その間に得られた超音波距離の最小値を dist_left として記録する
         (指向性のあるセンサが対象物に近い方向を向いたときほど、
          距離が短く読めるという前提)。
      2. turn_right_strong() で FINAL_SCAN_SEC 秒間右へ旋回しながら、
         同様に dist_right を記録する (これで一旦、旋回前の向きに近い
         状態へ戻ることになる)。
      3. dist_left と dist_right を比較し、より近かった方向 (対象物が
         その方向にある可能性が高い) へ turn_left_weak / turn_right_weak
         (両輪前進しつつ片側だけ弱める動き) で FINAL_SCAN_SEC 秒間だけ
         向き直しながら前進する。どちらも無効値ならとりあえず forward()
         で前進する。
      4. 1 に戻って繰り返す。

    スキャン中および接近中、いずれかの時点で超音波距離が
    FINAL_STOP_DIST_M (0.05 m) 以下になったら「ゴールに0距離まで到達」
    とみなして即座に停止・終了する。
    安全のため FINAL_APPROACH_TIMEOUT_SEC で強制終了し、超音波が長時間
    有効値を返さない場合は警告を出す。

    ※ BNO055 の Axis Remap (Phase0 で設定済み) による前方補正は、
       ログ表示用の参考ナビ情報 (calc_azimuth_with_front) にも自動的に
       反映される。

    戻り値: True = 目標到達, False = タイムアウト/中断
    """
    st.phase = 5
    if blink:
        blink_led(led)
    if timeout_sec is None:
        timeout_sec = C.FINAL_APPROACH_TIMEOUT_SEC
    log("─" * 62)
    log("[Phase5] (フォールバック) 超音波センサによる左右スキャン接近を開始します")
    log(f"  停止距離: {C.FINAL_STOP_DIST_M:.2f} m 以下で到達  "
        f"スキャン: 左右それぞれ {C.FINAL_SCAN_SEC:.1f} s  "
        f"タイムアウト: {timeout_sec:.0f} s")
    log_data_status("Phase5開始時点")

    HDR = (f"{'T[s]':>7}  {'Lat':>11}  {'Lng':>12}  "
           f"{'Dist[m]':>8}  {'Bear[°]':>7}  {'Az[°]':>6}  "
           f"{'Diff[°]':>7}  {'SonL[m]':>8}  {'SonR[m]':>8}  {'Motor':>16}")
    DAT = (f"{{:>7.2f}}  {{:>11.6f}}  {{:>12.6f}}  "
           f"{{:>8.2f}}  {{:>7.2f}}  {{:>6.2f}}  "
           f"{{:>7.2f}}  {{:>8}}  {{:>8}}  {{:>16}}")
    print()
    print("-" * len(HDR))
    print(HDR)
    print("-" * len(HDR))

    p5_start = time.time()
    last_nav_report_time  = p5_start
    last_sonar_valid_time = p5_start
    reached  = False
    timed_out = False

    def check_reached(dist) -> bool:
        """★追加: 距離が FINAL_STOP_DIST_M 以下なら到達とみなす共通判定。"""
        return dist is not None and dist <= C.FINAL_STOP_DIST_M

    def scan_direction(direction: str, note_prefix: str):
        """
        ★追加: 指定方向へ FINAL_SCAN_SEC 秒間強旋回しながら超音波を
        サンプリングし、その間に得られた最小距離を返す。
        途中で到達判定を満たした場合は即座に停止して "REACHED" を返す
        (呼び出し元はこれを見て全体を打ち切る)。
        戻り値: (最小距離 or None, 到達したか True/False)
        """
        nonlocal last_sonar_valid_time
        if direction == "L":
            motor.turn_left_strong()
            motor_cmd = f"{note_prefix}_SCAN_L"
        else:
            motor.turn_right_strong()
            motor_cmd = f"{note_prefix}_SCAN_R"

        best = None
        t_end = time.time() + C.FINAL_SCAN_SEC
        while time.time() < t_end:
            st.g_sonar_m = sonar.get_distance_m()
            now = time.time()
            if st.g_sonar_m is not None:
                last_sonar_valid_time = now
                if best is None or st.g_sonar_m < best:
                    best = st.g_sonar_m
            log_sensor_row(now - start_time, motor_cmd,
                           f"best={best if best is not None else '---'}")
            if check_reached(st.g_sonar_m):
                motor.stop()
                log(f"[Phase5] スキャン中に目標到達！ ({direction}方向, "
                    f"前方オブジェクトまで {st.g_sonar_m:.3f} m)")
                return best, True
            time.sleep(C.FINAL_SCAN_LOOP_DT)

        return best, False

    try:
        while True:
            now = time.time()
            elapsed = now - p5_start

            if elapsed > timeout_sec:
                log(f"[Phase5] タイムアウト ({timeout_sec:.0f} s)", "WARN")
                timed_out = True
                break

            if now - last_sonar_valid_time >= C.FINAL_SONAR_LOST_WARN_SEC:
                log(f"[Phase5][WARN] 超音波センサが {C.FINAL_SONAR_LOST_WARN_SEC:.0f}s "
                    f"以上有効値を返していません。前方に物体が無いか、センサ不調の可能性があります。", "WARN")
                last_sonar_valid_time = now  # 再警告を連発しないよう基準を更新

            # ── 1. 左スキャン ──
            dist_left, hit = scan_direction("L", "P5")
            if hit:
                reached = True
                break

            # ── 2. 右スキャン ──
            dist_right, hit = scan_direction("R", "P5")
            if hit:
                reached = True
                break

            # ── 3. 近かった方向を判定し、その方向へ向き直しながら前進 ──
            if dist_left is not None and (dist_right is None or dist_left <= dist_right):
                favored, favored_str = "L", f"{dist_left:.3f}"
                motor.turn_left_weak()
                motor_cmd = "APPROACH_L"
            elif dist_right is not None:
                favored, favored_str = "R", f"{dist_right:.3f}"
                motor.turn_right_weak()
                motor_cmd = "APPROACH_R"
            else:
                favored, favored_str = "-", "---"
                motor.forward()
                motor_cmd = "APPROACH_FWD"

            t_end = time.time() + C.FINAL_SCAN_SEC
            while time.time() < t_end:
                st.g_sonar_m = sonar.get_distance_m()
                tnow = time.time()
                if st.g_sonar_m is not None:
                    last_sonar_valid_time = tnow
                log_sensor_row(tnow - start_time, motor_cmd,
                               f"favored={favored}")
                if check_reached(st.g_sonar_m):
                    motor.stop()
                    log(f"[Phase5] 接近中に目標到達！ 前方オブジェクトまで "
                        f"{st.g_sonar_m:.3f} m")
                    reached = True
                    break
                time.sleep(C.FINAL_SCAN_LOOP_DT)
            if reached:
                break

            # ── 参考表示用の GPS/地磁気ナビ情報 (最終判定には使わない) ──
            lat, lng       = st.g_gps_lat, st.g_gps_lng
            distance       = calc_distance(lat, lng)
            azimuth        = calc_azimuth_with_front(st.g_mag)
            target_bearing = calc_target_bearing(lat, lng)
            diff           = calc_direction_diff(azimuth, target_bearing)

            sonl_str = f"{dist_left:.3f}" if dist_left is not None else "---"
            sonr_str = f"{dist_right:.3f}" if dist_right is not None else "---"
            row = DAT.format(elapsed, lat, lng, distance, target_bearing,
                             azimuth, diff, sonl_str, sonr_str,
                             f"{motor_cmd}(→{favored}:{favored_str})")
            print(row, flush=True)

            if now - last_nav_report_time >= C.FINAL_NAV_REPORT_INTERVAL_SEC:
                last_nav_report_time = now
                turn_side = "右" if diff < 0 else ("左" if diff > 0 else "正面")
                log(f"[Phase5] 目標までの距離(GPS)={distance:.2f} m  "
                    f"左スキャン距離={sonl_str} m  右スキャン距離={sonr_str} m  "
                    f"→ {favored}方向へ接近  "
                    f"機体方位={azimuth:.1f}°  目標方位={target_bearing:.1f}°  "
                    f"ずれ={diff:+.1f}° ({turn_side})")
                log_data_status("最終接近演算 (左右スキャン超音波距離を主判定に使用)")

    except KeyboardInterrupt:
        log("[Phase5] Ctrl+C 受信 → 緊急停止")

    finally:
        motor.stop()
        print()
        log("─" * 62)
        if reached:
            log("[Phase5] 結果: 目標到達 (ミッション完了)")
        elif timed_out:
            log("[Phase5] 結果: タイムアウト")
        else:
            log("[Phase5] 結果: 中断")

    return reached


def phase5_camera_approach(sonar: SonarSensor, motor: MotorController,
                           led: LED, start_time: float,
                           camera=None, detector=None, bno=None,
                           allow_return_to_gps: bool = True):
    """
    ★追加: カメラ色検知を主に使う最終接近フェーズ。

    参考にした手法:
      * F.T.E.15th (NSE2025): 赤領域の重心 x で左右旋回/前進、見失ったら
        その場旋回で探索、画面に大きく写ったら到達
      * cansat-system: 明るさ適応の HSV 閾値、近距離では停止した状態で
        超音波を複数回測って中央値・ばらつきで 0m を判定、短いパルス前進

    1 ループ = 「停止 → 撮影・検知 → 1 パルスだけ動く」(stop-and-go)。
      ・ゴールが見えない : 最後に見えた側へ小刻みに旋回して探索。
                          1周 (bno があれば地磁気で 360° を確認) しても見えなければ
                          少し前進して視点を変え、Phase4 (GPS 誘導) に戻る
                          (allow_return_to_gps=False のときは Phase5 内で探索を続ける)。
      ・ゴールが見える   : 重心のずれに比例して旋回、正面なら前進。
      ・近距離           : カメラで接触寸前 or 正面 & 超音波が近い → confirm_goal()
          1. 停止して超音波 GOAL_SONAR_SAMPLES 回 + カメラ 1 枚を取り直し、
             gv.judge_goal() で判定
          2. CONFIRMED → GOAL_FINAL_PUSH_SEC だけ押し込んで接触
             BLIND (カメラは接触寸前なのに超音波が測れない = 不感帯の疑い)
                   → GOAL_BACKOFF_SEC 後退して測り直し、目の前にゴールが
                     あると超音波で確認できたら、後退分 + 押し込み分前進
          3. 押し込み後にもう一度超音波 + カメラで gv.judge_after_push()
             → OK なら 0m 到達 (ミッション完了)
    0m 到達を True とするのは、超音波でゴールを確認できたときだけ。

    カメラが初期化できない・撮影が CAM_FAIL_LIMIT 回続けて失敗する・
    CAM_SEARCH_MAX_LAPS 周探しても見つからない場合は、残り時間で
    従来の超音波左右スキャン (phase5_sonar_fallback) に切り替える。

    camera / detector はテスト用の差し替え口 (None なら実機用を生成)。
    bno: 探索で「1周したか」を地磁気で測るための BNO055 (None ならパルス回数で判定)。
    戻り値: True = 0m 到達 (超音波で確認済み), False = 未到達,
            PHASE5_RETURN_GPS = 1周してもゴールが見えないので Phase4 へ戻る
    """
    st.phase = 5
    blink_led(led)
    log("─" * 62)
    log("[Phase5] カメラ色検知による最終接近を開始します (0m 到達は超音波で確認)")
    log(f"  色: H={C.CAM_HUE_RANGES} S>={C.CAM_S_MIN} V>={C.CAM_V_MIN} (明るさで自動調整) "
        f"R-max(G,B)>={C.CAM_RED_MARGIN}")
    log(f"  近距離: 面積>={C.CAM_NEAR_AREA_RATIO:.2f} or 超音波<={C.GOAL_CHECK_TRIGGER_DIST_M:.2f}m  "
        f"ゴール: 超音波中央値<={C.GOAL_SONAR_DIST_M:.3f}m ({C.GOAL_SONAR_MIN_VALID}/{C.GOAL_SONAR_SAMPLES}回以上有効)")
    log(f"  タイムアウト: {C.FINAL_APPROACH_TIMEOUT_SEC:.0f} s  "
        f"1周の判定: {'地磁気で ' + str(int(C.CAM_SEARCH_LAP_DEG)) + '°' if bno is not None else f'旋回 {C.CAM_SEARCH_PULSES_PER_LAP} 回'}  "
        f"見つからないとき: {'少し前進して Phase4 (GPS) へ戻る' if allow_return_to_gps else 'Phase5 内で探索を続ける'}")
    log_data_status("Phase5開始時点")

    if detector is None:
        try:
            detector = make_goal_detector()
        except Exception as e:      # CAM_BACKEND="cv2" 指定で OpenCV が無い場合など
            log(f"[Phase5][WARN] 検知器の初期化に失敗 ({e}) → numpy 版で続行します", "WARN")
            detector = make_goal_detector(backend="numpy")
    log(f"  色検知バックエンド: {detector.backend}"
        + (" (形状評価あり)" if detector.backend == "cv2" else " (OpenCV なし: 形状評価なし)"))
    cfg = make_goal_check_config()
    if camera is None:
        camera = make_camera()

    p5_start = time.time()

    def remaining() -> float:
        return C.FINAL_APPROACH_TIMEOUT_SEC - (time.time() - p5_start)

    if not camera.start():
        log("[Phase5][WARN] カメラを初期化できません → 超音波スキャン方式に切り替えます", "WARN")
        camera.close()
        return phase5_sonar_fallback(sonar, motor, led, start_time, blink=False,
                                     timeout_sec=max(1.0, remaining()))

    # ── 撮影画像の保存先 (後で検知の様子を確認するため) ──
    img_dir = C.LOG_DIR / f"cam_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
    saved = {"n": 0, "ev": 0, "last": 0.0}   # n: 定期保存, ev: 重要な瞬間 (別枠)

    def save_frame(rgb, det, tag: str, force: bool = False):
        """★改善: 定期保存 (CAM_SAVE_INTERVAL_SEC ごと, 上限 CAM_SAVE_MAX) とは別枠で、
        force=True の重要な瞬間 (初検知・見失い・0m 確認) は CAM_SAVE_EVENT_MAX 枚まで必ず保存する。
        以前は両方合わせて上限に達すると、最後の到達確認の画像が残らないことがあった。"""
        if rgb is None:
            return
        now = time.time()
        if force:
            if saved["ev"] >= C.CAM_SAVE_EVENT_MAX:
                return
        else:
            if saved["n"] >= C.CAM_SAVE_MAX:
                return
            if C.CAM_SAVE_INTERVAL_SEC <= 0 or now - saved["last"] < C.CAM_SAVE_INTERVAL_SEC:
                return
        try:
            img_dir.mkdir(parents=True, exist_ok=True)
            idx = saved["n"] + saved["ev"]
            path = img_dir / f"{idx:03d}_{now - p5_start:06.1f}s_{tag}.jpg"
            gv.save_rgb_jpeg(rgb, path, det)
            if force:
                saved["ev"] += 1
            else:
                saved["n"] += 1
                saved["last"] = now
        except Exception as e:
            log(f"[Phase5][WARN] 画像保存失敗: {e}", "WARN")

    # ── 補助関数 ──
    def pulse(action: str, sec: float, note: str = ""):
        """1 パルスだけ動いて止まる。"""
        if action == "NONE" or sec <= 0:
            return
        cmd = {"LEFT": motor.turn_left_strong, "RIGHT": motor.turn_right_strong,
               "FORWARD": motor.forward, "BACKWARD": motor.backward}[action]
        cmd()
        log_sensor_row(time.time() - start_time, f"P5_{action}", f"{note} pulse={sec:.2f}s")
        time.sleep(sec)
        motor.stop()

    def observe(discard: int = 1):
        """停止状態で 1 枚撮影して検知する。戻り値 (rgb, det)。撮影失敗なら (None, None)。"""
        time.sleep(C.CAM_SETTLE_SEC)
        rgb = camera.capture_rgb(discard=discard)
        if rgb is None:
            return None, None
        return rgb, detector.detect(rgb)

    def sample_sonar(n: int) -> "gv.SonarSummary":
        vals = []
        for i in range(n):
            vals.append(sonar.get_distance_m())
            if i < n - 1:
                time.sleep(C.GOAL_SONAR_INTERVAL_SEC)
        st.g_sonar_m = vals[-1] if vals else None
        return gv.summarize_sonar(vals)

    def read_heading():
        """★追加: 停止中に地磁気を 3 回読んで平均し、方位 [度] を返す (探索の 1 周判定用)。
        モータ通電中は地磁気が乱れる (Phase4 の実機ログで確認済み) ため、必ず停止中に読む。
        読めなければ None。"""
        if bno is None:
            return None
        try:
            vs = [bno.getMag() for _ in range(3)]
            st.g_mag = [sum(v[i] for v in vs) / 3.0 for i in range(3)]
            mark_sensor_ok("BNO055", "Phase5 探索の方位")
            return calc_azimuth(st.g_mag)
        except Exception as e:
            mark_sensor_fail("BNO055", f"読み取りエラー: {e}")
            return None

    blind_failures = 0
    goal_reason = ""

    def confirm_goal() -> str:
        """
        停止して超音波 + カメラで 0m 到達を確認する。
        戻り値: "GOAL" / "FAR" / "OBSTACLE" / "NO_VISUAL" / "SONAR_INVALID" /
                "NOISY" / "BLIND_FAIL" / "PUSH_FAIL" / "CAMERA_FAIL"
        """
        nonlocal blind_failures, goal_reason
        motor.stop()
        time.sleep(C.GOAL_SETTLE_SEC)
        son = sample_sonar(C.GOAL_SONAR_SAMPLES)
        rgb, det = observe()
        if det is None:
            return "CAMERA_FAIL"
        status, reason = gv.judge_goal(son, det, cfg)
        log(f"[Phase5][GOAL確認] {status}: {reason} | {son.summary()} | {det.summary()}")
        log_sensor_row(time.time() - start_time, "P5_GOAL_CHECK", f"{status} {son.summary()}")
        save_frame(rgb, det, f"check_{status}", force=True)

        if status == "CONFIRMED":
            push_sec = C.GOAL_FINAL_PUSH_SEC
        elif status == "BLIND":
            # 至近距離で超音波が測れない → 少し下がって「目の前にゴールがあるか」を測り直す
            pulse("BACKWARD", C.GOAL_BACKOFF_SEC, "不感帯対策の後退")
            time.sleep(C.GOAL_SETTLE_SEC)
            son_b = sample_sonar(C.GOAL_SONAR_SAMPLES)
            rgb_b, det_b = observe()
            if det_b is None:
                return "CAMERA_FAIL"
            ok, reason_b = gv.judge_backoff(son_b, det_b, cfg)
            log(f"[Phase5][GOAL確認] 後退して再測定: {'OK' if ok else 'NG'}: {reason_b} | "
                f"{son_b.summary()} | {det_b.summary()}")
            save_frame(rgb_b, det_b, f"backoff_{'ok' if ok else 'ng'}", force=True)
            if not ok:
                blind_failures += 1
                return "BLIND_FAIL"
            push_sec = C.GOAL_BACKOFF_SEC + C.GOAL_FINAL_PUSH_SEC   # 下がった分 + 押し込み分
        else:
            return status

        # ── 最終押し込み (接触) → 押し込み後にもう一度確認 ──
        if push_sec > 0:
            pulse("FORWARD", push_sec, "最終押し込み")
        time.sleep(C.GOAL_SETTLE_SEC)
        son_p = sample_sonar(C.GOAL_SONAR_SAMPLES)
        rgb_p, det_p = observe()
        if det_p is None:
            det_p = gv.GoalDetection()   # カメラが撮れなくても超音波だけで判定できるようにする
        ok, reason_p = gv.judge_after_push(son_p, det_p, cfg)
        log(f"[Phase5][GOAL確認] 押し込み後の検証: {'OK' if ok else 'NG'}: {reason_p} | "
            f"{son_p.summary()} | {det_p.summary()}")
        save_frame(rgb_p, det_p, f"after_push_{'ok' if ok else 'ng'}", force=True)
        if ok:
            goal_reason = f"{reason} / {reason_p}"
            return "GOAL"
        return "PUSH_FAIL"

    def avoid_obstacle():
        """ゴール以外の障害物が正面にある: 旋回してから (前が空いていれば) 前進。"""
        log("[Phase5] 正面の障害物を回避します (旋回 → 前進)")
        pulse("RIGHT", C.CAM_OBSTACLE_TURN_SEC, "障害物回避の旋回")
        d = sonar.get_distance_m()
        if d is None or d > C.STUCK_SONAR_DIST_THRESH:
            pulse("FORWARD", C.CAM_OBSTACLE_FWD_SEC, "障害物回避の前進")

    HDR = (f"{'T[s]':>7}  {'Det':>3}  {'Area':>6}  {'cx':>5}  {'Sonar[m]':>8}  "
           f"{'State':>8}  {'Action':>16}")
    print()
    print("-" * len(HDR))
    print(HDR)
    print("-" * len(HDR))

    reached = timed_out = aborted = fallback = return_gps = False
    fallback_reason = ""
    contact_far = 0      # 「カメラ接触寸前 & 超音波は遠い」の連続回数
    lost = 0
    search_pulses = 0
    laps = 0
    lap_turned = 0.0         # 今の探索で地磁気から測った旋回角の合計 [度]
    lap_use_heading = False  # 今の探索で地磁気を使えているか
    prev_heading = None
    last_side = "RIGHT"
    cam_fail = 0
    prev_detected = False
    last_report = p5_start

    try:
        while True:
            if remaining() <= 0:
                log(f"[Phase5] タイムアウト ({C.FINAL_APPROACH_TIMEOUT_SEC:.0f} s)", "WARN")
                timed_out = True
                break

            rgb, det = observe()
            if det is None:
                cam_fail += 1
                log(f"[Phase5][WARN] 撮影失敗 ({cam_fail}/{C.CAM_FAIL_LIMIT})", "WARN")
                if cam_fail >= C.CAM_FAIL_LIMIT:
                    fallback, fallback_reason = True, "撮影が続けて失敗"
                    break
                time.sleep(0.2)
                continue
            cam_fail = 0

            st.g_sonar_m = sonar.get_distance_m()   # 走行判断の目安 (0m の判定には使わない)
            sonar_str = f"{st.g_sonar_m:.3f}" if st.g_sonar_m is not None else "---"
            state = "TRACK" if det.detected else "SEARCH"
            action_str = ""
            # ★改善: 初めて見えた / 見失った瞬間は別枠で保存 (後で検知の様子を確認するため)
            if det.detected and not prev_detected:
                save_frame(rgb, det, "found", force=True)
            elif prev_detected and not det.detected:
                save_frame(rgb, det, "lost", force=True)
            else:
                save_frame(rgb, det, state.lower())
            prev_detected = det.detected

            if det.detected:
                lost = 0
                search_pulses = 0
                laps = 0
                last_side = "RIGHT" if det.offset > 0 else "LEFT"
                centered = abs(det.offset) <= C.CAM_CENTER_TOL
                sonar_close = st.g_sonar_m is not None and st.g_sonar_m <= C.GOAL_CHECK_TRIGGER_DIST_M
                need_check = (det.is_contact
                              or (centered and sonar_close)
                              or (centered and det.is_near and st.g_sonar_m is None))
                status = None
                if need_check:
                    status = confirm_goal()
                    action_str = f"CHECK:{status}"
                    if status == "GOAL":
                        reached = True
                        break
                    contact_far = contact_far + 1 if (status == "FAR" and det.is_contact) else 0
                    if status == "OBSTACLE":
                        avoid_obstacle()
                    elif status == "BLIND_FAIL" and blind_failures > C.GOAL_MAX_BLIND_RETRIES:
                        log("[Phase5] 不感帯の確認に何度も失敗 → 少し下がって再接近します", "WARN")
                        pulse("BACKWARD", C.GOAL_REAPPROACH_BACK_SEC, "再接近のための後退")
                        blind_failures = 0
                    elif contact_far >= C.GOAL_MAX_CONTACT_FAR:
                        log("[Phase5] カメラは接触寸前なのに超音波は遠いままです "
                            "(超音波がゴールを捉えていない可能性) → 下がって再接近します", "WARN")
                        pulse("BACKWARD", C.GOAL_REAPPROACH_BACK_SEC, "再接近のための後退")
                        contact_far = 0
                        status = "REAPPROACH"
                if status is None or status in ("FAR", "SONAR_INVALID", "NOISY"):
                    action, sec = gv.decide_steer(
                        det, center_tol=C.CAM_CENTER_TOL,
                        turn_min_sec=C.CAM_TURN_MIN_SEC, turn_gain_sec=C.CAM_TURN_GAIN_SEC,
                        turn_max_sec=C.CAM_TURN_MAX_SEC,
                        fwd_far_sec=C.CAM_FWD_FAR_SEC, fwd_near_sec=C.CAM_FWD_NEAR_SEC)
                    if action == "FORWARD" and (sonar_close or status is not None):
                        sec = C.CAM_FWD_NEAR_SEC     # ゴール手前は慎重に
                    pulse(action, sec, f"追尾 cx={det.cx:.2f} area={det.area_ratio:.3f}")
                    action_str = (action_str + " " if action_str else "") + f"{action}:{sec:.2f}s"
            else:
                lost += 1
                if lost <= C.CAM_LOST_HOLD_FRAMES:
                    action_str = f"HOLD({lost})"
                else:
                    if search_pulses == 0:
                        # 探索 (1周) の開始: 地磁気で旋回角を測り始める
                        prev_heading = read_heading()
                        lap_use_heading = prev_heading is not None
                        lap_turned = 0.0
                    pulse(last_side, C.CAM_SEARCH_TURN_SEC, "探索旋回")
                    search_pulses += 1
                    if lap_use_heading:
                        h = read_heading()
                        if h is None:
                            lap_use_heading = False   # 途中で読めなくなったら回数で判定
                            log("[Phase5][WARN] 地磁気が読めないため、旋回回数で1周を判定します", "WARN")
                        else:
                            lap_turned += calc_direction_diff(h, prev_heading)
                            prev_heading = h
                    if lap_use_heading:
                        lap_done = (abs(lap_turned) >= C.CAM_SEARCH_LAP_DEG
                                    or search_pulses >= C.CAM_SEARCH_MAX_PULSES_PER_LAP)
                        action_str = f"SEARCH_{last_side}({abs(lap_turned):.0f}/{C.CAM_SEARCH_LAP_DEG:.0f}°)"
                    else:
                        lap_done = search_pulses >= C.CAM_SEARCH_PULSES_PER_LAP
                        action_str = f"SEARCH_{last_side}({search_pulses}/{C.CAM_SEARCH_PULSES_PER_LAP})"
                    if lap_done:
                        turned = (f"地磁気で {abs(lap_turned):.0f}° 旋回" if lap_use_heading
                                  else f"旋回 {search_pulses} 回")
                        search_pulses = 0
                        laps += 1
                        log(f"[Phase5] 1周探索してもゴールが見えません ({turned}, {laps}周目)", "WARN")
                        if not allow_return_to_gps and laps >= C.CAM_SEARCH_MAX_LAPS:
                            fallback, fallback_reason = True, f"{laps}周探索しても見つからない"
                            break
                        d = sonar.get_distance_m()
                        if d is None or d > C.STUCK_SONAR_DIST_THRESH:
                            pulse("FORWARD", C.CAM_SEARCH_REPOSITION_SEC, "視点を変えるための前進")
                        else:
                            pulse("RIGHT", C.CAM_OBSTACLE_TURN_SEC, "前方が塞がっているため旋回")
                        if allow_return_to_gps:
                            log("[Phase5] 少し移動しました → Phase4 (GPS 誘導) に戻ってゴールに近づき直します")
                            return_gps = True
                            break

            print(f"{time.time() - p5_start:>7.2f}  {int(det.detected):>3d}  {det.area_ratio:>6.3f}  "
                  f"{det.cx:>5.2f}  {sonar_str:>8}  {state:>8}  {action_str:>16}", flush=True)
            log_sensor_row(time.time() - start_time, f"P5_{state}",
                           f"{det.summary()} sonar={sonar_str} act={action_str}")

            if time.time() - last_report >= C.CAM_REPORT_INTERVAL_SEC:
                last_report = time.time()
                dist_gps = calc_distance(st.g_gps_lat, st.g_gps_lng) if st.g_gps_valid else float("nan")
                log(f"[Phase5] state={state} {det.summary()} sonar={sonar_str}m "
                    f"GPS距離(参考)={dist_gps:.1f}m 残り={remaining():.0f}s")
                log_data_status("カメラ最終接近 (カメラ=操舵, 超音波=0m確認)")

    except KeyboardInterrupt:
        log("[Phase5] Ctrl+C 受信 → 緊急停止")
        aborted = True

    finally:
        motor.stop()
        camera.close()       # フォールバックに入る前にカメラのメモリを解放する
        print()
        log("─" * 62)
        if reached:
            log(f"[Phase5] 結果: 0m 到達 (超音波で確認済み) — {goal_reason}")
        elif timed_out:
            log("[Phase5] 結果: タイムアウト (0m 到達を超音波で確認できず)")
        elif return_gps:
            log("[Phase5] 結果: ゴールが見つからない → Phase4 (GPS 誘導) へ戻る", "WARN")
        elif fallback:
            log(f"[Phase5] 結果: カメラ方式を中止 ({fallback_reason}) → 超音波スキャンへ", "WARN")
        else:
            log("[Phase5] 結果: 中断")
        if saved["n"] + saved["ev"]:
            log(f"[Phase5] 撮影画像 {saved['n'] + saved['ev']} 枚を保存 (定期 {saved['n']} / 重要 {saved['ev']}): {img_dir}")

    if return_gps:
        return C.PHASE5_RETURN_GPS
    if fallback and not aborted and remaining() > 5.0:
        return phase5_sonar_fallback(sonar, motor, led, start_time, blink=False,
                                     timeout_sec=remaining())
    return reached
