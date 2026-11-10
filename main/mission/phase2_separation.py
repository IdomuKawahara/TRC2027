"""
phase2_separation.py
====================
Phase 2: 超音波分離検知後退 → 停止 → 前進 (サブキャリア脱出)。
★ パラシュート分離に関わるため、このフェーズは変更しないこと。
(main6.py から機械的に分割。動作は変更していない)
元の位置: main6.py 1372〜1493 行
"""

from gpiozero import LED
import time

from . import config as C
from . import state as st
from .logger import blink_led, log, log_data_status, log_sensor_row
from .hardware import MotorController, SonarSensor

# ─── Phase 2: 超音波分離検知後退 → 停止 (5 秒) → 前進 (10 秒) ────────────────

def phase2_initial_backward(motor: MotorController, sonar: SonarSensor,
                            led: LED, start_time: float):
    """
    サブキャリア脱出のため後退する。

    ★変更: 従来は PHASE2_BACK_SEC (固定30秒) 後退していたが、分離タイミングは
    機体の落下姿勢や地面との接触状況で毎回変わるため、固定秒数では
    「まだ分離していないのに後退をやめる」「分離済みなのに無駄に後退を
    続ける」の両方のリスクがあった。

    そこで前方の超音波センサでサブキャリアとの距離を後退中も継続的に
    測定し、直前の有効値から PHASE2_SONAR_JUMP_THRESH_M [m] 以上距離が
    急増した瞬間 (=サブキャリアから機体が実際に分離した瞬間) を検知して、
    その場で後退を打ち切る。後退開始直後の PHASE2_BACK_MIN_SEC 秒間は
    センサ値が安定しない可能性があるため判定を保留し (誤検知防止)、
    超音波が長時間有効値を返さない場合や、分離が検知できないまま
    PHASE2_BACK_TIMEOUT_SEC を超えた場合は、安全のためタイムアウトとして
    後退を打ち切る。

    分離検知(またはタイムアウト)後、PHASE2_PAUSE_SEC 秒停止して切り替えの
    衝撃・振動を落ち着かせ、確実に離脱・前方へ距離を取るために
    PHASE2_FWD_AFTER_SEC 秒前進する。
    """
    st.phase = 2
    blink_led(led)
    log("─" * 62)
    log(f"[Phase2] 超音波分離検知後退を開始します")
    log(f"  判定: 直前値から {C.PHASE2_SONAR_JUMP_THRESH_M:.2f} m 以上の距離急増で分離とみなす "
        f"(判定開始まで {C.PHASE2_BACK_MIN_SEC:.0f} s のウォームアップ)")
    log(f"  安全タイムアウト: {C.PHASE2_BACK_TIMEOUT_SEC:.0f} s")
    log_data_status("Phase2開始時点")

    motor.backward()
    p2_start = time.time()
    prev_valid_dist = None      # 直前の有効な超音波距離 [m]
    last_sonar_valid_time = p2_start
    separated = False
    timed_out = False

    print(f"\n{'経過[s]':>8}  {'Sonar[m]':>9}  {'直前値[m]':>10}  {'差分[m]':>8}  {'判定':>10}")
    print("-" * 55)

    while True:
        now = time.time()
        elapsed = now - p2_start

        dist = sonar.get_distance_m()
        dist_str = f"{dist:.3f}" if dist is not None else "---"

        judge = "計測中"
        if dist is not None:
            last_sonar_valid_time = now
            if elapsed < C.PHASE2_BACK_MIN_SEC:
                judge = "ウォームアップ中"
            elif prev_valid_dist is not None:
                jump = dist - prev_valid_dist
                if jump >= C.PHASE2_SONAR_JUMP_THRESH_M:
                    judge = "分離検知!"
                    log(f"[Phase2] 超音波距離が急増 ({prev_valid_dist:.3f}m → {dist:.3f}m, "
                        f"差分={jump:+.3f}m) → サブキャリアから分離したと判定します")
                    separated = True
                else:
                    judge = "後退中"
            prev_valid_dist = dist
        else:
            judge = "センサ無効"
            if now - last_sonar_valid_time >= C.PHASE2_SONAR_LOST_TIMEOUT_SEC:
                log(f"[Phase2][WARN] 超音波センサが {C.PHASE2_SONAR_LOST_TIMEOUT_SEC:.0f}s "
                    f"以上有効値を返していません。タイムアウト判定のみで後退を継続します。", "WARN")
                last_sonar_valid_time = now  # 再警告を連発しないよう基準を更新

        diff_str = (f"{dist - prev_valid_dist:+.3f}"
                    if (dist is not None and prev_valid_dist is not None and dist != prev_valid_dist)
                    else "--")
        print(f"{elapsed:>8.2f}  {dist_str:>9}  "
              f"{(f'{prev_valid_dist:.3f}' if prev_valid_dist is not None else '---'):>10}  "
              f"{diff_str:>8}  {judge:>10}", flush=True)
        log_sensor_row(time.time() - start_time, "BACKWARD_PHASE2",
                       f"sonar={dist_str} judge={judge}")

        if separated:
            break

        if elapsed > C.PHASE2_BACK_TIMEOUT_SEC:
            log(f"[Phase2][WARN] 分離を検知できないまま {C.PHASE2_BACK_TIMEOUT_SEC:.0f}s "
                f"のタイムアウトに達しました。後退を打ち切ります。", "WARN")
            timed_out = True
            break

        time.sleep(C.PHASE2_LOOP_DT)

    motor.stop()
    if separated:
        log(f"[Phase2] 後退完了 → 停止 (超音波による分離検知, 所要 {elapsed:.1f} s)")
    elif timed_out:
        log(f"[Phase2] 後退完了 → 停止 (タイムアウト強制終了, 所要 {elapsed:.1f} s)", "WARN")

    # ── ★変更: 後退と前進の間に 5 秒間の停止を挟む ──
    log(f"[Phase2] 停止 {C.PHASE2_PAUSE_SEC:.0f} 秒 (後退→前進の切り替え待ち)")
    deadline_pause = time.time() + C.PHASE2_PAUSE_SEC
    while time.time() < deadline_pause:
        remaining = deadline_pause - time.time()
        log(f"  停止中... 残り {remaining:.1f} s")
        log_sensor_row(time.time() - start_time, "STOP_PHASE2")
        time.sleep(1.0)

    # ── 後退・停止後に前進 (サブキャリアから確実に離脱するため) ──
    log(f"[Phase2] 前進 {C.PHASE2_FWD_AFTER_SEC:.0f} 秒")
    motor.forward()
    deadline2 = time.time() + C.PHASE2_FWD_AFTER_SEC
    while time.time() < deadline2:
        remaining = deadline2 - time.time()
        log(f"  前進中... 残り {remaining:.1f} s")
        log_sensor_row(time.time() - start_time, "FORWARD_PHASE2")
        time.sleep(1.0)
    motor.stop()
    log("[Phase2] 前進完了 → 停止")
