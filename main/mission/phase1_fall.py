"""
phase1_fall.py
==============
Phase 1: 落下検知。
(main6.py から機械的に分割。動作は変更していない)
元の位置: main6.py 1313〜1371 行
"""

from gpiozero import LED
import math
import time
from BNO055 import BNO055

from . import config as C
from . import state as st
from .state import mark_sensor_fail, mark_sensor_ok
from .logger import blink_led, log, log_data_status, log_sensor_row

# ─── Phase 1: 落下検知 ──────────────────────────────────────────────────────

def phase1_fall_detection(bno: BNO055, led: LED, start_time: float) -> bool:
    st.phase = 1
    blink_led(led)
    log("─" * 62)
    log(f"[Phase1] 落下検知開始  閾値={C.FALL_THRESHOLD} m/s²  ×{C.FALL_COUNT_THRESHOLD}回連続")
    log(f"         タイムアウト={C.FALL_TIMEOUT_SEC} s")
    log_data_status("Phase1開始時点")

    fall_count = 0
    p1_start   = time.time()
    last_status_report = p1_start  # ★追加: データ取得状況の定期報告用

    # コンソールヘッダー
    print(f"\n{'経過[s]':>8}  {'acc_norm':>10}  {'fall_cnt':>8}  {'Calib(S/G/A/M)':>16}")
    print("-" * 50)

    while True:
        try:
            st.g_acc   = bno.getAcc()
            st.g_calib = bno.getCalibrationStatus()
            mark_sensor_ok("BNO055", f"acc_norm算出用")
        except Exception as e:
            mark_sensor_fail("BNO055", f"読み取りエラー: {e}")
            log(f"BNO055 読み取りエラー: {e}", "WARN")
            time.sleep(0.05)
            continue

        acc_norm = math.sqrt(st.g_acc[0]**2 + st.g_acc[1]**2 + st.g_acc[2]**2)
        elapsed  = time.time() - p1_start

        # 落下判定 (NICS2026/NOA.py phase0() と同じ構造)
        if acc_norm <= C.FALL_THRESHOLD:
            fall_count += 1
        else:
            fall_count = 0

        calib_str = f"{st.g_calib[0]}/{st.g_calib[1]}/{st.g_calib[2]}/{st.g_calib[3]}"
        print(f"{elapsed:>8.2f}  {acc_norm:>10.3f}  {fall_count:>8d}  {calib_str:>16}", flush=True)
        log_sensor_row(time.time() - start_time, "WAIT_FALL", f"norm={acc_norm:.3f} cnt={fall_count}")

        # ── ★追加: データ取得状況を定期的にログへ報告 ──
        if time.time() - last_status_report >= 5.0:
            last_status_report = time.time()
            log_data_status(f"落下判定演算中 (acc_norm={acc_norm:.3f}, 閾値={C.FALL_THRESHOLD})")

        if fall_count >= C.FALL_COUNT_THRESHOLD:
            log(f"[Phase1] 落下検知！ (連続 {fall_count} 回, {elapsed:.2f} s)")
            return True

        if elapsed > C.FALL_TIMEOUT_SEC:
            log(f"[Phase1] タイムアウト ({C.FALL_TIMEOUT_SEC} s) → 強制移行")
            return False

        time.sleep(0.05)
