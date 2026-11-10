"""
phase3_calibration.py
=====================
Phase 3: BNO055 キャリブレーション (静止⇔スピン) と GPS 安定化。
(main6.py から機械的に分割。動作は変更していない)
元の位置: main6.py 1494〜1749 行
"""

from gpiozero import LED
import math
import time
from BNO055 import BNO055
import bno_calib

from . import config as C
from . import state as st
from .state import mark_sensor_fail, mark_sensor_ok
from .logger import blink_led, log, log_data_status, log_sensor_row
from .hardware import MotorController
from .navigation import calc_distance, calc_target_bearing

# ─── Phase 3 補助: GPS Fix 安定化・平均化 (★追加) ───────────────────────────

def phase3_gps_stabilize(start_time: float):
    """
    GPS Fix の安定化・平均化。

    有効な Fix が得られ、衛星捕捉数が GPS_MIN_SATS 以上になるまで待機し
    (最大 GPS_CALIB_TIMEOUT_SEC 秒)、その後 GPS_CALIB_SAMPLE_SEC 秒間
    (最大 GPS_CALIB_SAMPLES 点) の緯度経度サンプルを収集して平均を取る。
    これにより単発の測位ノイズ・ふらつきの影響を減らし、誘導走行開始時点
    での目標までの距離・方位をログへ確認用に出力する。
    (誘導走行自体は毎ループ最新の GPS 値を使うため、この平均値は
     Phase4 の航法計算には使わず、あくまで開始前の状況確認・調整用)
    """
    log("─" * 62)
    log(f"[Phase3] GPS 安定化待機 (衛星数 >= {C.GPS_MIN_SATS}, タイムアウト {C.GPS_CALIB_TIMEOUT_SEC:.0f}s)")

    t_start = time.time()
    while True:
        elapsed = time.time() - t_start
        if st.g_gps_valid and st.g_gps_sats >= C.GPS_MIN_SATS:
            log(f"[Phase3] GPS Fix 取得 (衛星数={st.g_gps_sats}) → 位置サンプル収集を開始します")
            break
        if elapsed > C.GPS_CALIB_TIMEOUT_SEC:
            log("[Phase3] GPS 安定化タイムアウト。現状の Fix のまま続行します。", "WARN")
            break
        log(f"  GPS 待機中... valid={st.g_gps_valid} sats={st.g_gps_sats}  経過={elapsed:.1f}s")
        log_sensor_row(time.time() - start_time, "GPS_WAIT_CALIB")
        time.sleep(1.0)

    lat_samples, lng_samples = [], []
    t_sample_start = time.time()
    while (time.time() - t_sample_start < C.GPS_CALIB_SAMPLE_SEC
           and len(lat_samples) < C.GPS_CALIB_SAMPLES):
        if st.g_gps_valid:
            lat_samples.append(st.g_gps_lat)
            lng_samples.append(st.g_gps_lng)
        log_sensor_row(time.time() - start_time, "GPS_CALIB_SAMPLE")
        time.sleep(0.5)

    if lat_samples:
        avg_lat = sum(lat_samples) / len(lat_samples)
        avg_lng = sum(lng_samples) / len(lng_samples)
        dist    = calc_distance(avg_lat, avg_lng)
        bearing = calc_target_bearing(avg_lat, avg_lng)
        log(f"[Phase3] GPS 平均位置: LAT={avg_lat:.6f} LNG={avg_lng:.6f}  ({len(lat_samples)} サンプル)")
        log(f"[Phase3] 目標までの距離={dist:.2f} m  方位={bearing:.1f}°")
    else:
        log("[Phase3] 有効な GPS サンプルが取得できませんでした。生値のまま誘導走行に入ります。", "WARN")


# ─── Phase 3: キャリブレーション & 誘導準備 ──────────────────────────────────

def phase3_calibration(bno: BNO055, motor: MotorController, led: LED, start_time: float) -> float:
    """
    BNO055 キャリブレーション待機。

    【これまでの改良の経緯】
    (1) 以前は Acc/Gyro が揃うまで完全静止し続けてからスピン(Mag促進)に
        入る仕組みで、Acc が上がらずスピンが始まらないままタイムアウト
        していた問題を、静止⇔スピンのサイクル方式に変更して解消。
    (2) それでも Acc・Gyro のレベルが上がりきらずタイムアウトする事例が
        あったため、今回さらに以下の2点を追加改良した。

    【今回の追加改良】
    ① 軸ごとに現実的な目標レベルを個別設定 (CALIB_MIN_LEVEL_*)。
       地上機は複数姿勢を取れないため Acc の高レベル到達は現実的に
       難しく、また静止直後はモータ停止の慣性余韻で完全な静止に
       なっていない場合があり Gyro も上がりにくいことがある。
       これらは Sys/Mag ほど高いレベルを要求せず (デフォルト 1)、
       比較的到達しやすい Mag は従来通り高め (デフォルト 2) のまま
       とすることで、無駄な足踏みで CALIB_TIMEOUT_SEC に達するのを防ぐ。
    ② スピン直後の「静止」区間の頭に CALIB_SETTLE_SEC 秒の
       セトリング(慣性が収まるのを待つだけの捨て時間)を追加。
       モータを止めた直後は機体がまだわずかに揺れており、その揺れの
       間を「静止」としてカウントしてしまうと Gyro/Acc の内部
       キャリブレーションアルゴリズムが正しく収束しないため。

    完了後 (または CALIB_TIMEOUT_SEC タイムアウト後)、モータを停止し、
    静止時の水平加速度ノルムを基準値として計測する。
    続けて GPS Fix の安定化・平均化 (phase3_gps_stabilize) を行い、
    誘導走行の開始位置を確認してから終了する。
    """
    st.phase = 3
    blink_led(led)
    log("─" * 62)

    min_level = {
        "Sys":  C.CALIB_MIN_LEVEL_SYS,
        "Gyro": C.CALIB_MIN_LEVEL_GYRO,
        "Acc":  C.CALIB_MIN_LEVEL_ACC,
        "Mag":  C.CALIB_MIN_LEVEL_MAG,
    }
    log(f"[Phase3] キャリブレーション開始 目標レベル: "
        f"Sys>={min_level['Sys']} Gyro>={min_level['Gyro']} "
        f"Acc>={min_level['Acc']} Mag>={min_level['Mag']}")
    log("[Phase3] 静止(セトリング+Acc/Gyro) ⇔ スピン(Mag) のサイクルを繰り返します")
    # ★main6: プロファイルを書き戻せていれば補正値は既に入っているので、待ち時間を短くする
    calib_timeout = C.CALIB_TIMEOUT_WITH_PROFILE_SEC if st.g_bno_profile_loaded else C.CALIB_TIMEOUT_SEC
    if st.g_bno_profile_loaded:
        log(f"[Phase3] キャリブレーションプロファイル書き戻し済み → 待ち時間の上限を "
            f"{calib_timeout:.0f}s に短縮します")
    log_data_status("Phase3開始時点")

    p3_start = time.time()
    AXIS_NAMES = ("Sys", "Gyro", "Acc", "Mag")

    last_level        = {name: 0 for name in AXIS_NAMES}
    last_improve_time = {name: p3_start for name in AXIS_NAMES}
    last_report_time  = p3_start

    # ★追加: 静止フェーズの合計時間 = セトリング + 実質静止(計測)時間
    STILL_TOTAL_SEC = C.CALIB_SETTLE_SEC + C.CALIB_STILL_SEC

    state         = "STILL"     # "STILL" or "SPIN"
    next_spin_dir = "L"
    state_start   = p3_start
    motor.stop()
    motor_cmd = "STOP(settle)"
    log(f"[Phase3] STILL: セトリング{C.CALIB_SETTLE_SEC:.1f}s + 静止{C.CALIB_STILL_SEC:.0f}s (Acc/Gyro 安定化)")

    print(f"\n{'経過[s]':>8}  {'Sys':>4}  {'Gyro':>5}  {'Acc':>4}  {'Mag':>4}  {'状態':>12}  {'Motor':>16}")
    print("-" * 60)

    while True:
        try:
            st.g_acc   = bno.getAcc()
            st.g_mag   = bno.getMag()
            st.g_gyro  = bno.getGyro()
            st.g_calib = bno.getCalibrationStatus()
            mark_sensor_ok("BNO055", "キャリブレーション監視中")
        except Exception as e:
            mark_sensor_fail("BNO055", f"読み取りエラー: {e}")
            log(f"BNO055 エラー: {e}", "WARN")
            time.sleep(0.1)
            continue

        now     = time.time()
        elapsed = now - p3_start
        s, gy, ac, mg = st.g_calib
        cur = {"Sys": s, "Gyro": gy, "Acc": ac, "Mag": mg}

        # 停滞検知用: レベルが上がったら更新時刻をリセット
        for name in AXIS_NAMES:
            if cur[name] > last_level[name]:
                last_level[name] = cur[name]
                last_improve_time[name] = now

        # ── 静止(セトリング+計測) ⇔ スピン の状態遷移 ──
        state_elapsed = now - state_start
        if state == "STILL":
            motor_cmd = "STOP(settle)" if state_elapsed < C.CALIB_SETTLE_SEC else "STOP(Acc/Gyro)"
            if state_elapsed >= STILL_TOTAL_SEC:
                if C.FIG8_ENABLE:
                    if next_spin_dir == "L":
                        motor.turn_left_strong()
                        motor_cmd = "SPIN_L(Mag)"
                        next_spin_dir = "R"
                    else:
                        motor.turn_right_strong()
                        motor_cmd = "SPIN_R(Mag)"
                        next_spin_dir = "L"
                    state, state_start = "SPIN", now
                    log(f"[Phase3] {motor_cmd}: スピン {C.FIG8_SPIN_SEC:.0f} s (Mag 安定化)")
                else:
                    # FIG8 無効時は静止のまま (手動で回転させる運用を想定)
                    state_start = now
        elif state == "SPIN" and state_elapsed >= C.FIG8_SPIN_SEC:
            motor.stop()
            motor_cmd = "STOP(settle)"
            state, state_start = "STILL", now
            log(f"[Phase3] STILL: セトリング{C.CALIB_SETTLE_SEC:.1f}s + 静止{C.CALIB_STILL_SEC:.0f}s (Acc/Gyro 安定化)")

        # ── 定期進捗レポート ──
        if now - last_report_time >= C.CALIB_REPORT_INTERVAL_SEC:
            last_report_time = now
            log(f"[Phase3] 進捗: Sys={s} Gyro={gy} Acc={ac} Mag={mg}  "
                f"state={state}({motor_cmd})  経過={elapsed:.0f}s")
            log_data_status("キャリブレーション進捗判定 (BNO055 calib値使用)")

        # ── 軸別の停滞警告 ──
        for name in AXIS_NAMES:
            if cur[name] < min_level[name] and (now - last_improve_time[name]) >= C.CALIB_STALL_WARN_SEC:
                log(f"[Phase3][WARN] {name} 軸が {C.CALIB_STALL_WARN_SEC:.0f}s 以上レベル {cur[name]} "
                    f"(目標{min_level[name]}) のまま停滞しています", "WARN")
                last_improve_time[name] = now  # 連続警告を防ぐため基準時刻を更新

        all_ok = (s >= min_level["Sys"] and gy >= min_level["Gyro"]
                  and ac >= min_level["Acc"] and mg >= min_level["Mag"])

        if all_ok:
            status = "OK ✓"
        elif ac < min_level["Acc"]:
            status = "Acc 不足"
        elif mg < min_level["Mag"]:
            status = "Mag 不足"
        elif gy < min_level["Gyro"]:
            status = "Gyro 不足"
        else:
            status = "待機中..."

        print(f"{elapsed:>8.2f}  {s:>4d}  {gy:>5d}  {ac:>4d}  {mg:>4d}  {status:>12}  {motor_cmd:>16}", flush=True)

        if all_ok:
            log(f"[Phase3] キャリブレーション完了！ (所要 {elapsed:.1f} s)")
            break

        if elapsed > calib_timeout:
            log(f"[Phase3] タイムアウト ({calib_timeout:.0f}s)。現状 (Sys={s} Gyro={gy} Acc={ac} Mag={mg}) で続行します。"
                + (" (プロファイルの補正値は適用済み)" if st.g_bno_profile_loaded else ""), "WARN")
            break

        time.sleep(0.2)

    motor.stop()

    # ★main6: 地磁気・ジャイロが最高レベルなら、その補正値を保存しておく (次回の起動で使う)
    if C.BNO_CALIB_AUTOSAVE and st.g_calib[1] >= 3 and st.g_calib[3] >= 3:
        try:
            data = bno_calib.read_offsets(bno)
            if bno_calib.is_plausible(data):
                bno_calib.save_profile(C.BNO_CALIB_FILE, data, st.g_calib, note="Phase3 で自動保存")
                log(f"[Phase3] キャリブレーション結果を保存しました: {C.BNO_CALIB_FILE}")
        except Exception as e:
            log(f"[Phase3][WARN] キャリブレーション結果の保存に失敗: {e}", "WARN")

    # ── 静止時水平加速度の基準計測 ──
    log(f"[Phase3] 静止基準加速度を {C.CALIB_ACC_MEASURE_SEC:.0f} 秒計測...")
    samples = []
    t_end = time.time() + C.CALIB_ACC_MEASURE_SEC
    while time.time() < t_end:
        try:
            acc = bno.getAcc()
            # 水平面 (XY) の加速度ノルム
            horiz = math.sqrt(acc[0]**2 + acc[1]**2)
            samples.append(horiz)
        except Exception:
            pass
        time.sleep(0.05)

    if samples:
        baseline_horiz_acc = sum(samples) / len(samples)
    else:
        baseline_horiz_acc = 0.0
    log(f"[Phase3] 静止基準水平加速度 = {baseline_horiz_acc:.4f} m/s²")

    # ── GPS Fix 安定化・平均化 (緯度経度の調整) ──
    phase3_gps_stabilize(start_time)

    log("[Phase3] 誘導走行準備完了")
    return baseline_horiz_acc
