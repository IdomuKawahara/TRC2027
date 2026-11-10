"""
logger.py
=========
ログ出力 (log / log_data_status / log_sensor_row / save_log) と LED 点滅 (blink_led)。
(main6.py から機械的に分割。動作は変更していない)
元の位置: main6.py 656〜736 行
"""

from gpiozero import LED
import time
import csv
import datetime
from pathlib import Path

from . import config as C
from . import state as st
from .state import format_sensor_status

# ===========================================================================
# ロガー
# ===========================================================================

def log(msg: str, level: str = "INFO"):
    """タイムスタンプ付きログを標準出力に出す。nohup 経由でファイルにも残る。
    どのフェーズで何が起きているか一目でわかるよう、フェーズ番号に加えて
    日本語名も併記する。"""
    ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
    phase_name = C.PHASE_NAMES.get(st.phase, "?")
    line = f"[{ts}][Phase{st.phase}:{phase_name}][{level}] {msg}"
    print(line, flush=True)

def log_data_status(context: str = ""):
    """
    ★追加: 現在の全センサのデータ取得状況を 1 行にまとめてログ出力する。
    「今どの演算をしているか」を示す context と併せて呼ぶことで、
    その演算がどのセンサ値に基づいているか・データが正常かがわかりやすくなる。
    """
    prefix = f"{context}: " if context else ""
    log(f"[DataStatus] {prefix}{format_sensor_status()}")

def log_sensor_row(elapsed: float, motor_cmd: str, note: str = ""):
    """センサ全値を 1 行ログとしてバッファに追記する。"""
    with st.log_lock:
        st.log_rows.append([
            round(elapsed, 3), st.phase,
            round(st.g_acc[0],  4), round(st.g_acc[1],  4), round(st.g_acc[2],  4),
            round(st.g_mag[0],  4), round(st.g_mag[1],  4), round(st.g_mag[2],  4),
            round(st.g_gyro[0], 4), round(st.g_gyro[1], 4), round(st.g_gyro[2], 4),
            st.g_calib[0], st.g_calib[1], st.g_calib[2], st.g_calib[3],
            round(st.g_temp,     3), round(st.g_pressure, 2), round(st.g_altitude, 3),
            round(st.g_gps_lat,  6), round(st.g_gps_lng,  6),
            round(st.g_gps_speed,3), int(st.g_gps_sats),
            round(st.g_sonar_m, 4) if st.g_sonar_m is not None else -1.0,
            motor_cmd, note,
        ])

def save_log(log_path: Path):
    csv_header = [
        "Time_s", "Phase",
        "AccX", "AccY", "AccZ",
        "MagX", "MagY", "MagZ",
        "GyroX", "GyroY", "GyroZ",
        "Cal_Sys", "Cal_Gyro", "Cal_Acc", "Cal_Mag",
        "Temp_C", "Pres_Pa", "Alt_m",
        "Lat", "Lng", "GPS_Speed_kts", "GPS_Sats",
        "Sonar_m",
        "Motor_cmd", "Note",
    ]
    with st.log_lock:
        rows = list(st.log_rows)
    try:
        with open(log_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(csv_header)
            writer.writerows(rows)
        log(f"CSV 保存完了: {log_path} ({len(rows)} 行)")
    except Exception as e:
        log(f"CSV 保存失敗: {e}", "ERROR")

# ===========================================================================
# LED 点滅 (フェーズ切り替え通知)
# ===========================================================================

def blink_led(led: LED, duration: float = C.LED_BLINK_SEC, interval: float = C.LED_BLINK_INTERVAL):
    """
    フェーズ切り替えのタイミングで LED を duration 秒間点滅させる。
    ブロッキング実装 (フェーズ開始を明示的に知らせてから本処理に入るため)。
    点滅終了後は常時点灯に戻す。
    """
    log(f"LED 点滅開始 ({duration:.0f} 秒) — フェーズ切り替え通知")
    t_end = time.time() + duration
    while time.time() < t_end:
        led.on()
        time.sleep(interval)
        led.off()
        time.sleep(interval)
    led.on()
    log("LED 点滅終了 → 常時点灯に復帰")
