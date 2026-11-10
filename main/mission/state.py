"""
state.py
========
フェーズ関数・スレッド間で共有する状態 (センサ最新値・現在フェーズ・ログバッファ・
センサ取得状況)。main6.py の「グローバル共有変数」セクションをそのまま移動。
他のモジュールからは `from . import state as st` して `st.g_acc` のように読み書きする。

元の位置: main6.py 218〜247 行, 251〜253 行, 266〜309 行
"""

import threading
import time

phase = 0               # 現在フェーズ番号

# センサ最新値
g_acc        = [0.0, 0.0, 0.0]
g_mag        = [0.0, 0.0, 0.0]
g_gyro       = [0.0, 0.0, 0.0]
g_calib      = (0, 0, 0, 0)    # (sys, gyro, accel, mag)
g_temp       = 0.0
g_pressure   = 0.0
g_altitude   = 0.0
g_gps_lat    = 0.0
g_gps_lng    = 0.0
g_gps_speed  = 0.0
g_gps_sats   = 0
g_gps_valid  = False
g_gps_course = 0.0              # ★main6: GPS の進行方位 (COG) [deg]
g_gps_fix_time = None           # ★main6: GPS の値が最後に更新された時刻 (time.time())
g_sonar_m    = None             # None = 測定失敗
g_heading_est = None            # ★main6: 方位の融合器 (Phase4 で作り、Phase4 に戻っても学習結果を引き継ぐ)
g_bno_profile_loaded = False    # ★main6: 起動時にキャリブレーションプロファイルを書き戻せたか
g_p4_camera = None              # ★main6: Phase4 でゴールを見つけたとき Phase5 に引き継ぐカメラ

# ★追加: 地磁気ベクトルの平滑化 (EMA: 指数移動平均) 状態。
#        実機ログ解析により、モータへ高デューティで通電している間、
#        地磁気センサの読み取り値が100ms間隔で数十〜100度以上も
#        ジャンプするほどのノイズ (モータ電流による磁気干渉と推定) が
#        発生し、これが原因で機体方位が暴れて GPS 誘導が発散
#        (目標から遠ざかる) することが確認された。calc_azimuth_with_front()
#        内でこの EMA フィルタを適用し、瞬間的なノイズの影響を抑える。
g_mag_ema = None   # [x, y, z] の平滑化済みベクトル。None = 未初期化 (最初のサンプルで初期化)

# ログバッファ
log_rows  = []
log_lock  = threading.Lock()

# センサ別のデータ取得状況トラッキング ──────────────────────────
# 「直近の読み取りが成功したか」「最後に成功したのはいつか」をセンサごとに
# 記録しておき、ログでひと目に状況がわかるようにする
# (例: GPS だけ Fix していない、Sonar が長時間 NG、等にすぐ気づける)。
SENSOR_NAMES = ("BNO055", "GPS", "Sonar")
g_sensor_status = {
    name: {"ok": False, "last_ok_time": None, "detail": "未取得"}
    for name in SENSOR_NAMES
}
g_sensor_status_lock = threading.Lock()

def mark_sensor_ok(name: str, detail: str = ""):
    """センサ読み取り成功時に状況を更新する。"""
    with g_sensor_status_lock:
        g_sensor_status[name] = {"ok": True, "last_ok_time": time.time(), "detail": detail}

def mark_sensor_fail(name: str, detail: str = ""):
    """センサ読み取り失敗時に状況を更新する (直近成功時刻は保持し、経過時間を追えるようにする)。"""
    with g_sensor_status_lock:
        prev = g_sensor_status.get(name, {})
        g_sensor_status[name] = {
            "ok": False,
            "last_ok_time": prev.get("last_ok_time"),
            "detail": detail,
        }

def format_sensor_status() -> str:
    """全センサの取得状況を 1 行にまとめた文字列を作る (ログ表示用)。"""
    now = time.time()
    parts = []
    with g_sensor_status_lock:
        snapshot = {k: dict(v) for k, v in g_sensor_status.items()}
    for name in SENSOR_NAMES:
        st = snapshot.get(name, {})
        if st.get("ok"):
            detail = st.get("detail", "")
            parts.append(f"{name}=OK" + (f"({detail})" if detail else ""))
        else:
            last = st.get("last_ok_time")
            if last is None:
                parts.append(f"{name}=未取得")
            else:
                parts.append(f"{name}=NG(最終成功 {now - last:.0f}s前)")
    return "  ".join(parts)
