"""
phase0_init.py
==============
Phase 0: 初期化 (センサ・モータ・LED・ログ・キャリブレーションプロファイル)。
(main6.py から機械的に分割。動作は変更していない)
元の位置: main6.py 1216〜1312 行
"""

from gpiozero import LED, Device
import sys
import os
import time
import threading
import datetime
from gpiozero.pins.lgpio import LGPIOFactory
from BNO055 import BNO055
from BMP180 import BMP180
from micropyGPS import MicropyGPS
import bno_calib

from . import config as C
from . import state as st
from .logger import log
from .hardware import MotorController, SonarSensor, gps_thread_func, set_bno_orientation

# ===========================================================================
# フェーズ関数
# ===========================================================================

# ─── Phase 0: 初期化 & SSH 切断耐性確立 ────────────────────────────────────

def phase0_init() -> tuple:
    """
    全センサ・モータを初期化して返す。
    nohup で起動されている前提で SSH 切断耐性は OS 側が担保する。
    ログディレクトリを作成し、ログパスを返す。
    """
    st.phase = 0
    log("=" * 62)
    log("  NSE2026 ミッションシーケンス 開始")
    log(f"  PID={os.getpid()}  (nohup で SSH 切断耐性あり)")
    log(f"  目標座標: LAT={C.TARGET_LAT}  LNG={C.TARGET_LNG}")
    log("=" * 62)

    # gpiozero バックエンド (全テストコードと統一)
    Device.pin_factory = LGPIOFactory()

    # ログ保存先
    C.LOG_DIR.mkdir(parents=True, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = C.LOG_DIR / f"mission_{ts}.csv"
    log(f"ログパス: {log_path}")

    # LED 初期化 (ミッション動作中は点灯)
    led = LED(C.LED_PIN)
    led.on()
    log(f"LED 点灯 (BCM{C.LED_PIN})")

    # BNO055
    log("BNO055 初期化中...")
    bno = BNO055()
    if not bno.setUp(operation_mode=BNO055.OPERATION_MODE_NDOF):
        log("BNO055 初期化失敗", "ERROR")
        sys.exit(1)

    # ★追加: Axis Remap により、機体の物理的な取り付け向きに合わせて
    #        BNO055 内部の基準軸を再配置する (test_GPSrun.py と同じ方式)。
    #        これにより以降 bno.getMag() 等で得られる方位は、ソフトウェア
    #        側で別途オフセットを補正する必要なく、そのまま機体前方基準
    #        として扱える。
    set_bno_orientation(bno, C.BNO_ORIENTATION)

    # ★main6: 事前に保存したキャリブレーションプロファイルを書き戻す
    #   (setUp のソフトウェアリセットで消えるので、必ず setUp・Axis Remap の後に行う)
    st.g_bno_profile_loaded = False
    if C.BNO_CALIB_LOAD:
        data, info = bno_calib.load_profile(C.BNO_CALIB_FILE)
        if data is None:
            log(f"[BNO055] キャリブレーションプロファイルなし ({info}: {C.BNO_CALIB_FILE}) "
                f"→ Phase3 で通常どおりキャリブレーションします", "WARN")
        else:
            try:
                bno_calib.write_offsets(bno, data)
                st.g_bno_profile_loaded = True
                log(f"[BNO055] キャリブレーションプロファイルを書き戻しました ({info})")
            except Exception as e:
                log(f"[BNO055][WARN] プロファイルの書き戻しに失敗: {e}", "WARN")

    # BMP180
    log("BMP180 初期化中...")
    bmp = BMP180(oss=3)
    if not bmp.setUp():
        log("BMP180 初期化失敗", "ERROR")
        sys.exit(1)

    # 基準気圧 (3 回平均)
    log("基準気圧取得中...")
    bp_list = []
    for _ in range(3):
        bmp.getTemperature()
        bp_list.append(bmp.getPressure())
        time.sleep(0.1)
    base_pressure = sum(bp_list) / len(bp_list)
    log(f"基準気圧 = {base_pressure:.2f} Pa")

    # GPS
    gps_obj = MicropyGPS(local_offset=9, location_formatting='ddm')
    gps_t = threading.Thread(target=gps_thread_func, args=(gps_obj,), daemon=True)
    gps_t.start()

    # 超音波センサ
    sonar = SonarSensor()

    # モータ
    motor = MotorController()

    log("[Phase0] 全デバイス初期化完了")
    return bno, bmp, base_pressure, sonar, motor, led, log_path
