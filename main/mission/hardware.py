"""
hardware.py
===========
ハードウェア: GPS 受信スレッド、超音波センサ (SonarSensor)、モータ (MotorController)、
BNO055 の Axis Remap (set_bno_orientation)。
(main6.py から機械的に分割。動作は変更していない)
元の位置: main6.py 738〜1031 行, 1132〜1190 行
"""

from gpiozero import Motor, PWMOutputDevice, OutputDevice, DigitalOutputDevice, DigitalInputDevice
import time
import threading
from BNO055 import BNO055
from micropyGPS import MicropyGPS

from . import config as C
from . import state as st
from .state import mark_sensor_fail, mark_sensor_ok
from .logger import log

# --- シリアル ----------------------------------------------------------------
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

# ===========================================================================
# GPS スレッド (test_GPSrun.py / test_finishv.py と同じ構造)
# ===========================================================================

def gps_thread_func(gps_obj: MicropyGPS):
    if not SERIAL_AVAILABLE:
        log("pyserial 未インストール。GPS 無効。", "WARN")
        return
    try:
        with serial.Serial(C.GPS_PORT, C.GPS_BAUDRATE, timeout=1.0) as ser:
            log(f"GPS Serial open: {C.GPS_PORT} @ {C.GPS_BAUDRATE}")
            ser.readline()  # 先頭不完全行を捨てる
            while True:
                try:
                    if ser.in_waiting > 128:
                        ser.reset_input_buffer()
                    line = ser.readline().decode("ascii", errors="replace")
                    if not line.startswith("$"):
                        continue
                    for c in line:
                        gps_obj.update(c)
                    if gps_obj.valid:
                        lr = gps_obj.latitude
                        lo = gps_obj.longitude
                        lat = lr[0] + lr[1] / 60.0
                        if lr[2] == 'S': lat = -lat
                        lng = lo[0] + lo[1] / 60.0
                        if lo[2] == 'W': lng = -lng
                        st.g_gps_lat   = lat
                        st.g_gps_lng   = lng
                        st.g_gps_speed = gps_obj.speed[0]
                        st.g_gps_course = gps_obj.course
                        st.g_gps_sats  = gps_obj.satellites_in_use
                        st.g_gps_valid = (lat != 0.0)
                        st.g_gps_fix_time = time.time()
                        if st.g_gps_valid:
                            mark_sensor_ok("GPS", f"sats={st.g_gps_sats}")
                        else:
                            mark_sensor_fail("GPS", "Fix未取得(lat=0)")
                except Exception as e:
                    mark_sensor_fail("GPS", f"読み取りエラー: {e}")
                    log(f"GPS 読み取りエラー: {e}", "WARN")
    except serial.SerialException as e:
        mark_sensor_fail("GPS", f"ポートエラー: {e}")
        log(f"GPS ポートエラー: {e}", "WARN")

# ===========================================================================
# 超音波センサ (sensor/HC-SR04.py と同じ原理, デバイス共有のためインライン実装)
# ===========================================================================

class SonarSensor:
    """HC-SR04 超音波距離センサ。測定失敗時は None を返す。"""
    def __init__(self):
        self._trig = DigitalOutputDevice(C.PIN_TRIG, initial_value=False)
        self._echo = DigitalInputDevice(C.PIN_ECHO)
        self._available = True
        log(f"超音波センサ初期化完了 TRIG=BCM{C.PIN_TRIG} ECHO=BCM{C.PIN_ECHO}")

    def get_distance_m(self) -> float | None:
        if not self._available:
            mark_sensor_fail("Sonar", "デバイス無効")
            return None
        try:
            self._trig.off()
            time.sleep(C.SONAR_SETTLE)
            self._trig.on()
            time.sleep(C.SONAR_PULSE)
            self._trig.off()

            t0 = time.time()
            while not self._echo.value:
                if time.time() - t0 > C.SONAR_TIMEOUT:
                    mark_sensor_fail("Sonar", "ECHO立ち上がりタイムアウト")
                    return None
            t_start = time.time()
            while self._echo.value:
                if time.time() - t_start > C.SONAR_TIMEOUT:
                    mark_sensor_fail("Sonar", "ECHO立ち下がりタイムアウト")
                    return None
            t_end = time.time()
            dist_m = (t_end - t_start) * C.SOUND_SPEED / 2.0
            mark_sensor_ok("Sonar", f"{dist_m:.2f}m")
            return dist_m
        except Exception as e:
            log(f"超音波センサエラー: {e}", "WARN")
            mark_sensor_fail("Sonar", f"エラー: {e}")
            self._available = False
            return None

    def close(self):
        self._trig.close()
        self._echo.close()

# ===========================================================================
# モータコントローラ (test_run.py / fall.py / test_GPSrun.py と統一)
# ===========================================================================

class MotorController:
    def __init__(self):
        self._pwm_a  = PWMOutputDevice(C.PIN_PWMA)
        self._pwm_b  = PWMOutputDevice(C.PIN_PWMB)
        self._mot_a  = Motor(forward=C.PIN_AIN1, backward=C.PIN_AIN2)   # 右
        self._mot_b  = Motor(forward=C.PIN_BIN1, backward=C.PIN_BIN2)   # 左
        self._stby   = OutputDevice(C.PIN_STBY)
        # ★削除: モータの forward/backward を丸ごと反転させる _reversed
        #        フラグは撤去した。モータは常に素直に forward=前進 /
        #        backward=後退として動作する (test_GPSrun.py と同じ、
        #        余計な補正を挟まないシンプルな設計)。

        # ★追加: 後退方向への回転を絶対に禁止する安全ロック。
        #        True の間は backward() が呼ばれても実際には後退させず、
        #        警告ログを出して停止のみ行う。Phase4 (誘導走行・
        #        スタック回復) のように「いかなる場合も後退してはならない」
        #        フェーズで set_forward_only(True) を呼んで有効化する想定。
        #        コード上のバグや将来の変更で誤って backward() が呼ばれても
        #        物理的に後退できないようにする、最後の防波堤として機能する。
        self._forward_only = False

        # ★追加: ソフトスタート (デューティ比 段階的引き上げ) 用の状態。
        #   バックグラウンドスレッド (_ramp_loop) が RAMP_UPDATE_INTERVAL_SEC
        #   ごとに「動き始めてからの経過時間」に応じたデューティ比を計算して
        #   PWM 出力へ反映し続ける。forward/backward/turn_* のどのメソッドを
        #   呼んでも同じ仕組みが自動的に働くため、各メソッド側は
        #   「今どちらの車輪をどの倍率で回すか (0.0〜1.0)」だけを指定すればよい。
        self._ramp_lock              = threading.Lock()
        self._move_start_time        = None   # 現在の連続動作の開始時刻 (None=停止中)
        self._duty_scale_a           = 1.0    # 右モータ: 現在デューティに対する倍率
        self._duty_scale_b           = 1.0    # 左モータ: 現在デューティに対する倍率
        self._last_duty_pct_reported = None   # ログ間引き用 (5%刻みでのみ報告)
        self._closed                 = False
        self._ramp_thread = threading.Thread(target=self._ramp_loop, daemon=True)
        self._ramp_thread.start()

        self.stop()
        log("モータ初期化完了")
        log(f"[Motor] ソフトスタート設定: {C.RAMP_START_DUTY*100:.0f}% → {C.RAMP_END_DUTY*100:.0f}% "
            f"を {C.RAMP_DURATION_SEC:.1f} 秒かけて引き上げ (更新周期 {C.RAMP_UPDATE_INTERVAL_SEC:.2f}s)")

    def set_forward_only(self, enabled: bool):
        """
        ★追加: 後退方向への回転を絶対に禁止する安全ロックを有効/無効にする。
        有効化 (True) すると、以降 backward() が呼ばれても実際には
        モータを後退方向へ一切回転させず、警告ログを出して stop() のみ
        行う (＝呼び出し元がミスをしても物理的に後退できない)。
        Phase4 (誘導走行・スタック回復) のように、後退動作が
        絶対に許されないフェーズの開始時に True、終了時に False へ戻す
        使い方を想定している。
        """
        state = "有効(backward()は無効化)" if enabled else "無効(通常通り)"
        log(f"[Motor] 後退禁止モードを{state}に設定")
        self._forward_only = enabled

    # ── ★追加: ソフトスタート (デューティ比 段階的引き上げ) ──────────────
    def _ramp_loop(self):
        """
        バックグラウンドで常時動き続けるループ。
        _move_start_time が設定されている (=モータが動作中) 間、
        RAMP_UPDATE_INTERVAL_SEC ごとに経過時間から現在のデューティ比を
        計算して PWM 出力 (_pwm_a / _pwm_b) へ反映する。
        stop() が呼ばれて _move_start_time が None に戻ると何もせず待機する。
        """
        while not self._closed:
            with self._ramp_lock:
                move_start = self._move_start_time
                scale_a    = self._duty_scale_a
                scale_b    = self._duty_scale_b

            if move_start is not None:
                elapsed = time.time() - move_start
                if elapsed >= C.RAMP_DURATION_SEC:
                    duty = C.RAMP_END_DUTY
                else:
                    duty = C.RAMP_START_DUTY + (C.RAMP_END_DUTY - C.RAMP_START_DUTY) * (elapsed / C.RAMP_DURATION_SEC)

                self._pwm_a.value = max(0.0, min(1.0, duty * scale_a))
                self._pwm_b.value = max(0.0, min(1.0, duty * scale_b))

                # ── 5%刻みでのみログ出力し、ログが埋まらないようにする ──
                pct = round(duty * 100)
                if pct != self._last_duty_pct_reported and pct % 5 == 0:
                    self._last_duty_pct_reported = pct
                    log(f"[Motor] ソフトスタート中: デューティ比 {pct}% "
                        f"(経過 {elapsed:.1f}/{C.RAMP_DURATION_SEC:.1f} s, "
                        f"右倍率={scale_a:.2f} 左倍率={scale_b:.2f})")

            time.sleep(C.RAMP_UPDATE_INTERVAL_SEC)

    def _start_move(self, scale_a: float, scale_b: float):
        """
        新しい動作を開始する際に呼ぶ。
        既に停止していた (_move_start_time is None) 場合は「今」を
        動き始めの基準時刻として記録し、ソフトスタートを RAMP_START_DUTY
        から開始する。stop() を挟まずに方向だけ変えた場合 (例: 前進中に
        微調整で弱旋回へ切り替わる等) は経過時間を引き継ぎ、
        デューティ比はそのまま引き上げを継続する。
        scale_a / scale_b は「現在のデューティ比に対する各モータの倍率」
        (0.0〜1.0) で、強旋回側は 1.0、弱旋回側は WEAK_RATIO、
        停止側は 0.0 を指定する。
        """
        with self._ramp_lock:
            if self._move_start_time is None:
                self._move_start_time = time.time()
                self._last_duty_pct_reported = None
                log(f"[Motor] 動作開始 → ソフトスタート {C.RAMP_START_DUTY*100:.0f}% から立ち上げます")
            self._duty_scale_a = scale_a
            self._duty_scale_b = scale_b
        self._stby.on()

    def forward(self):
        self._start_move(1.0, 1.0)
        self._mot_a.forward()
        self._mot_b.forward()

    def backward(self):
        # ★追加: 後退禁止ロック中は、実際には後退させず安全に停止するだけ。
        #        Phase4 等「絶対に後退してはならない」フェーズ中に
        #        誤って backward() が呼ばれても、物理的に後退できない
        #        ようにするための最終防波堤。
        if self._forward_only:
            log("[Motor][WARN] 後退禁止モード中に backward() が呼び出されました。"
                "安全のため後退はせず停止します。", "WARN")
            self.stop()
            return
        self._start_move(1.0, 1.0)
        self._mot_a.backward()
        self._mot_b.backward()

    def stop(self):
        with self._ramp_lock:
            self._move_start_time        = None
            self._duty_scale_a           = 1.0
            self._duty_scale_b           = 1.0
            self._last_duty_pct_reported = None
        self._pwm_a.value = 0
        self._pwm_b.value = 0
        self._mot_a.stop()
        self._mot_b.stop()
        self._stby.off()

    def turn_left_strong(self):
        """右モータ前進 / 左モータ停止 → 左旋回"""
        self._start_move(1.0, 0.0)
        self._mot_a.forward()
        self._mot_b.stop()

    def turn_right_strong(self):
        """右モータ停止 / 左モータ前進 → 右旋回"""
        self._start_move(0.0, 1.0)
        self._mot_a.stop()
        self._mot_b.forward()

    def turn_left_weak(self):
        self._start_move(1.0, C.WEAK_RATIO)
        self._mot_a.forward()
        self._mot_b.forward()

    def turn_right_weak(self):
        self._start_move(C.WEAK_RATIO, 1.0)
        self._mot_a.forward()
        self._mot_b.forward()

    def apply_diff(self, diff: float) -> str:
        """
        diff (−180〜+180) でモータ指令を自動選択。
        test_GPSrun.py の MotorController.apply_diff() をベースに、
        ★追加: 真逆 (|diff| が ANGLE_180_PREFER_RIGHT_DEG 以上、
        ほぼ180°) のときは、diff の符号 (+180寄りか-180寄りか) に
        左右されず必ず右回転 (turn_right_strong) を選ぶよう変更した。
        これにより、真逆を向いた状態でセンサノイズにより diff の符号が
        ±180の境界で頻繁に入れ替わっても、旋回方向が一意に定まり、
        左右への拮抗(その場で行ったり来たりして進めなくなる現象)を防ぐ。
        """
        if abs(diff) < C.ANGLE_DEADBAND:
            self.forward();           return "FORWARD"
        elif abs(diff) >= C.ANGLE_180_PREFER_RIGHT_DEG:
            self.turn_right_strong(); return "TURN_R_STRONG(180°反転優先)"
        elif diff > C.ANGLE_TURN_STRONG:
            self.turn_left_strong();  return "TURN_L_STRONG"
        elif diff > 0:
            self.turn_left_weak();    return "TURN_L_WEAK"
        elif diff < -C.ANGLE_TURN_STRONG:
            self.turn_right_strong(); return "TURN_R_STRONG"
        else:
            self.turn_right_weak();   return "TURN_R_WEAK"

    def close(self):
        self.stop()
        self._closed = True   # ★追加: バックグラウンドの _ramp_loop を終了させる
        self._pwm_a.close(); self._pwm_b.close()
        self._mot_a.close(); self._mot_b.close()
        self._stby.close()


def set_bno_orientation(bno: BNO055, orientation: int):
    """
    ★追加 (test_GPSrun.py より移植): BNO055 の Axis Remap 機能を使って、
    センサ内部の基準軸を機体の物理的な取り付け向きに合わせて再配置する。
    これにより、以降 bno.getMag() 等で得られる値は「機体前方が正しく
    軸に合った」状態になり、ソフトウェア側で前方オフセットを別途
    補正する必要がなくなる。

    設定は一度 CONFIG モードへ切り替えてレジスタ (0x41: 軸配置,
    0x42: 符号) を書き込み、その後 NDOF (フュージョン) モードへ戻す
    ことで反映される。ドライバの実装差異に対応するため、write_reg /
    set_mode / write_bytes / 直接 I2C アクセスのいずれかが使える方を
    試す (test_GPSrun.py と同一のフォールバック構造)。
    """
    remap_profiles = {
        1: (0x24, 0x00),  # P0: 0°  (正規)
        2: (0x21, 0x04),  # P1: 90°
        3: (0x24, 0x06),  # P2: 180°
        4: (0x21, 0x02),  # P3: 270°
    }

    if orientation not in remap_profiles:
        log(f"[BNO055][WARN] 無効な BNO_ORIENTATION ({orientation})。デフォルト(1)で動作します。", "WARN")
        orientation = 1

    config_val, sign_val = remap_profiles[orientation]

    # ★main6 修正: 以前は write_reg / set_mode / write_bytes / _i2c_bus のどれかが
    #   ある前提だったが、sensor/BNO055.py にはどれも無い (_write_byte / _set_mode /
    #   i2c がある) ため、何も書き込まれないまま「完了」とログに出ていた。
    #   今のドライバの関数で書き込み、書いた値を読み返して確認する。
    try:
        def write(reg, val):
            if hasattr(bno, '_write_byte'):
                bno._write_byte(reg, val)
            elif hasattr(bno, 'write_reg'):
                bno.write_reg(reg, val)
            else:
                raise RuntimeError("BNO055 ドライバに書き込み関数がありません")

        write(0x3D, 0x00)               # CONFIG モードへ
        time.sleep(0.05)
        write(0x41, config_val)
        write(0x42, sign_val)
        time.sleep(0.02)
        if hasattr(bno, '_read_byte'):
            got = (bno._read_byte(0x41), bno._read_byte(0x42))
            if got != (config_val, sign_val):
                log(f"[BNO055][WARN] Axis Remap の読み返しが一致しません "
                    f"(書いた値 {config_val:#04x},{sign_val:#04x} / 読んだ値 {got[0]:#04x},{got[1]:#04x})", "WARN")
        write(0x3D, 0x0C)               # NDOF (フュージョン) モードへ復帰
        time.sleep(0.05)

        degrees_map = {1: "0°(正規)", 2: "90°", 3: "180°", 4: "270°"}
        log(f"[BNO055] Axis Remap 完了: 設定 {orientation} ({degrees_map[orientation]})")

    except Exception as e:
        log(f"[BNO055][WARN] Axis Remap 設定時の警告: {e}", "WARN")
