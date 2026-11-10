"""
stuck.py
========
Phase4 (GPS 誘導走行) のスタック検知と回避動作。

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
スタックには少なくとも 2 種類ある:
  STEP (段差・障害物型) : 前には進めないが、後ろに下がって向きを変えれば抜けられる
  RUT  (轍・はまり込み型): 前にも後ろにも動けず、その場で回ることもできない
                           (タイヤが溝に落ちている / 腹が地面に乗っている / 片輪が空転)

検知 (StuckDetector) ─ 走行ループから毎周期 update() を呼ぶ:
  A  前進指令中に 水平加速度が静止時並み & GPS 速度が低い (従来)
  B  超音波が近すぎる = 正面に壁・障害物 (従来)
  C  ★追加 前進指令が続いているのに GPS 位置が STUCK_NOPROGRESS_SEC 秒で
     STUCK_NOPROGRESS_DIST_M も動いていない (振動で A が成立しない空転を拾う)
  D  ★追加 走行指令中なのに GPS 速度が出ず、直近 STUCK_SPIN_WINDOW 周期のうち
     STUCK_SPIN_COUNT 周期以上でヨー角速度が大きい = 片輪だけ空転してその場で
     回っている → 轍型 (RUT) の疑い (誘導の旋回は長くても 3 秒程度なので区別できる)
  C は「前進だけ」ではなく「何らかの走行指令 (前進・旋回)」が続いているのに
  位置が変わらないことで判定する。片輪空転で向きが揺れて前進⇄旋回を繰り返す
  場合でも拾えるようにするため。

回避 (StuckRecovery.run) ─ 別スレッドで実行。回復中だけ後退を許可する:
  1. STEP 想定: 後退 STUCK_BACK_SEC 秒 → ジャイロで角度を測りながら
     STUCK_RECOVER_TURN_DEG 回転
     RUT 想定 : 「ツイスト」(左右のタイヤを互い違いに回す = 左ピボット⇄右ピボット
     を短いパルスで交互に) で溝の壁を崩し、「ロッキング」(前進⇄後退) で
     揺すってから、同じく回転を試みる
  2. 回転中にジャイロがほとんど動かなければ、タイヤが地面を掴めていない
     = RUT と判断して次のラウンドはツイストに切り替える (パルスは段階的に長く)
  3. 回れたら前進 STUCK_RECOVER_FWD_SEC 秒 (従来どおり再スタック監視つき。
     再スタックなら +STUCK_RECOVER_RETRY_TURN_DEG 回ってやり直し)
  4. STUCK_MAX_ROUNDS ラウンドで抜けられなければあきらめて誘導走行に戻る
     (Phase4 全体のタイムアウトで止まるのを防ぐため、無限には繰り返さない)

回転量は地磁気ではなくジャイロ (g_gyro[2]) の積分で測る。モータ通電中は
地磁気が大きく乱れることが実機ログで分かっているため。ジャイロが読めない
ときだけ従来どおり地磁気の変化で測る。
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import math
import threading
import time
from collections import deque

from . import config as C
from . import state as st
from .logger import log, log_sensor_row
from .navigation import calc_azimuth_with_front, calc_direction_diff, calc_distance_between


# ===========================================================================
# 検知
# ===========================================================================

class StuckDetector:
    def __init__(self, baseline_horiz_acc: float):
        self.baseline = baseline_horiz_acc
        self.count = 0            # 判定 A/B の連続回数 (従来の stuck_count)
        self.spin_win = deque(maxlen=C.STUCK_SPIN_WINDOW)   # 判定 D: 直近の周期で「回っている」か
        self.anchor = None        # 判定 C: (時刻, lat, lng) 走り始めた/最後に進んだ地点
        self.last_reason = ""
        self.kind_hint = None     # "RUT" (轍型の疑い) / None

    def reset(self):
        self.count = 0
        self.spin_win.clear()
        self.anchor = None
        self.kind_hint = None

    @property
    def spin_count(self) -> int:
        return sum(self.spin_win)

    def update(self, now: float, forward_intent: bool, horiz_acc: float,
               gps_valid: bool, gps_speed_mps: float, sonar_m, gyro_z_dps: float,
               lat: float, lng: float, driving: bool = None):
        """
        戻り値: (stuck_now, confirmed, reason)
          stuck_now : この周期でスタック条件 (A or B) を満たしている (従来の is_stuck)
          confirmed : スタック確定 (回避動作を始めてよい)
          reason    : "ACC" / "SONAR" / "NOPROGRESS" / "SPIN" / ""
          driving   : 何らかの走行指令 (前進・旋回) を出しているか (None なら forward_intent)
        """
        if driving is None:
            driving = forward_intent
        # ── A/B (従来と同じ条件・同じ閾値) ──
        is_slow_gps = (not gps_valid) or (gps_speed_mps < C.STUCK_GPS_SPEED_THRESH_MPS)
        is_stuck_acc = forward_intent and (horiz_acc < self.baseline + C.STUCK_HORIZON_ACCEL_THRESH) and is_slow_gps
        is_stuck_sonar = (sonar_m is not None and sonar_m < C.STUCK_SONAR_DIST_THRESH)
        stuck_now = is_stuck_acc or is_stuck_sonar
        if stuck_now:
            self.count += 1
        else:
            self.count = 0
        if self.count >= C.STUCK_COUNT_THRESHOLD:
            self.last_reason = "SONAR" if is_stuck_sonar else "ACC"
            return stuck_now, True, self.last_reason

        # ── D ★追加: 走行指令中なのに進まず、直近の窓の大半で回っている (片輪空転) ──
        self.spin_win.append(1 if (driving and is_slow_gps and abs(gyro_z_dps) >= C.STUCK_SPIN_GYRO_DPS) else 0)
        if not driving:
            self.spin_win.clear()
        if len(self.spin_win) == self.spin_win.maxlen and self.spin_count >= C.STUCK_SPIN_COUNT:
            self.last_reason = "SPIN"
            self.kind_hint = "RUT"
            return stuck_now, True, self.last_reason

        # ── C ★追加: 走行指令が続いているのに GPS 位置が進まない ──
        if driving and gps_valid:
            if self.anchor is None:
                self.anchor = (now, lat, lng)
            else:
                t0, lat0, lng0 = self.anchor
                moved = calc_distance_between(lat0, lng0, lat, lng)
                if moved >= C.STUCK_NOPROGRESS_DIST_M:
                    self.anchor = (now, lat, lng)
                elif now - t0 >= C.STUCK_NOPROGRESS_SEC:
                    self.last_reason = "NOPROGRESS"
                    return stuck_now, True, self.last_reason
        else:
            self.anchor = None

        return stuck_now, False, ""


# ===========================================================================
# 回避
# ===========================================================================

class StuckRecovery:
    """
    motor を使って回避動作を行う。センサ値は走行ループが更新している
    st.g_* を読むだけで、自分では BNO055 / 超音波に触れない
    (走行ループとの I2C 同時アクセスを避けるため。従来と同じ方針)。
    """

    def __init__(self, motor, start_time: float, baseline_horiz_acc: float):
        self.motor = motor
        self.start_time = start_time
        self.baseline = baseline_horiz_acc
        self._cancel = threading.Event()
        self.last_summary = ""

    def cancel(self):
        """Phase4 が終わるときに呼ぶ。進行中の回避動作を打ち切ってモータを止める。"""
        self._cancel.set()

    @property
    def cancelled(self) -> bool:
        return self._cancel.is_set()

    # ── 基本動作 ──────────────────────────────────────────────────────
    def _cmd(self, action: str):
        m = self.motor
        {"FORWARD": m.forward, "BACKWARD": m.backward,
         "LEFT": m.turn_left_strong, "RIGHT": m.turn_right_strong,
         "PIVOT_L": m.pivot_left, "PIVOT_R": m.pivot_right}[action]()

    def pulse(self, action: str, sec: float, note: str):
        """action を sec 秒だけ実行して止める。"""
        if self.cancelled or sec <= 0:
            return
        self._cmd(action)
        t_end = time.time() + sec
        while time.time() < t_end and not self.cancelled:
            log_sensor_row(time.time() - self.start_time, f"RECOVER_{action}", note)
            time.sleep(min(0.1, max(0.0, t_end - time.time())))
        self.motor.stop()
        time.sleep(C.STUCK_PAUSE_SEC)

    def rotate(self, direction: str, target_deg: float, note_prefix: str,
               early_check_sec: float = None, early_min_deg: float = None) -> float:
        """
        ジャイロの積分で角度を測りながら、direction ("L"/"R") へ target_deg 回る。
        early_check_sec 秒たっても early_min_deg 回れていなければ「回れない」と
        判断して打ち切る (轍型の判定に使う)。STUCK_RECOVER_TURN_TIMEOUT_SEC で
        安全に打ち切る。戻り値: 実際に回れた角度 [deg] (常に正)。
        """
        if self.cancelled:
            return 0.0
        self._cmd("LEFT" if direction == "L" else "RIGHT")
        note = f"{note_prefix}_{direction}"
        t_start = time.time()
        prev_t = t_start
        prev_az = calc_azimuth_with_front(st.g_mag)
        gyro_deg = 0.0
        mag_deg = 0.0
        gyro_alive = False
        turned = 0.0
        while not self.cancelled:
            now = time.time()
            dt = now - prev_t
            prev_t = now
            rate = st.g_gyro[2]
            if rate != 0.0:
                gyro_alive = True
            gyro_deg += abs(rate) * dt
            cur_az = calc_azimuth_with_front(st.g_mag)
            mag_deg += abs(calc_direction_diff(cur_az, prev_az))
            prev_az = cur_az
            turned = gyro_deg if gyro_alive else mag_deg
            elapsed = now - t_start
            log_sensor_row(now - self.start_time, note,
                           f"rot={turned:.1f}/{target_deg:.0f}deg ({'gyro' if gyro_alive else 'mag'})")
            if turned >= target_deg:
                break
            if elapsed > C.STUCK_RECOVER_TURN_TIMEOUT_SEC:
                log(f"[Phase4][STUCK] {note}: 回転タイムアウト "
                    f"({C.STUCK_RECOVER_TURN_TIMEOUT_SEC:.0f}s, 約{turned:.1f}度で打ち切り)", "WARN")
                break
            if (early_check_sec is not None and elapsed >= early_check_sec
                    and turned < early_min_deg):
                log(f"[Phase4][STUCK] {note}: {elapsed:.1f}s で {turned:.1f}° しか回れません "
                    f"(タイヤが地面を掴めていない)", "WARN")
                break
            time.sleep(0.1)
        self.motor.stop()
        time.sleep(C.STUCK_PAUSE_SEC)
        log(f"[Phase4][STUCK] {note}: 回転 目標{target_deg:.0f}° / 実測約{turned:.1f}° "
            f"({'ジャイロ' if gyro_alive else '地磁気'})")
        return turned

    def forward_with_monitor(self, duration_sec: float, note_prefix: str) -> bool:
        """
        duration_sec 秒前進しつつ、従来と同じ条件 (加速度・GPS 速度・超音波) で
        再スタックを監視する。再スタックなら True。
        """
        if self.cancelled:
            return False
        self.motor.forward()
        t_end = time.time() + duration_sec
        local_count = 0
        restuck = False
        while time.time() < t_end and not self.cancelled:
            horiz_acc = math.sqrt(st.g_acc[0] ** 2 + st.g_acc[1] ** 2)
            gps_speed_mps = st.g_gps_speed * C.KNOTS_TO_MPS
            is_slow_gps = (not st.g_gps_valid) or (gps_speed_mps < C.STUCK_GPS_SPEED_THRESH_MPS)
            is_stuck_acc = is_slow_gps and (horiz_acc < (self.baseline + C.STUCK_HORIZON_ACCEL_THRESH))
            is_stuck_sonar = (st.g_sonar_m is not None and st.g_sonar_m < C.STUCK_SONAR_DIST_THRESH)
            local_count = local_count + 1 if (is_stuck_acc or is_stuck_sonar) else 0
            log_sensor_row(time.time() - self.start_time, f"{note_prefix}_FORWARD",
                           f"remain={t_end - time.time():.1f}s stuck_cnt={local_count}")
            if local_count >= C.STUCK_COUNT_THRESHOLD:
                restuck = True
                break
            time.sleep(0.1)
        self.motor.stop()
        return restuck

    # ── 轍型の脱出動作 ───────────────────────────────────────────────
    def twist(self, pulse_sec: float, cycles: int, note: str):
        """左右のタイヤを互い違いに回す (左ピボット ⇄ 右ピボット)。
        溝の壁を崩す・タイヤの接地面を変えるのが目的。"""
        log(f"[Phase4][STUCK] ツイスト: ピボット左右交互 {pulse_sec:.2f}s × {cycles} 回")
        for i in range(cycles):
            if self.cancelled:
                return
            self.pulse("PIVOT_L" if i % 2 == 0 else "PIVOT_R", pulse_sec, f"{note} {i + 1}/{cycles}")

    def rock(self, sec: float, cycles: int, note: str):
        """前進 ⇄ 後退で揺する (反動で乗り越える)。"""
        log(f"[Phase4][STUCK] ロッキング: 前後 {sec:.2f}s × {cycles} 回")
        for i in range(cycles):
            if self.cancelled:
                return
            self.pulse("BACKWARD" if i % 2 == 0 else "FORWARD", sec, f"{note} {i + 1}/{cycles}")

    # ── 回避の本体 ──────────────────────────────────────────────────
    def run(self, last_turn_dir, reason: str, kind_hint=None) -> bool:
        """
        回避動作を行う。戻り値: True = 前進できた (抜けた), False = あきらめた/中断。
        last_turn_dir: スタック直前に旋回していた方向 ("L"/"R"/None)。反対側へ回る。
        """
        recover_dir = "R" if last_turn_dir == "L" else ("L" if last_turn_dir == "R" else "R")
        kind = kind_hint or "STEP"
        log(f"[Phase4][STUCK] スタック検知 (判定={reason}, 想定={kind}, 直前の旋回: {last_turn_dir or 'なし'}) "
            f"→ 回避開始。回復中は後退を許可します")
        self.motor.set_forward_only(False)
        escaped = False
        try:
            for rnd in range(1, C.STUCK_MAX_ROUNDS + 1):
                if self.cancelled:
                    break
                log(f"[Phase4][STUCK] ラウンド {rnd}/{C.STUCK_MAX_ROUNDS} ({kind})")
                if kind == "RUT":
                    grow = 1.0 + C.STUCK_TWIST_PULSE_GROWTH * (rnd - 1)
                    self.twist(C.STUCK_TWIST_PULSE_SEC * grow, C.STUCK_TWIST_CYCLES, f"TWIST{rnd}")
                    self.rock(C.STUCK_ROCK_SEC * grow, C.STUCK_ROCK_CYCLES, f"ROCK{rnd}")
                else:
                    self.pulse("BACKWARD", C.STUCK_BACK_SEC, f"BACK{rnd} 後退して離れる")

                turned = self.rotate(recover_dir, C.STUCK_RECOVER_TURN_DEG, f"RECOVER{rnd}",
                                     early_check_sec=C.STUCK_ROTATE_CHECK_SEC,
                                     early_min_deg=C.STUCK_ROTATE_MIN_DEG)
                if turned < C.STUCK_ROTATE_OK_FRAC * C.STUCK_RECOVER_TURN_DEG:
                    if kind != "RUT":
                        log("[Phase4][STUCK] 後退しても向きを変えられません → 轍型とみなしてツイストに切り替えます", "WARN")
                    kind = "RUT"
                    continue

                # ── 回れた → 前進して離れる (再スタックなら +90° して再試行) ──
                cur_dir = recover_dir
                for attempt in range(1, C.STUCK_RECOVER_MAX_RETRIES + 1):
                    restuck = self.forward_with_monitor(C.STUCK_RECOVER_FWD_SEC, f"RECOVER_ATT{attempt}")
                    if self.cancelled:
                        break
                    if not restuck:
                        escaped = True
                        break
                    if attempt >= C.STUCK_RECOVER_MAX_RETRIES:
                        log(f"[Phase4][STUCK] 前進中に再スタック: 最大試行回数 ({C.STUCK_RECOVER_MAX_RETRIES}) "
                            f"に達しました", "WARN")
                        break
                    cur_dir = "R" if cur_dir == "L" else "L"
                    log(f"[Phase4][STUCK] 前進中に再びスタック → {'左' if cur_dir == 'L' else '右'}へ "
                        f"{C.STUCK_RECOVER_RETRY_TURN_DEG:.0f}° 回って再試行 ({attempt + 1}/{C.STUCK_RECOVER_MAX_RETRIES})")
                    self.pulse("BACKWARD", C.STUCK_BACK_SEC, f"BACK_RETRY{attempt}")
                    t2 = self.rotate(cur_dir, C.STUCK_RECOVER_RETRY_TURN_DEG, f"RECOVER90_{attempt}",
                                     early_check_sec=C.STUCK_ROTATE_CHECK_SEC,
                                     early_min_deg=C.STUCK_ROTATE_MIN_DEG)
                    if t2 < C.STUCK_ROTATE_OK_FRAC * C.STUCK_RECOVER_RETRY_TURN_DEG:
                        kind = "RUT"
                        break
                if escaped:
                    break
                if kind != "RUT":
                    kind = "STEP"
        finally:
            self.motor.stop()
            self.motor.set_forward_only(True)
        if escaped:
            self.last_summary = f"回避成功 (判定={reason}, 最終={kind})"
            log(f"[Phase4][STUCK] 回避動作完了 → 誘導走行に戻ります")
        elif self.cancelled:
            self.last_summary = "回避中断"
            log("[Phase4][STUCK] 回避動作を中断しました (Phase4 終了)")
        else:
            self.last_summary = f"回避失敗 (判定={reason}, 最終={kind})"
            log(f"[Phase4][STUCK] {C.STUCK_MAX_ROUNDS} ラウンド試しても抜けられませんでした → "
                f"誘導走行に戻って様子を見ます", "WARN")
        return escaped
