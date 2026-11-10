"""
test_stuck_recovery.py
======================
Phase4 のスタック検知 (mission/stuck.py StuckDetector) と回避動作 (StuckRecovery) を、
モータ・センサを偽物にした簡易世界で確認する。実機のモータ・GPIO には触れない。

    python3 test/test_stuck_recovery.py

仮想世界の「罠」:
  step : 前進だけ効かない (段差・壁)。後退と旋回は効く。
         罠の地点から 0.5m 離れるか、向きが 45° 以上変われば抜ける
  rut  : 前進・後退・片輪旋回がすべて効かない (タイヤが溝に落ちた)。
         ピボット (左右逆回転) を合計 rut_strength 秒行うと抜ける
  spin : 前進指令で片輪だけ空転し、その場で回ってしまう (進まない)。rut と同じ抜け方
  shake: 進まないが振動で加速度は出る (空転)。GPS が進まないことでしか分からない
  wall : 超音波が 0.2m を返す壁。step と同じ抜け方
"""
import math
import sys
import threading
import types
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import mission_facade                     # noqa: E402  (sys.path 設定)
from mission import config as C           # noqa: E402
from mission import state as st           # noqa: E402
from mission import phase4_gps as P4      # noqa: E402
from mission import stuck as SK           # noqa: E402
from mission import logger as LG          # noqa: E402

M_PER_DEG_LAT = 111320.0


class Clock:
    """偽の時計。走行ループと回避スレッドの両方から sleep() が呼ばれるので、
    世界の更新と時刻の進みはロックで直列化する。"""
    def __init__(self):
        self.t = 1_000_000.0
        self.world = None
        self.lock = threading.Lock()

    def time(self):
        return self.t

    def sleep(self, dt):
        dt = max(0.0, float(dt))
        with self.lock:
            if self.world is not None:
                self.world.advance(dt)
            self.t += dt
        # 偽の sleep は実際には待たないので、回避スレッドが GIL を握ったまま何十周も進み、
        # 走行ループ (センサ値の更新役) が止まってしまう。実機の sleep と同じように
        # 必ず一度 GIL を手放して、走行ループにも周期が回るようにする。
        threading.Event().wait(0.0003)


CLOCK = Clock()


class World:
    """x=東, y=北 [m], 原点 = ゴール。heading [deg] 北=0 時計回り。"""
    V_FWD, V_PIVOT_ONE, R_ONE, R_PIVOT = 0.5, 0.10, 70.0, 120.0

    def __init__(self, x, y, heading, traps=(), mag_bias=0.0, gps_noise=0.3, seed=0):
        self.x, self.y, self.heading = x, y, heading
        self.traps = list(traps)          # dict(kind, x, y, r, strength)
        self.active = None
        self.trap_log = []
        self.mag_bias = mag_bias
        self.cmd = None
        self.t = 0.0
        self.last_fix = -1.0
        self.rng = np.random.default_rng(seed)
        self.gps_noise = gps_noise
        self.speed = 0.0
        self.yaw_rate = 0.0
        self.shaking = False
        self.pivot_time = 0.0
        self.history = []

    # ── 罠 ──
    def _enter_traps(self):
        if self.active is not None:
            return
        for tr in self.traps:
            if not tr.get("done") and math.hypot(self.x - tr["x"], self.y - tr["y"]) <= tr["r"]:
                self.active = dict(tr, enter_heading=self.heading, enter_x=self.x, enter_y=self.y)
                tr["done"] = True
                self.trap_log.append(("enter", tr["kind"], round(self.t, 1)))

    def _maybe_escape(self):
        a = self.active
        if a is None:
            return
        if a["kind"] in ("step", "wall"):
            moved = math.hypot(self.x - a["enter_x"], self.y - a["enter_y"])
            dh = abs((self.heading - a["enter_heading"] + 180) % 360 - 180)
            if moved >= 0.5 or dh >= 45:
                self.trap_log.append(("escape", a["kind"], round(self.t, 1)))
                self.active = None
        elif a["kind"] in ("rut", "spin", "shake"):
            if self.pivot_time >= a.get("strength", 3.0):
                self.trap_log.append(("escape", a["kind"], round(self.t, 1)))
                self.active = None
                self.pivot_time = 0.0

    def motion(self, cmd):
        """(前進速度, ヨーレート, 振動あり) を罠の状態に応じて返す。"""
        base = {None: (0, 0), "FORWARD": (self.V_FWD, 0), "BACKWARD": (-self.V_FWD, 0),
                "LEFT": (self.V_PIVOT_ONE, -self.R_ONE), "RIGHT": (self.V_PIVOT_ONE, self.R_ONE),
                "LEFT_WEAK": (0.45, -25.0), "RIGHT_WEAK": (0.45, 25.0),
                "PIVOT_L": (0, -self.R_PIVOT), "PIVOT_R": (0, self.R_PIVOT)}[cmd]
        a = self.active
        if a is None or cmd is None:
            return base[0], base[1], cmd is not None
        kind = a["kind"]
        if kind in ("step", "wall"):
            if cmd in ("FORWARD", "LEFT_WEAK", "RIGHT_WEAK"):
                return 0.0, 0.0, False
            return base[0], base[1], True
        if kind == "rut":
            if cmd in ("PIVOT_L", "PIVOT_R"):
                return 0.0, 0.0, True            # 動かないが振動はある
            return 0.0, 0.0, False
        if kind == "spin":
            # 左輪が穴に落ちて空転、右輪だけ接地している想定:
            #   前進 → 右輪だけ押すのでその場で回る / LEFT (右輪のみ駆動) → 同じく回る /
            #   RIGHT (左輪のみ駆動) → 空転して何も起きない / ピボット → 振動するだけ
            if cmd in ("FORWARD", "LEFT_WEAK", "RIGHT_WEAK"):
                return 0.0, 40.0, True
            if cmd == "LEFT":
                return 0.0, -self.R_ONE, True
            if cmd in ("PIVOT_L", "PIVOT_R"):
                return 0.0, 0.0, True
            return 0.0, 0.0, False
        if kind == "shake":
            if cmd in ("PIVOT_L", "PIVOT_R"):
                return 0.0, 0.0, True
            return 0.0, 0.0, cmd == "FORWARD"    # 空転: 振動はあるが進まない
        raise ValueError(kind)

    def advance(self, dt):
        n = max(1, int(dt / 0.02))
        h = dt / n
        for _ in range(n):
            self._enter_traps()
            v, r, shaking = self.motion(self.cmd)
            self.heading = (self.heading + r * h) % 360
            self.x += v * h * math.sin(math.radians(self.heading))
            self.y += v * h * math.cos(math.radians(self.heading))
            if self.active is not None and self.cmd in ("PIVOT_L", "PIVOT_R"):
                self.pivot_time += h
            self.speed, self.yaw_rate, self.shaking = abs(v), r, shaking
            self.t += h
            self._maybe_escape()
        self.update_gps()

    def update_gps(self):
        if self.t - self.last_fix < 1.0:
            return
        self.last_fix = self.t
        nx, ny = self.rng.normal(0, self.gps_noise, 2)
        st.g_gps_lat = C.TARGET_LAT + (self.y + ny) / M_PER_DEG_LAT
        st.g_gps_lng = C.TARGET_LNG + (self.x + nx) / (M_PER_DEG_LAT * math.cos(math.radians(C.TARGET_LAT)))
        st.g_gps_speed = self.speed / C.KNOTS_TO_MPS
        st.g_gps_course = (self.heading + self.rng.normal(0, 3)) % 360 if self.speed > 0 else 0.0
        st.g_gps_valid = True
        st.g_gps_sats = 8
        st.g_gps_fix_time = CLOCK.t

    def dist(self):
        return math.hypot(self.x, self.y)

    def sonar(self):
        a = self.active
        if a is not None and a["kind"] == "wall" and self.cmd in (None, "FORWARD"):
            return 0.2
        return 3.0


class SimMotor:
    def __init__(self, world):
        self.world = world
        self.forward_only = False
        self.history = []
        self.lock_violations = 0

    def _set(self, c):
        self.world.cmd = c
        if c and (not self.history or self.history[-1] != c):
            self.history.append(c)

    def forward(self): self._set("FORWARD")
    def stop(self): self._set(None)
    def turn_left_strong(self): self._set("LEFT")
    def turn_right_strong(self): self._set("RIGHT")
    def turn_left_weak(self): self._set("LEFT_WEAK")
    def turn_right_weak(self): self._set("RIGHT_WEAK")

    def backward(self):
        if self.forward_only:
            self.lock_violations += 1
            return self.stop()
        self._set("BACKWARD")

    def pivot_left(self):
        if self.forward_only:
            self.lock_violations += 1
            return self.stop()
        self._set("PIVOT_L")

    def pivot_right(self):
        if self.forward_only:
            self.lock_violations += 1
            return self.stop()
        self._set("PIVOT_R")

    def set_forward_only(self, v):
        self.forward_only = v

    apply_diff = mission_facade.hardware.MotorController.apply_diff


class SimBNO:
    BASE = 0.3

    def __init__(self, world):
        self.world = world

    def getAcc(self):
        a = self.BASE + (1.2 if self.world.shaking or self.world.speed > 0 else 0.0)
        return [a * 0.8, a * 0.6, 9.8]

    def getGyro(self): return [0.0, 0.0, self.world.yaw_rate]
    def getCalibrationStatus(self): return (3, 3, 0, 3)

    def getMag(self):
        a = math.radians(self.world.heading + self.world.mag_bias + 90.0 - C.MAG_DECLINATION)
        return [30 * math.cos(a), 30 * math.sin(a), -20.0]


class SimSonar:
    def __init__(self, world): self.world = world
    def get_distance_m(self): return self.world.sonar()
    def close(self): pass


class Phase4StuckBase(unittest.TestCase):
    def setUp(self):
        global CLOCK
        CLOCK = Clock()
        fake_time = types.SimpleNamespace(time=CLOCK.time, sleep=CLOCK.sleep)
        self.patched = []
        for mod in (P4, SK):
            self.patched.append((mod, "time", mod.time))
            mod.time = fake_time
        self.saved_cfg = {k: getattr(C, k) for k in ("TIMEOUT_SEC", "CAM_P4_ENABLE")}
        C.TIMEOUT_SEC = 600
        C.CAM_P4_ENABLE = False
        self.logs = []
        self.saved_log = {}
        for mod in (P4, SK, LG):
            self.saved_log[mod] = mod.log
            mod.log = lambda m, level="INFO": self.logs.append((level, m))
        self.saved_print = P4.__dict__.get("print")
        P4.print = lambda *a, **k: None
        P4.blink_led = lambda *a, **k: None
        st.g_heading_est = None
        st.g_mag_ema = None
        with st.log_lock:
            st.log_rows.clear()

    def tearDown(self):
        for mod, name, val in self.patched:
            setattr(mod, name, val)
        for k, v in self.saved_cfg.items():
            setattr(C, k, v)
        for mod, f in self.saved_log.items():
            mod.log = f
        del P4.print
        del P4.blink_led

    def run_p4(self, world, timeout=None):
        CLOCK.world = world
        world.update_gps()
        if timeout:
            C.TIMEOUT_SEC = timeout
        motor = SimMotor(world)
        res = P4.phase4_guided_run(SimBNO(world), SimSonar(world), motor, None, CLOCK.t, SimBNO.BASE)
        return res, motor

    def log_text(self):
        return "\n".join(m for _, m in self.logs)

    def recovery_actions(self, motor):
        return [c for c in motor.history if c in ("BACKWARD", "PIVOT_L", "PIVOT_R")]


class TestStuckScenarios(Phase4StuckBase):
    START = (-3.0, -14.0, 10.0)       # ゴールの南 14m、やや西。北向きにスタート

    def trap(self, kind, dy=-8.0, **kw):
        # スタート (-3,-14) → ゴール (0,0) の直線上に置く
        return dict(kind=kind, x=-3.0 + 3.0 * (dy + 14.0) / 14.0, y=dy, r=0.9, **kw)

    def test_step_back_then_turn(self):
        w = World(*self.START, traps=[self.trap("step")])
        res, motor = self.run_p4(w)
        self.assertEqual(res, "NEAR_GOAL", self.log_text()[-1500:])
        self.assertEqual([e[0] for e in w.trap_log], ["enter", "escape"])
        self.assertIn("判定=ACC", self.log_text())
        acts = self.recovery_actions(motor)
        self.assertEqual(acts[0], "BACKWARD", "段差型はまず後退")
        self.assertNotIn("PIVOT_L", acts, "段差型ではツイストしない")
        self.assertIn("回避動作完了", self.log_text())
        self.assertEqual(motor.lock_violations, 0, "後退禁止ロック中に後退していない")
        self.assertTrue(motor.forward_only is False)   # Phase4 終了時に解除

    def test_rut_needs_twist(self):
        w = World(*self.START, traps=[self.trap("rut", strength=2.5)])
        res, motor = self.run_p4(w)
        self.assertEqual(res, "NEAR_GOAL", self.log_text()[-1500:])
        acts = self.recovery_actions(motor)
        self.assertEqual(acts[0], "BACKWARD", "最初は段差型として後退を試す")
        self.assertIn("PIVOT_L", acts)
        self.assertIn("PIVOT_R", acts)
        self.assertIn("轍型とみなしてツイスト", self.log_text())
        self.assertIn("ツイスト", self.log_text())
        self.assertEqual([e[0] for e in w.trap_log], ["enter", "escape"])

    def test_spin_detected_by_gyro_goes_straight_to_twist(self):
        w = World(*self.START, traps=[self.trap("spin", strength=2.0)])
        res, motor = self.run_p4(w)
        self.assertEqual(res, "NEAR_GOAL", self.log_text()[-1500:])
        self.assertIn("判定=SPIN", self.log_text())
        acts = self.recovery_actions(motor)
        self.assertTrue(acts[0].startswith("PIVOT"), f"轍型の疑いなら最初からツイスト: {acts[:4]}")

    def test_shake_detected_by_no_progress(self):
        w = World(*self.START, traps=[self.trap("shake", strength=2.0)])
        res, motor = self.run_p4(w)
        self.assertEqual(res, "NEAR_GOAL", self.log_text()[-1500:])
        self.assertIn("判定=NOPROGRESS", self.log_text())
        self.assertIn("PIVOT_L", self.recovery_actions(motor))

    def test_wall_detected_by_sonar(self):
        w = World(*self.START, traps=[self.trap("wall")])
        res, motor = self.run_p4(w)
        self.assertEqual(res, "NEAR_GOAL", self.log_text()[-1500:])
        self.assertIn("判定=SONAR", self.log_text())
        self.assertEqual(self.recovery_actions(motor)[0], "BACKWARD")

    def test_two_traps_in_a_row(self):
        # 1 つ目: 経路上の段差。2 つ目: ゴールを囲む半径 2.6m の轍 (2m 圏に入るには必ず通る)
        w = World(*self.START, traps=[self.trap("step", dy=-10.0),
                                      dict(kind="rut", x=0.0, y=0.0, r=2.6, strength=2.0)])
        res, motor = self.run_p4(w)
        self.assertEqual(res, "NEAR_GOAL", self.log_text()[-1500:])
        self.assertEqual([e[0] for e in w.trap_log], ["enter", "escape", "enter", "escape"])
        self.assertEqual(self.log_text().count("回避動作完了"), 2)

    def test_unescapable_rut_gives_up_and_continues(self):
        w = World(*self.START, traps=[self.trap("rut", strength=9999)])
        res, motor = self.run_p4(w, timeout=150)
        self.assertEqual(res, "TIMEOUT")
        self.assertIn("ラウンド試しても抜けられませんでした", self.log_text())
        self.assertGreaterEqual(self.log_text().count("ラウンド 1/"), 2, "あきらめた後も誘導に戻って再度試みる")
        self.assertEqual(motor.lock_violations, 0)

    def test_no_false_stuck_on_open_field(self):
        w = World(-18.0, -14.0, 200.0, mag_bias=30.0)
        res, motor = self.run_p4(w)
        self.assertEqual(res, "NEAR_GOAL")
        self.assertNotIn("スタック検知", self.log_text())
        self.assertEqual(self.recovery_actions(motor), [])

    def test_timeout_during_recovery_cancels_thread(self):
        w = World(*self.START, traps=[self.trap("rut", strength=9999)])
        res, motor = self.run_p4(w, timeout=24)     # 到達 ~12s + 検知 4s → 回避の途中でタイムアウト
        self.assertEqual(res, "TIMEOUT")
        self.assertIn("スタック検知", self.log_text())
        self.assertIn("回避動作を中断しました", self.log_text())
        self.assertIsNone(w.cmd, "Phase4 を抜けたときモータが止まっている")
        self.assertFalse(motor.forward_only)


class TestStuckDetectorUnit(unittest.TestCase):
    def setUp(self):
        self.d = SK.StuckDetector(baseline_horiz_acc=0.3)
        self.lat, self.lng = C.TARGET_LAT, C.TARGET_LNG

    def upd(self, t, forward=True, acc=0.35, gps_valid=True, speed=0.0, sonar=3.0, gyro=0.0,
            lat=None, lng=None, driving=None):
        return self.d.update(t, forward, acc, gps_valid, speed, sonar, gyro,
                             self.lat if lat is None else lat, self.lng if lng is None else lng,
                             driving=driving)

    def test_acc_after_threshold(self):
        for i in range(C.STUCK_COUNT_THRESHOLD - 1):
            self.assertEqual(self.upd(i * 0.1)[1], False)
        now, conf, reason = self.upd(4.0)
        self.assertTrue(now and conf)
        self.assertEqual(reason, "ACC")

    def test_acc_not_while_turning_or_moving(self):
        for i in range(100):
            self.assertFalse(self.upd(i * 0.1, forward=False)[1])
        for i in range(100):
            self.assertFalse(self.upd(i * 0.1, acc=2.0, speed=0.5, lat=self.lat + i * 1e-5)[1])

    def test_sonar(self):
        for i in range(C.STUCK_COUNT_THRESHOLD):
            r = self.upd(i * 0.1, forward=False, sonar=0.2)
        self.assertEqual(r[2], "SONAR")

    def test_spin_window(self):
        # 前進⇄旋回を繰り返しながら回り続ける (片輪空転)。窓が埋まるまでは確定しない
        r = None
        for i in range(C.STUCK_SPIN_WINDOW - 1):
            r = self.upd(i * 0.1, forward=(i % 3 == 0), acc=2.0, gyro=40.0, driving=True)
            self.assertFalse(r[1], i)
        r = self.upd(6.0, forward=True, acc=2.0, gyro=40.0, driving=True)
        self.assertEqual(r[2], "SPIN")
        self.assertEqual(self.d.kind_hint, "RUT")

    def test_normal_turn_is_not_spin(self):
        # 誘導の 180° 旋回 (70deg/s × 2.6 s) を含む 6 秒間 → 確定しない
        for i in range(120):
            turning = 20 <= i < 46
            r = self.upd(i * 0.1, forward=not turning, acc=2.0, speed=0.5 if not turning else 0.0,
                         gyro=70.0 if turning else 0.0, driving=True,
                         lat=self.lat + i * 0.05 / M_PER_DEG_LAT)
            self.assertFalse(r[1], i)

    def test_no_progress(self):
        t = 0.0
        r = None
        while t < C.STUCK_NOPROGRESS_SEC + 0.2:
            r = self.upd(t, acc=2.0, speed=0.05)      # 振動あり・位置は動かない
            if r[1]:
                break
            t += 0.1
        self.assertEqual(r[2], "NOPROGRESS")
        self.assertGreaterEqual(t, C.STUCK_NOPROGRESS_SEC - 0.2)

    def test_progress_resets_anchor(self):
        for i in range(300):
            lat = self.lat + (i // 50) * 1.2 / M_PER_DEG_LAT       # 5 秒ごとに 1.2m 進む
            self.assertFalse(self.upd(i * 0.1, acc=2.0, speed=0.05, lat=lat)[1])

    def test_reset(self):
        for i in range(30):
            self.upd(i * 0.1)
        self.d.reset()
        self.assertEqual((self.d.count, self.d.spin_count, self.d.anchor, self.d.kind_hint), (0, 0, None, None))

    def test_no_progress_while_oscillating(self):
        # 前進⇄旋回を繰り返して位置が変わらない (走行指令は続いている) → NOPROGRESS
        r = None
        t = 0.0
        while t < C.STUCK_NOPROGRESS_SEC + 0.5:
            r = self.upd(t, forward=(int(t * 10) % 4 == 0), acc=2.0, speed=0.05, driving=True)
            if r[1]:
                break
            t += 0.1
        self.assertEqual(r[2], "NOPROGRESS")


class TestPivotLock(unittest.TestCase):
    def test_pivot_respects_forward_only(self):
        HW = mission_facade.hardware

        class Fake:
            def __init__(self): self.calls = []
            def forward(self): self.calls.append("f")
            def backward(self): self.calls.append("b")
            def stop(self): self.calls.append("s")

        m = object.__new__(HW.MotorController)
        m._mot_a, m._mot_b = Fake(), Fake()
        m._forward_only = True
        m._start_move = lambda a, b: None
        m.stop = lambda: (m._mot_a.stop(), m._mot_b.stop())
        logs = []
        saved = HW.log
        HW.log = lambda msg, level="INFO": logs.append(level)
        try:
            m.pivot_left(); m.pivot_right()
            self.assertEqual(m._mot_a.calls, ["s", "s"])
            self.assertEqual(logs, ["WARN", "WARN"])
            m._forward_only = False
            m.pivot_left()
            self.assertEqual((m._mot_a.calls[-1], m._mot_b.calls[-1]), ("f", "b"))
            m.pivot_right()
            self.assertEqual((m._mot_a.calls[-1], m._mot_b.calls[-1]), ("b", "f"))
        finally:
            HW.log = saved


if __name__ == "__main__":
    unittest.main(verbosity=2)
