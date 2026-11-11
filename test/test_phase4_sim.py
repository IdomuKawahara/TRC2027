"""
test_phase4_sim.py
==================
Phase4 (GPS 誘導) まわりのテスト (実機のモータ・GPIO・I2C・カメラには触れない)。

    python3 test/test_phase4_sim.py

  * BNO055 キャリブレーションプロファイルの読み書き (偽のレジスタ)
  * set_bno_orientation の修正 (Axis Remap が実際に書き込まれるか)
  * ConeLock / ConeWatcher
  * Phase4 の走行シミュレーション
      - 地磁気が 40° ずれていても、GPS COG との融合でずれを学習してまっすぐ近づく
      - GPS で近づいたらカメラでゴールを見つけて、2m より手前で Phase5 へ (CONE_FOUND)
      - GPS の目標方位と大きく違う方向の赤い物ではロックしない
  * Phase4 ⇄ Phase5 の流れ (タイムアウト時に Phase5 を試す / カメラの引き継ぎ)
"""
import math
import sys
import tempfile
import types
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "test"))
import mission_facade      # noqa: E402
# mission パッケージを「M.名前」で読み書きする (差し替えは全モジュールに効く)
M = mission_facade.MissionFacade()
import goal_vision as gv   # noqa: E402
import bno_calib           # noqa: E402

M_PER_DEG_LAT = 111320.0


# ===========================================================================
# 偽の BNO055 (レジスタ単位)
# ===========================================================================

class RegBNO:
    def __init__(self):
        self.regs = {}
        self.mode_log = []

    def _read_byte(self, reg):
        return self.regs.get(reg, 0)

    def _write_byte(self, reg, val):
        self.regs[reg] = val & 0xFF
        if reg == 0x3D:
            self.mode_log.append(val)

    def _set_mode(self, mode):
        self._write_byte(0x3D, mode)


class TestBnoCalib(unittest.TestCase):
    def sample(self):
        return bytes([0xF6, 0xFF, 0x12, 0x00, 0xE8, 0xFF,       # acc
                      0x40, 0x01, 0x9C, 0xFF, 0x20, 0x00,       # mag
                      0xFE, 0xFF, 0x01, 0x00, 0x00, 0x00,       # gyr
                      0xE8, 0x03, 0x8F, 0x02])                  # acc_r=1000, mag_r=655

    def test_write_then_read_roundtrip(self):
        bno = RegBNO()
        bno_calib.write_offsets(bno, self.sample())
        self.assertEqual(bno.mode_log, [0x00, 0x0C], "CONFIG で書いて NDOF に戻す")
        self.assertEqual(bno_calib.read_offsets(bno), self.sample())
        d = bno_calib.decode(self.sample())
        self.assertEqual((d["acc_x"], d["mag_x"], d["acc_radius"], d["mag_radius"]), (-10, 320, 1000, 655))

    def test_save_load_and_validation(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as d:
            p = Path(d) / "sub" / "calib.json"
            data, why = bno_calib.load_profile(p)
            self.assertIsNone(data)
            bno_calib.save_profile(p, self.sample(), (3, 3, 3, 3), note="test")
            data, why = bno_calib.load_profile(p)
            self.assertEqual(data, self.sample())
            self.assertIn("[3, 3, 3, 3]", why)
            p.write_text("{broken", encoding="utf-8")
            self.assertIsNone(bno_calib.load_profile(p)[0])
        self.assertFalse(bno_calib.is_plausible(bytes(22)))
        self.assertFalse(bno_calib.is_plausible(b"\xff" * 22))
        self.assertFalse(bno_calib.is_plausible(b"\x01" * 10))
        with self.assertRaises(ValueError):
            bno_calib.write_offsets(RegBNO(), b"\x00" * 3)


class TestAxisRemapFix(unittest.TestCase):
    def setUp(self):
        self._log = M.log
        self.logs = []
        M.log = lambda m, level="INFO": self.logs.append((level, m))

    def tearDown(self):
        M.log = self._log

    def test_remap_is_written_with_current_driver(self):
        for orient, expect in ((1, (0x24, 0x00)), (2, (0x21, 0x04)), (3, (0x24, 0x06)), (4, (0x21, 0x02))):
            with self.subTest(orient=orient):
                bno = RegBNO()
                M.set_bno_orientation(bno, orient)
                self.assertEqual((bno.regs.get(0x41), bno.regs.get(0x42)), expect)
                self.assertEqual(bno.mode_log, [0x00, 0x0C])
                self.assertFalse([m for lv, m in self.logs if lv == "WARN"])


# ===========================================================================
# ConeLock / ConeWatcher
# ===========================================================================

def det_at(cx, detected=True):
    return gv.GoalDetection(detected=detected, cx=cx, area_ratio=0.02)


class TestConeLock(unittest.TestCase):
    def test_locks_after_consecutive_consistent_frames(self):
        lock = gv.ConeLock(confirm_frames=3)
        self.assertFalse(lock.update(det_at(0.6), 90.0, 95.0, 8.0))
        self.assertFalse(lock.update(det_at(0.55), 95.0, 95.0, 8.0))
        self.assertTrue(lock.update(det_at(0.5), 100.0, 95.0, 8.0))
        self.assertAlmostEqual(lock.world_bearing, 100.0, delta=0.5)

    def test_lost_frame_resets(self):
        lock = gv.ConeLock(confirm_frames=3)
        lock.update(det_at(0.5), 0, 0, 8)
        lock.update(det_at(0.5), 0, 0, 8)
        self.assertFalse(lock.update(det_at(0.5, detected=False), 0, 0, 8))
        self.assertFalse(lock.update(det_at(0.5), 0, 0, 8))
        self.assertEqual(lock.count, 1)

    def test_inconsistent_direction_restarts_count(self):
        lock = gv.ConeLock(confirm_frames=3, consistency_deg=20)
        lock.update(det_at(0.5), 0, 0, 3)
        lock.update(det_at(0.5), 0, 0, 3)
        self.assertFalse(lock.update(det_at(0.5), 60, 0, 3))      # 急に 60° 違う方向
        self.assertEqual(lock.count, 1)

    def test_gps_bearing_check_only_when_far(self):
        lock = gv.ConeLock(confirm_frames=1, gps_bearing_tol_deg=60, gps_check_min_dist_m=4)
        self.assertFalse(lock.update(det_at(0.5), 180, 0, 8.0))   # 遠いのに目標と逆方向
        self.assertIn("GPS", lock.last_reason)
        self.assertTrue(lock.update(det_at(0.5), 180, 0, 3.0))    # 近ければ GPS 方位は見ない
        self.assertTrue(gv.ConeLock(confirm_frames=1).update(det_at(0.5), 180, 0, None))

    def test_wraparound(self):
        lock = gv.ConeLock(confirm_frames=2)
        lock.update(det_at(0.5), 355, 5, 8)
        self.assertTrue(lock.update(det_at(0.5), 5, 5, 8))


class FakeCam:
    def __init__(self, frames, start_ok=True):
        self.frames = list(frames)
        self.start_ok = start_ok
        self.closed = 0
        self.started = 0
        self.last_error = "fake"

    def start(self):
        self.started += 1
        return self.start_ok

    def capture_rgb(self, discard=1):
        return self.frames.pop(0) if self.frames else None

    def close(self):
        self.closed += 1


class TestConeWatcher(unittest.TestCase):
    def red_frame(self):
        img = np.zeros((240, 320, 3), dtype=np.uint8)
        img[:] = (60, 120, 45)
        img[80:180, 200:240] = (210, 30, 30)
        return img

    def test_unthreaded_interval_and_seq(self):
        t = [0.0]
        cam = FakeCam([self.red_frame()] * 5)
        w = gv.ConeWatcher(cam, gv.ColorGoalDetector(), interval=0.25, threaded=False, clock=lambda: t[0])
        self.assertTrue(w.start())
        det, ts, seq = w.latest()
        self.assertTrue(det.detected)
        self.assertEqual(seq, 1)
        t[0] = 0.1
        self.assertEqual(w.latest()[2], 1, "interval 未満なら撮り直さない")
        t[0] = 0.3
        self.assertEqual(w.latest()[2], 2)
        w.stop()
        self.assertEqual(cam.closed, 0, "stop はカメラを閉じない (Phase5 に引き継ぐ)")
        w.close()
        self.assertEqual(cam.closed, 1)

    def test_threaded_runs_and_stops(self):
        cam = FakeCam([self.red_frame()] * 50)
        w = gv.ConeWatcher(cam, gv.ColorGoalDetector(), interval=0.05, threaded=True)
        self.assertTrue(w.start())
        import time
        deadline = time.time() + 20
        while w.latest() is None and time.time() < deadline:
            time.sleep(0.05)
        self.assertIsNotNone(w.latest())
        w.close()
        self.assertIsNone(w._thread)

    def test_start_failure(self):
        w = gv.ConeWatcher(FakeCam([], start_ok=False), gv.ColorGoalDetector(), threaded=False)
        self.assertFalse(w.start())

    def test_camera_start_is_idempotent(self):
        cap = gv.PiCameraCapture(log=lambda m: None)
        cap.cam = object()                     # 起動済みのふり
        self.assertTrue(cap.start())


# ===========================================================================
# Phase4 の走行シミュレーション
# ===========================================================================

class Clock:
    def __init__(self, world=None):
        self.t = 1_000_000.0
        self.world = world

    def time(self):
        return self.t

    def sleep(self, dt):
        dt = max(0.0, float(dt))
        if self.world is not None:
            self.world.advance(dt)
        self.t += dt


class World2D:
    """平面の世界。原点 = 目標座標。x = 東 [m], y = 北 [m]。"""
    V_FWD, V_PIVOT, V_WEAK = 0.5, 0.10, 0.45        # [m/s]
    R_PIVOT, R_WEAK = 70.0, 25.0                     # [deg/s]

    def __init__(self, x, y, heading, mag_bias=0.0, gps_noise=0.3, seed=0):
        self.x, self.y, self.heading = x, y, heading
        self.mag_bias = mag_bias
        self.cmd = None
        self.t = 0.0
        self.last_fix = -1.0
        self.rng = np.random.default_rng(seed)
        self.gps_noise = gps_noise
        self.path = 0.0
        self.yaw_rate = 0.0
        self.speed = 0.0
        self.objects = [(0.0, 0.0)]                  # 赤い物の位置 (既定: ゴールのコーン)
        self.cam_frames = 0

    def motion(self):
        return {None: (0, 0), "FORWARD": (self.V_FWD, 0),
                "LEFT": (self.V_PIVOT, -self.R_PIVOT), "RIGHT": (self.V_PIVOT, self.R_PIVOT),
                "LEFT_WEAK": (self.V_WEAK, -self.R_WEAK), "RIGHT_WEAK": (self.V_WEAK, self.R_WEAK)}[self.cmd]

    def advance(self, dt):
        n = max(1, int(dt / 0.02))
        h = dt / n
        v, r = self.motion()
        for _ in range(n):
            self.heading = (self.heading + r * h) % 360
            self.x += v * h * math.sin(math.radians(self.heading))
            self.y += v * h * math.cos(math.radians(self.heading))
            self.path += v * h
        self.yaw_rate, self.speed = r, v
        self.t += dt
        self.update_gps()

    def update_gps(self):
        if self.t - self.last_fix < 1.0:
            return
        self.last_fix = self.t
        nx, ny = self.rng.normal(0, self.gps_noise, 2)
        M.g_gps_lat = M.TARGET_LAT + (self.y + ny) / M_PER_DEG_LAT
        M.g_gps_lng = M.TARGET_LNG + (self.x + nx) / (M_PER_DEG_LAT * math.cos(math.radians(M.TARGET_LAT)))
        M.g_gps_speed = self.speed / M.KNOTS_TO_MPS
        M.g_gps_course = (self.heading + self.rng.normal(0, 3)) % 360 if self.speed > 0 else 0.0
        M.g_gps_valid = True
        M.g_gps_sats = 8
        M.g_gps_fix_time = CLOCK.t + 0.0

    def dist(self):
        return math.hypot(self.x, self.y)

    def render(self):
        """前向きのカメラ画像 (320x240)。赤い物を高さ 0.7m × 幅 0.25m で描く。"""
        self.cam_frames += 1
        img = np.zeros((240, 320, 3), dtype=np.uint8)
        img[:] = (60, 120, 45)
        img[:80] = (150, 190, 240)
        f = 160 / math.tan(math.radians(gv.HFOV_DEG / 2))
        for ox, oy in self.objects:
            dx, dy = ox - self.x, oy - self.y
            d = math.hypot(dx, dy)
            rel = (math.degrees(math.atan2(dx, dy)) - self.heading + 180) % 360 - 180
            if abs(rel) > 40 or d < 0.05:
                continue
            cx = 160 + f * math.tan(math.radians(rel))
            w, hgt = f * 0.25 / d, f * 0.7 / d
            bottom = 120 + f * 0.1 / d
            x0, x1 = int(max(0, cx - w / 2)), int(min(320, cx + w / 2))
            y0, y1 = int(max(0, bottom - hgt)), int(min(240, bottom))
            if x1 > x0 and y1 > y0:
                img[y0:y1, x0:x1] = (210, 30, 30)
        return img


class SimMotor:
    def __init__(self, world):
        self.world = world
        self.history = []
        self.forward_only = False

    def _set(self, c):
        self.world.cmd = c
        if c and (not self.history or self.history[-1] != c):
            self.history.append(c)

    def forward(self): self._set("FORWARD")
    def backward(self): self._set(None)
    def stop(self): self._set(None)
    def turn_left_strong(self): self._set("LEFT")
    def turn_right_strong(self): self._set("RIGHT")
    def turn_left_weak(self): self._set("LEFT_WEAK")
    def turn_right_weak(self): self._set("RIGHT_WEAK")
    def set_forward_only(self, v): self.forward_only = v
    apply_diff = M.MotorController.apply_diff


class SimBNO:
    def __init__(self, world):
        self.world = world

    def getAcc(self): return [0.9, 0.4, 9.8]
    def getGyro(self): return [0.0, 0.0, self.world.yaw_rate]
    def getCalibrationStatus(self): return (3, 3, 0, 3)

    def getMag(self):
        a = math.radians(self.world.heading + self.world.mag_bias + 90.0 - M.MAG_DECLINATION)
        return [30 * math.cos(a), 30 * math.sin(a), -20.0]


class SimSonar:
    def get_distance_m(self): return 3.0
    def close(self): pass


class SimCam:
    def __init__(self, world):
        self.world = world
        self.closed = 0
        self.last_error = None

    def start(self): return True
    def capture_rgb(self, discard=1): return self.world.render()
    def close(self): self.closed += 1


CLOCK = None


class Phase4SimBase(unittest.TestCase):
    SAVE = ("time", "blink_led", "log", "print", "TIMEOUT_SEC", "CAM_P4_ENABLE", "CAM_P4_THREADED",
            "make_camera", "g_heading_est", "g_mag_ema", "HEADING_COG_MIN_SPEED_MPS")

    def setUp(self):
        global CLOCK
        self._saved = {k: getattr(M, k) for k in self.SAVE if hasattr(M, k)}
        self.logs = []
        M.log = lambda m, level="INFO": self.logs.append((level, m))
        M.blink_led = lambda *a, **k: None
        M.print = lambda *a, **k: None
        M.TIMEOUT_SEC = 400
        M.CAM_P4_THREADED = False
        M.g_heading_est = None
        M.g_mag_ema = None
        CLOCK = Clock()
        M.time = types.SimpleNamespace(time=CLOCK.time, sleep=CLOCK.sleep)
        with M.log_lock:
            M.log_rows.clear()

    def tearDown(self):
        for k, v in self._saved.items():
            setattr(M, k, v)
        if "print" not in self._saved:
            del M.print

    def run_p4(self, world, cam=True):
        CLOCK.world = world
        world.update_gps()
        M.CAM_P4_ENABLE = cam
        self.cam = SimCam(world)
        M.make_camera = lambda: self.cam
        motor = SimMotor(world)
        res = M.phase4_guided_run(SimBNO(world), SimSonar(), motor, None, CLOCK.t, 0.0)
        return res, motor

    def log_text(self):
        return "\n".join(m for _, m in self.logs)


class TestPhase4Sim(Phase4SimBase):
    def test_mag_bias_is_learned_and_path_is_short(self):
        start = (-18.0, -14.0)                                       # 約 22.8 m
        w = World2D(*start, heading=200.0, mag_bias=40.0)
        res, motor = self.run_p4(w, cam=False)
        self.assertEqual(res, "NEAR_GOAL", self.log_text()[-1500:])
        est = M.g_heading_est
        self.assertTrue(est.offset_ready)
        self.assertAlmostEqual(est.offset_deg, -40.0, delta=8.0)
        self.path_fused = w.path
        straight = math.hypot(*start)
        self.assertLess(w.path, straight * 1.5 + 6.0, f"path={w.path:.1f} m")
        self.assertIn("COG", self.log_text())

    def test_fusion_beats_mag_only(self):
        start = (-18.0, -14.0)
        w1 = World2D(*start, heading=200.0, mag_bias=40.0)
        r1, _ = self.run_p4(w1, cam=False)
        t1 = CLOCK.t
        self.tearDown(); self.setUp()
        M.HEADING_COG_MIN_SPEED_MPS = 99.0                         # COG を使わない = 地磁気のみ
        w2 = World2D(*start, heading=200.0, mag_bias=40.0)
        t0 = CLOCK.t
        r2, _ = self.run_p4(w2, cam=False)
        self.assertEqual(r1, "NEAR_GOAL")
        self.assertLess(w1.path, w2.path, f"融合 {w1.path:.1f} m / 地磁気のみ {w2.path:.1f} m")
        self.assertLess(t1 - 1_000_000.0, CLOCK.t - t0)

    def test_cone_found_before_2m(self):
        w = World2D(-6.0, -9.0, heading=30.0)                      # 約 10.8 m
        res, motor = self.run_p4(w, cam=True)
        self.assertEqual(res, "CONE_FOUND", self.log_text()[-1500:])
        self.assertGreater(w.dist(), M.PHASE4_TO_5_RADIUS)
        self.assertIs(M.g_p4_camera, self.cam, "カメラを Phase5 に引き継ぐ")
        self.assertEqual(self.cam.closed, 0)
        self.assertIn("カメラでゴールを確定しました", self.log_text())

    def test_red_object_off_bearing_not_locked(self):
        # 目標 (方位約 35°) と逆を向いた状態から旋回を始める。旋回の途中で、
        # 目標とは約 85° 違う方位 120° の 6m 先にある赤い物がカメラに映る。
        # コーンは置かない。
        sx, sy = -5.0, -7.0
        w = World2D(sx, sy, heading=200.0)
        w.objects = [(sx + 6.0 * math.sin(math.radians(120)), sy + 6.0 * math.cos(math.radians(120)))]
        res, _ = self.run_p4(w, cam=True)
        self.assertEqual(res, "NEAR_GOAL")
        self.assertIsNone(M.g_p4_camera)
        self.assertEqual(self.cam.closed, 1, "Phase5 に引き継がないカメラは閉じる")
        notes = [r[-1] for r in M.log_rows if r[-2] == "P4_CONE"]
        self.assertTrue(notes, "赤い物は実際にカメラに映っている")
        self.assertTrue(any("GPS の目標方位" in n for n in notes), notes[:3])

    def test_camera_not_started_when_far(self):
        w = World2D(-30.0, -30.0, heading=45.0)
        M.TIMEOUT_SEC = 20
        res, _ = self.run_p4(w, cam=True)
        self.assertEqual(res, "TIMEOUT")
        self.assertEqual(w.cam_frames, 0)


class TestPhase4And5Flow(unittest.TestCase):
    def setUp(self):
        self._saved = {k: getattr(M, k) for k in ("phase4_guided_run", "phase5_camera_approach", "log",
                                                  "g_gps_lat", "g_gps_lng", "g_gps_valid", "g_p4_camera")}
        M.log = lambda *a, **k: None
        self.p5_calls = []

    def tearDown(self):
        for k, v in self._saved.items():
            setattr(M, k, v)

    def _run(self, p4_results, p5_results, dist_m=5.0, cam=None):
        p4, p5 = iter(p4_results), iter(p5_results)
        M.g_gps_valid = True
        M.g_gps_lat = M.TARGET_LAT + dist_m / M_PER_DEG_LAT
        M.g_gps_lng = M.TARGET_LNG

        def fake_p4(*a, **k):
            r = next(p4)
            M.g_p4_camera = cam if r == "CONE_FOUND" else None
            return r

        def fake_p5(*a, camera=None, allow_return_to_gps=True, **k):
            self.p5_calls.append((camera, allow_return_to_gps))
            return next(p5)
        M.phase4_guided_run, M.phase5_camera_approach = fake_p4, fake_p5
        return M.run_phase4_and_5("BNO", None, None, None, 0.0, 0.0)

    def test_cone_found_passes_camera(self):
        r = self._run(["CONE_FOUND"], [True], cam="CAM")
        self.assertIs(r, True)
        self.assertEqual(self.p5_calls, [("CAM", True)])

    def test_timeout_near_goal_tries_phase5_without_return(self):
        r = self._run(["TIMEOUT"], [False], dist_m=12.0)
        self.assertIs(r, False)
        self.assertEqual(self.p5_calls, [(None, False)])

    def test_timeout_far_skips_phase5(self):
        r = self._run(["TIMEOUT"], [], dist_m=50.0)
        self.assertIsNone(r)
        self.assertEqual(self.p5_calls, [])

    def test_abort_skips_phase5(self):
        self.assertIsNone(self._run(["ABORT"], []))


if __name__ == "__main__":
    unittest.main(verbosity=2)
