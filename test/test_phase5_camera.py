"""
test_phase5_camera.py
=====================
main/main2.py の Phase5 (phase5_camera_approach) を、モータ・超音波・カメラを
すべて偽物 (モック) に差し替えた簡易シミュレーションで通しテストする。
実機のモータ・GPIO・カメラには一切触れない。

    python3 test/test_phase5_camera.py -v

仮想世界:
  * ゴール (赤いコーン) の機体から見た方位 angle [deg] (+ = 右) と距離 dist [m]
  * 偽モータ: 旋回すると angle が変わり、前進/後退すると dist が変わる
  * 偽カメラ: angle / dist から画面上の赤い矩形を描いた画像を返す
  * 偽超音波: 正面 ±15° にゴールがあれば dist を返す (不感帯より近いと None)
  * 時間は偽の時計で進める (time.sleep は一瞬で終わる)
"""
import math
import sys
import tempfile
import types
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sensor"))
sys.path.insert(0, str(ROOT / "main"))
import main2 as M          # noqa: E402  (import してもハードウェアは初期化されない)
import goal_vision as gv   # noqa: E402

W, H = 320, 240
HFOV = 62.2                       # IMX219 の水平画角 [deg]
FOCAL = (W / 2) / math.tan(math.radians(HFOV / 2))


class Clock:
    def __init__(self):
        self.t = 1_000_000.0
        self.listeners = []

    def time(self):
        return self.t

    def sleep(self, dt):
        self.t += max(0.0, float(dt))

    def monotonic(self):
        return self.t


class World:
    def __init__(self, clock, angle=0.0, dist=3.0, *, turn_rate=120.0, speed=0.30,
                 cone_w=0.25, cone_h=0.70, blind_m=0.02, beam_deg=15.0,
                 sonar_override=None, visible=True):
        self.clock = clock
        self.angle, self.dist = angle, dist
        self.turn_rate, self.speed = turn_rate, speed
        self.cone_w, self.cone_h = cone_w, cone_h
        self.blind_m, self.beam_deg = blind_m, beam_deg
        self.sonar_override = sonar_override
        self.visible = visible
        self.min_dist = dist
        self.heading = 0.0          # 機体の方位 [deg] (偽 BNO055 の地磁気に使う)

    def apply(self, cmd, dt):
        if cmd in ("RIGHT", "RIGHT_WEAK"):
            d = self.turn_rate * dt * (0.5 if cmd.endswith("WEAK") else 1.0)
            self.angle -= d
            self.heading += d
        elif cmd in ("LEFT", "LEFT_WEAK"):
            d = self.turn_rate * dt * (0.5 if cmd.endswith("WEAK") else 1.0)
            self.angle += d
            self.heading -= d
        elif cmd == "FORWARD":
            a = math.radians(self.angle)
            # 進行方向にゴールがある分だけ近づく (横方向のずれは方位に反映)
            x = self.dist * math.sin(a)
            y = self.dist * math.cos(a) - self.speed * dt
            if y <= 0 and abs(x) < self.cone_w / 2:
                self.dist = 0.0  # コーンにぶつかったらそれ以上進まない (接触, 方位はそのまま)
            else:
                self.dist = math.hypot(x, y)
                self.angle = math.degrees(math.atan2(x, y))
        elif cmd == "BACKWARD":
            self.dist += self.speed * dt
        self.angle = (self.angle + 180.0) % 360.0 - 180.0
        self.min_dist = min(self.min_dist, self.dist)

    def render(self):
        img = np.zeros((H, W, 3), dtype=np.uint8)
        img[:] = (60, 120, 45)                  # 芝
        img[:H // 3] = (150, 190, 240)          # 空
        if not self.visible or abs(self.angle) >= 85:
            return img
        d = max(self.dist, 0.02)
        cx = W / 2 + FOCAL * math.tan(math.radians(self.angle))
        w = FOCAL * self.cone_w / d
        h = FOCAL * self.cone_h / d
        bottom = H / 2 + FOCAL * 0.10 / d       # カメラ高さ 10cm
        x0, x1 = int(max(0, cx - w / 2)), int(min(W, cx + w / 2))
        y0, y1 = int(max(0, bottom - h)), int(min(H, bottom))
        if x1 > x0 and y1 > y0:
            img[y0:y1, x0:x1] = (210, 30, 30)
        return img

    def sonar(self):
        if self.sonar_override is not None:
            return self.sonar_override(self)
        if abs(self.angle) <= self.beam_deg:
            if self.dist < self.blind_m:
                return None
            return self.dist
        return 3.5


class FakeMotor:
    def __init__(self, world, clock):
        self.world, self.clock = world, clock
        self.cmd, self.t0 = None, clock.time()
        self.history = []

    def _set(self, cmd):
        self._flush()
        self.cmd, self.t0 = cmd, self.clock.time()
        if cmd:
            self.history.append(cmd)

    def _flush(self):
        if self.cmd:
            self.world.apply(self.cmd, self.clock.time() - self.t0)
        self.t0 = self.clock.time()

    def forward(self): self._set("FORWARD")
    def backward(self): self._set("BACKWARD")
    def turn_left_strong(self): self._set("LEFT")
    def turn_right_strong(self): self._set("RIGHT")
    def turn_left_weak(self): self._set("LEFT_WEAK")
    def turn_right_weak(self): self._set("RIGHT_WEAK")
    def stop(self): self._set(None)
    def apply_diff(self, diff): self.forward(); return "FORWARD"
    def close(self): self.stop()


class FakeSonar:
    def __init__(self, world):
        self.world = world
        self.calls = 0

    def get_distance_m(self):
        self.calls += 1
        return self.world.sonar()

    def close(self):
        pass


class FakeBNO:
    """偽 BNO055: 機体の方位 world.heading に対応する地磁気ベクトルを返す
    (main2.calc_azimuth() に通すと heading になる)。"""
    def __init__(self, world, broken=False):
        self.world = world
        self.broken = broken
        self.reads = 0

    def getMag(self):
        self.reads += 1
        if self.broken:
            raise OSError("I2C error")
        a = math.radians(self.world.heading + 90.0 - M.MAG_DECLINATION)
        return [30.0 * math.cos(a), 30.0 * math.sin(a), -20.0]


class FakeCamera:
    def __init__(self, world, start_ok=True, fail_after=None):
        self.world = world
        self.start_ok = start_ok
        self.fail_after = fail_after
        self.shots = 0
        self.closed = 0

    def start(self):
        return self.start_ok

    def capture_rgb(self, discard=1):
        self.shots += 1
        if self.fail_after is not None and self.shots > self.fail_after:
            return None
        return self.world.render()

    def close(self):
        self.closed += 1


class Phase5SimBase(unittest.TestCase):
    TIMEOUT = 300

    def setUp(self):
        self.clock = Clock()
        fake_time = types.SimpleNamespace(time=self.clock.time, sleep=self.clock.sleep,
                                          monotonic=self.clock.monotonic)
        self._saved = {k: getattr(M, k) for k in
                       ("time", "blink_led", "phase5_sonar_fallback", "print",
                        "FINAL_APPROACH_TIMEOUT_SEC", "CAM_SAVE_MAX", "CAM_SAVE_EVENT_MAX", "LOG_DIR")
                       if hasattr(M, k)}
        M.time = fake_time
        M.blink_led = lambda *a, **k: None
        M.FINAL_APPROACH_TIMEOUT_SEC = self.TIMEOUT
        M.CAM_SAVE_MAX = 0
        M.CAM_SAVE_EVENT_MAX = 0             # (mission 版) 重要な瞬間の別枠保存も止める
        M.print = lambda *a, **k: None       # 表形式のコンソール出力を抑制
        self.logs = []
        self._orig_log = M.log
        M.log = lambda msg, level="INFO": self.logs.append((level, msg))
        self.fallback_calls = []

        def fake_fallback(sonar, motor, led, start_time, blink=True, timeout_sec=None):
            self.fallback_calls.append(timeout_sec)
            return "FALLBACK"
        M.phase5_sonar_fallback = fake_fallback
        with M.log_lock:
            M.log_rows.clear()

    def tearDown(self):
        for k, v in self._saved.items():
            setattr(M, k, v)
        if "print" not in self._saved:
            del M.print
        M.log = self._orig_log

    def run_sim(self, world, camera=None, **kw):
        motor = FakeMotor(world, self.clock)
        sonar = FakeSonar(world)
        camera = camera or FakeCamera(world)
        t0 = self.clock.time()
        result = M.phase5_camera_approach(sonar, motor, None, t0, camera=camera, **kw)
        motor.stop()
        return result, motor, sonar, camera, self.clock.time() - t0

    def log_text(self):
        return "\n".join(m for _, m in self.logs)


class TestPhase5Sim(Phase5SimBase):
    def test_cone_ahead_right_reaches_contact(self):
        w = World(self.clock, angle=20.0, dist=3.0)
        ok, motor, sonar, cam, el = self.run_sim(w)
        self.assertIs(ok, True, self.log_text()[-2000:])
        self.assertLess(w.dist, 0.03, "押し込み後はほぼ接触しているはず")
        self.assertIn("RIGHT", motor.history)
        self.assertIn("0m 到達 (超音波で確認済み)", self.log_text())
        self.assertEqual(cam.closed, 1)
        self.assertEqual(self.fallback_calls, [])

    def test_cone_ahead_left(self):
        w = World(self.clock, angle=-25.0, dist=2.0)
        ok, motor, *_ = self.run_sim(w)
        self.assertIs(ok, True)
        self.assertIn("LEFT", motor.history)

    def test_cone_behind_search_then_reach(self):
        w = World(self.clock, angle=150.0, dist=2.5)
        ok, motor, *_ = self.run_sim(w)
        self.assertIs(ok, True, self.log_text()[-2000:])
        self.assertIn("探索旋回", " ".join(r[-1] for r in M.log_rows))

    def test_sonar_blind_zone_uses_backoff(self):
        # 超音波の不感帯が 12cm (goal 8cm より大きい) → 至近で測れない
        w = World(self.clock, angle=0.0, dist=1.5, blind_m=0.12)
        ok, motor, *_ = self.run_sim(w)
        self.assertIs(ok, True, self.log_text()[-3000:])
        self.assertIn("BACKWARD", motor.history)
        self.assertIn("後退して再測定: OK", self.log_text())
        self.assertLess(w.dist, 0.03)

    def test_broken_sonar_never_declares_goal(self):
        # 超音波が常に失敗 → カメラだけでは 0m 到達にしない
        w = World(self.clock, angle=0.0, dist=1.5, sonar_override=lambda wd: None)
        self.TIMEOUT = 60
        M.FINAL_APPROACH_TIMEOUT_SEC = 60
        ok, motor, sonar, cam, el = self.run_sim(w)
        self.assertIs(ok, False)
        self.assertNotIn("0m 到達 (超音波で確認済み)", self.log_text())
        self.assertIn("タイムアウト", self.log_text())
        self.assertGreaterEqual(el, 60)

    def test_obstacle_in_front_is_not_goal(self):
        # 正面に障害物 (超音波 5cm) があるがカメラ上のゴールは遠くて小さい
        w = World(self.clock, angle=0.0, dist=6.0, sonar_override=lambda wd: 0.05)
        M.FINAL_APPROACH_TIMEOUT_SEC = 40
        ok, *_ = self.run_sim(w)
        self.assertIs(ok, False)
        self.assertIn("OBSTACLE", self.log_text())
        self.assertIn("障害物を回避", self.log_text())

    def test_camera_contact_but_sonar_far_reapproaches(self):
        # 超音波が常に 2m (コーンの上を素通りしている想定) → カメラだけでは到達にしない
        w = World(self.clock, angle=0.0, dist=1.0, sonar_override=lambda wd: 2.0)
        M.FINAL_APPROACH_TIMEOUT_SEC = 60
        ok, motor, *_ = self.run_sim(w)
        self.assertIs(ok, False)
        self.assertIn("下がって再接近します", self.log_text())
        self.assertIn("BACKWARD", motor.history)

    def test_goal_moved_after_push_is_rejected(self):
        # 押し込み直前までは正常、押し込み後は超音波が 50cm (コーンが倒れて離れた想定)
        state = {"pushed": False}

        def sonar(wd):
            if state["pushed"]:
                return 0.50
            if abs(wd.angle) <= wd.beam_deg:
                return wd.dist if wd.dist >= 0.02 else None
            return 3.5
        w = World(self.clock, angle=0.0, dist=1.0, sonar_override=sonar)
        orig_apply = w.apply

        def apply(cmd, dt):
            orig_apply(cmd, dt)
            if cmd == "FORWARD" and abs(dt - M.GOAL_FINAL_PUSH_SEC) < 1e-6:
                state["pushed"] = True
                w.visible = False
        w.apply = apply
        M.FINAL_APPROACH_TIMEOUT_SEC = 30
        ok, *_ = self.run_sim(w)
        self.assertIsNot(ok, True)   # 成功にしない (見えなくなるので1周探して GPS へ戻る)
        self.assertIn("押し込み後の検証: NG", self.log_text())

    def test_camera_start_failure_falls_back_to_sonar(self):
        w = World(self.clock)
        cam = FakeCamera(w, start_ok=False)
        ok, *_ = self.run_sim(w, camera=cam)
        self.assertEqual(ok, "FALLBACK")
        self.assertEqual(len(self.fallback_calls), 1)
        self.assertGreater(self.fallback_calls[0], 0)

    def test_repeated_capture_failure_falls_back(self):
        w = World(self.clock, angle=0.0, dist=3.0)
        cam = FakeCamera(w, fail_after=3)
        ok, *_ = self.run_sim(w, camera=cam)
        self.assertEqual(ok, "FALLBACK")
        self.assertEqual(cam.closed, 1, "フォールバック前にカメラを解放する")

    def test_cone_never_visible_falls_back(self):
        # GPS へ戻れない (戻る回数の上限に達した) ときは従来どおり探索 → 超音波スキャン
        w = World(self.clock, visible=False, sonar_override=lambda wd: 3.0)
        ok, motor, *_ = self.run_sim(w, allow_return_to_gps=False)
        self.assertEqual(ok, "FALLBACK")
        self.assertIn("周探索しても見つからない", self.log_text())
        self.assertIn("FORWARD", motor.history)   # 1周ごとに視点を変える前進

    # ── 1周しても見つからない → 少し移動して Phase4 へ戻る ──
    def _search_turns(self, motor):
        return sum(1 for c in motor.history if c in ("LEFT", "RIGHT"))

    def test_lap_without_goal_returns_to_gps_using_heading(self):
        w = World(self.clock, visible=False, sonar_override=lambda wd: 3.0)
        bno = FakeBNO(w)
        ok, motor, *_ = self.run_sim(w, bno=bno)
        self.assertEqual(ok, M.PHASE5_RETURN_GPS)
        # 1 パルス = 120°/s × 0.25 s = 30° → 地磁気で 360° を測って 12 回で1周
        self.assertGreaterEqual(abs(w.heading), 360.0)
        self.assertLess(abs(w.heading), 360.0 + 31.0, "1周を大きく超えて回り続けない")
        self.assertEqual(self._search_turns(motor), 12)
        self.assertEqual(motor.history[-1], "FORWARD", "戻る前に少し前進する")
        self.assertGreater(bno.reads, 0)
        self.assertIn("地磁気で", self.log_text())
        self.assertIn("Phase4 (GPS 誘導) へ戻る", self.log_text())

    def test_lap_without_bno_uses_pulse_count(self):
        w = World(self.clock, visible=False, sonar_override=lambda wd: 3.0)
        ok, motor, *_ = self.run_sim(w)                     # bno なし
        self.assertEqual(ok, M.PHASE5_RETURN_GPS)
        self.assertEqual(self._search_turns(motor), M.CAM_SEARCH_PULSES_PER_LAP)

    def test_broken_bno_falls_back_to_pulse_count(self):
        w = World(self.clock, visible=False, sonar_override=lambda wd: 3.0)
        ok, motor, *_ = self.run_sim(w, bno=FakeBNO(w, broken=True))
        self.assertEqual(ok, M.PHASE5_RETURN_GPS)
        self.assertEqual(self._search_turns(motor), M.CAM_SEARCH_PULSES_PER_LAP)

    def test_blocked_front_turns_instead_of_forward_before_return(self):
        w = World(self.clock, visible=False, sonar_override=lambda wd: 0.10)
        ok, motor, *_ = self.run_sim(w, bno=FakeBNO(w))
        self.assertEqual(ok, M.PHASE5_RETURN_GPS)
        self.assertNotIn("FORWARD", motor.history, "前が塞がっていたら前進しない")

    def test_cone_found_during_lap_does_not_return(self):
        w = World(self.clock, angle=-150.0, dist=2.0)
        ok, *_ = self.run_sim(w, bno=FakeBNO(w))
        self.assertIs(ok, True)

    def test_images_saved_when_enabled(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as d:
            M.LOG_DIR = Path(d)
            M.CAM_SAVE_MAX = 50
            M.CAM_SAVE_EVENT_MAX = 50
            w = World(self.clock, angle=10.0, dist=1.0)
            ok, *_ = self.run_sim(w)
            self.assertIs(ok, True)
            files = list(Path(d).glob("cam_*/*.jpg"))
            self.assertTrue(files)
            self.assertTrue(any("after_push_ok" in f.name for f in files), sorted(f.name for f in files))


class TestPhase4And5Loop(unittest.TestCase):
    """main() の Phase4 ⇄ Phase5 のくり返し (run_phase4_and_5)。各フェーズは差し替える。"""

    def setUp(self):
        self._saved = {k: getattr(M, k) for k in ("phase4_guided_run", "phase5_camera_approach", "log")}
        M.log = lambda *a, **k: None
        self.p4_calls, self.p5_calls = [], []

    def tearDown(self):
        for k, v in self._saved.items():
            setattr(M, k, v)

    def _run(self, p4_results, p5_results):
        p4 = iter(p4_results)
        p5 = iter(p5_results)

        def fake_p4(*a, min_run_sec=0.0, **k):
            self.p4_calls.append(min_run_sec)
            return next(p4)

        def fake_p5(*a, bno=None, allow_return_to_gps=True, **k):
            self.p5_calls.append((bno, allow_return_to_gps))
            return next(p5)
        M.phase4_guided_run = fake_p4
        M.phase5_camera_approach = fake_p5
        return M.run_phase4_and_5("BNO", None, None, None, 0.0, 0.0)

    def test_return_to_gps_then_reach(self):
        r = self._run(["NEAR_GOAL"] * 3, [M.PHASE5_RETURN_GPS, M.PHASE5_RETURN_GPS, True])
        self.assertIs(r, True)
        self.assertEqual(self.p4_calls, [0.0, M.PHASE4_REENTRY_MIN_SEC, M.PHASE4_REENTRY_MIN_SEC])
        self.assertEqual([c[1] for c in self.p5_calls], [True, True, True])
        self.assertTrue(all(c[0] == "BNO" for c in self.p5_calls), "Phase5 に BNO055 を渡す")

    def test_max_returns(self):
        n = M.PHASE5_MAX_RETURNS_TO_GPS
        r = self._run(["NEAR_GOAL"] * (n + 1), [M.PHASE5_RETURN_GPS] * n + [False])
        self.assertIs(r, False)
        self.assertEqual(len(self.p4_calls), n + 1)
        self.assertEqual([c[1] for c in self.p5_calls], [True] * n + [False],
                         "上限に達したら最後の Phase5 は GPS に戻らない")

    def test_phase4_timeout_after_return(self):
        r = self._run(["NEAR_GOAL", "TIMEOUT"], [M.PHASE5_RETURN_GPS])
        self.assertIsNone(r)
        self.assertEqual(len(self.p5_calls), 1)


class TestCameraConfig(unittest.TestCase):
    def test_camera_mounted_upside_down(self):
        self.assertTrue(M.CAM_ROTATE_180)
        self.assertTrue(M.make_camera().rotate180)


if __name__ == "__main__":
    unittest.main(verbosity=2)
