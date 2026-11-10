"""
test_mission_flow.py
====================
main/main.py のフェーズ切り替え (状態遷移) を、ハードウェア無しで確認する。
各フェーズ関数をモックに差し替え、呼ばれる順番・引数の受け渡し・
終了処理 (motor.close / sonar.close / led.off / save_log) を検証する。

    python3 test/test_mission_flow.py
"""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mission_facade                     # noqa: E402
from mission import state as st           # noqa: E402
from mission import config as C           # noqa: E402

M = mission_facade.load_main_module()


class Dev:
    def __init__(self, log):
        self.log = log

    def _rec(self, name):
        def f(*a, **k):
            self.log.append(name)
        return f

    def __getattr__(self, name):
        return self._rec(f"{type(self).__name__}.{name}")


class Motor(Dev):
    pass


class Sonar(Dev):
    pass


class Led(Dev):
    pass


class FlowBase(unittest.TestCase):
    PATCH = ("phase0_init", "phase1_fall_detection", "phase2_initial_backward",
             "phase3_calibration", "phase4_guided_run", "phase5_camera_approach",
             "save_log", "log")

    def setUp(self):
        self.calls = []
        self.events = []
        self.saved = {k: getattr(M, k) for k in self.PATCH}
        M.log = lambda msg, level="INFO": self.events.append((level, msg))
        self.motor, self.sonar, self.led = Motor(self.events), Sonar(self.events), Led(self.events)

        def p0():
            self.calls.append(("phase0", st.phase))
            return "BNO", "BMP", 101325.0, self.sonar, self.motor, self.led, Path("/tmp/x.csv")

        def p1(bno, led, start_time):
            st.phase = 1
            self.calls.append(("phase1", bno, led is self.led))
            return self.fall_ok

        def p2(motor, sonar, led, start_time):
            st.phase = 2
            self.calls.append(("phase2", motor is self.motor, sonar is self.sonar))

        def p3(bno, motor, led, start_time):
            st.phase = 3
            self.calls.append(("phase3", bno))
            return 0.123

        def p4(bno, sonar, motor, led, start_time, baseline, min_run_sec=0.0):
            st.phase = 4
            self.calls.append(("phase4", baseline, min_run_sec))
            return self.p4_results.pop(0)

        def p5(sonar, motor, led, start_time, bno=None, camera=None, allow_return_to_gps=True):
            st.phase = 5
            self.calls.append(("phase5", bno, camera, allow_return_to_gps))
            return self.p5_results.pop(0)

        M.phase0_init, M.phase1_fall_detection, M.phase2_initial_backward = p0, p1, p2
        M.phase3_calibration, M.phase4_guided_run, M.phase5_camera_approach = p3, p4, p5
        M.save_log = lambda path: self.events.append(("save_log", str(path)))
        self.fall_ok = True
        self.p4_results, self.p5_results = ["NEAR_GOAL"], [True]
        st.phase = 0
        st.g_p4_camera = None
        st.g_gps_valid = False

    def tearDown(self):
        for k, v in self.saved.items():
            setattr(M, k, v)

    def phases(self):
        return [c[0] for c in self.calls]


class TestMainFlow(FlowBase):
    def test_normal_sequence(self):
        M.main()
        self.assertEqual(self.phases(), ["phase0", "phase1", "phase2", "phase3", "phase4", "phase5"])
        self.assertEqual(self.calls[1][1], "BNO")
        self.assertEqual(self.calls[4], ("phase4", 0.123, 0.0), "Phase3 の基準加速度が Phase4 に渡る")
        self.assertEqual(self.calls[5], ("phase5", "BNO", None, True))
        # 終了処理
        tail = [e for e in self.events if isinstance(e, str) or e[0] == "save_log"]
        self.assertEqual(tail, ["Motor.close", "Sonar.close", "Led.off", "Led.close", ("save_log", "/tmp/x.csv")])
        self.assertTrue(any("ミッションシーケンス 完了" in m for lv, m in self.events if lv == "INFO"))

    def test_fall_timeout_still_continues(self):
        self.fall_ok = False
        M.main()
        self.assertEqual(self.phases(), ["phase0", "phase1", "phase2", "phase3", "phase4", "phase5"])
        self.assertTrue(any("タイムアウトのため強制移行" in m for _, m in self.events if isinstance(m, str)))

    def test_phase5_returns_to_gps_then_reaches(self):
        self.p4_results = ["NEAR_GOAL", "NEAR_GOAL", "CONE_FOUND"]
        self.p5_results = [C.PHASE5_RETURN_GPS, C.PHASE5_RETURN_GPS, True]
        M.main()
        self.assertEqual(self.phases(), ["phase0", "phase1", "phase2", "phase3",
                                         "phase4", "phase5", "phase4", "phase5", "phase4", "phase5"])
        p4 = [c for c in self.calls if c[0] == "phase4"]
        self.assertEqual([c[2] for c in p4], [0.0, C.PHASE4_REENTRY_MIN_SEC, C.PHASE4_REENTRY_MIN_SEC])

    def test_return_limit(self):
        n = C.PHASE5_MAX_RETURNS_TO_GPS
        self.p4_results = ["NEAR_GOAL"] * (n + 1)
        self.p5_results = [C.PHASE5_RETURN_GPS] * n + [False]
        M.main()
        p5 = [c for c in self.calls if c[0] == "phase5"]
        self.assertEqual([c[3] for c in p5], [True] * n + [False], "上限に達したら GPS に戻らない")

    def test_phase4_timeout_near_goal(self):
        st.g_gps_valid = True
        st.g_gps_lat, st.g_gps_lng = C.TARGET_LAT + 5.0 / 111320.0, C.TARGET_LNG
        self.p4_results, self.p5_results = ["TIMEOUT"], [False]
        M.main()
        self.assertEqual(self.phases()[-2:], ["phase4", "phase5"])
        self.assertEqual(self.calls[-1][3], False)

    def test_phase4_timeout_far_skips_phase5(self):
        st.g_gps_valid = True
        st.g_gps_lat, st.g_gps_lng = C.TARGET_LAT + 100.0 / 111320.0, C.TARGET_LNG
        self.p4_results, self.p5_results = ["TIMEOUT"], []
        M.main()
        self.assertEqual(self.phases()[-1], "phase4")
        self.assertIn("Motor.close", self.events)

    def test_phase4_abort_skips_phase5(self):
        self.p4_results, self.p5_results = ["ABORT"], []
        M.main()
        self.assertNotIn("phase5", self.phases())

    def test_camera_from_phase4_is_passed_to_phase5(self):
        def p4(*a, **k):
            st.g_p4_camera = "CAM"
            return "CONE_FOUND"
        M.phase4_guided_run = p4
        M.main()
        self.assertEqual(self.calls[-1], ("phase5", "BNO", "CAM", True))

    def test_exception_in_phase_stops_motor_and_saves_log(self):
        def boom(*a, **k):
            raise RuntimeError("sensor died")
        M.phase3_calibration = boom
        with self.assertRaises(RuntimeError):
            M.main()
        self.assertIn("Motor.stop", self.events)
        self.assertIn("Motor.close", self.events)
        self.assertIn(("save_log", "/tmp/x.csv"), self.events)
        self.assertNotIn("phase4", self.phases())

    def test_keyboard_interrupt_is_handled(self):
        def ctrl_c(*a, **k):
            raise KeyboardInterrupt
        M.phase4_guided_run = ctrl_c
        M.main()                                   # 例外にならずに終了処理まで進む
        self.assertIn("Motor.stop", self.events)
        self.assertIn(("save_log", "/tmp/x.csv"), self.events)


class TestDeclinationPresets(unittest.TestCase):
    """地磁気偏角のプリセット (仙台/能代/種子島) と auto 選択。"""

    def test_presets_resolve(self):
        import datetime
        d = datetime.date(2026, 11, 10)
        r = C._resolve_mag_declination
        self.assertEqual(r("sendai", 0, 0, None, d), ("sendai", -9.01))
        self.assertEqual(r("noshiro", 0, 0, None, d), ("noshiro", -9.21))
        self.assertEqual(r("tanegashima", 0, 0, None, d), ("tanegashima", -7.35))

    def test_auto_picks_nearest_site(self):
        import datetime
        d = datetime.date(2026, 11, 10)
        r = C._resolve_mag_declination
        self.assertEqual(r("auto", 38.26, 140.85, None, d)[0], "sendai")
        self.assertEqual(r("auto", 40.15, 140.00, None, d)[0], "noshiro")
        self.assertEqual(r("auto", 30.37, 130.96, None, d)[0], "tanegashima")

    def test_override_and_errors(self):
        r = C._resolve_mag_declination
        self.assertEqual(r("auto", 0, 0, -7.5), ("override", -7.5))
        with self.assertRaises(ValueError):
            r("tokyo", 0, 0, None)

    def test_resolved_value_is_used_by_navigation(self):
        from mission import navigation as nav
        self.assertIsInstance(C.MAG_DECLINATION, float)
        self.assertIn(C.MAG_SITE_RESOLVED, list(C.MAG_SITE_PRESETS) + ["override"])
        # 磁北を向いた地磁気ベクトル (この計算式では y 成分が前方) → 方位 = 偏角分だけずれる
        az = nav.calc_azimuth([0.0, 30.0, -20.0])
        self.assertAlmostEqual(az, (0.0 + C.MAG_DECLINATION) % 360.0, places=6)


class TestPackageLayout(unittest.TestCase):
    def test_no_circular_import_and_names(self):
        import importlib
        for name in ("config", "state", "logger", "hardware", "navigation", "camera",
                     "phase0_init", "phase1_fall", "phase2_separation",
                     "phase3_calibration", "phase4_gps", "phase5_camera"):
            importlib.import_module(f"mission.{name}")
        self.assertTrue(callable(M.main) and callable(M.run_phase4_and_5))

    def test_config_has_no_hardware_or_state(self):
        import mission.config as cfg
        names = [n for n in dir(cfg) if not n.startswith("_")]
        self.assertTrue(all(n.isupper() or n == "Path" for n in names), names)


if __name__ == "__main__":
    unittest.main(verbosity=2)
