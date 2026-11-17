"""
test_vehicle.py
===============
機体プロファイル (main/mission/vehicle.py)・機体ごとのキャリブレーション
(sensor/bno_calib.py)・Phase0 での扱いのテスト。実機には触れない。

    python3 test/test_vehicle.py
"""
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import _testenv            # noqa: E402,F401
import mission_facade      # noqa: E402,F401  (sys.path)
from mission import vehicle as V          # noqa: E402
from mission import config as C           # noqa: E402
from mission import state as st           # noqa: E402
from mission import phase0_init as P0     # noqa: E402
import bno_calib                          # noqa: E402

sys.path.insert(0, str(ROOT / "tools"))
import vehicle_measure as VM              # noqa: E402

SAMPLE = bytes([0xF6, 0xFF, 0x12, 0x00, 0xE8, 0xFF, 0x40, 0x01, 0x9C, 0xFF, 0x20, 0x00,
                0xFE, 0xFF, 0x01, 0x00, 0x00, 0x00, 0xE8, 0x03, 0x8F, 0x02])


def config_for(vehicle, extra_env=None):
    """別プロセスで、その機体として config を読み込んだ結果を返す。"""
    code = ("import json, sys; sys.path.insert(0, %r)\n"
            "from mission import config as C\n"
            "print(json.dumps({'name': C.VEHICLE_NAME, 'src': C.VEHICLE_SOURCE, 'role': C.VEHICLE_ROLE,"
            " 'n': len(C.VEHICLE_OVERRIDES), 'calib': str(C.BNO_CALIB_FILE),"
            " 'weak': C.WEAK_RATIO, 'rot': C.CAM_ROTATE_180, 'fall': C.FALL_TIMEOUT_SEC}))") % str(ROOT / "main")
    env = dict(os.environ)
    env.pop("CANSAT_VEHICLE", None)
    if vehicle is not None:
        env["CANSAT_VEHICLE"] = vehicle
    env.update(extra_env or {})
    p = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, env=env)
    return p.returncode, (json.loads(p.stdout.strip().splitlines()[-1]) if p.returncode == 0 else p.stderr)


class TestActiveVehicle(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=HERE)
        self.dir = Path(self.tmp.name)
        self.active = self.dir / "active_vehicle"

    def tearDown(self):
        self.tmp.cleanup()

    def test_default_is_competition_vehicle(self):
        self.assertEqual(V.active_vehicle_name(env={}, active_file=self.active), ("honban", "default"))

    def test_file_then_env_priority(self):
        self.active.write_text("old\n", encoding="utf-8")
        self.assertEqual(V.active_vehicle_name(env={}, active_file=self.active), ("old", "file"))
        self.assertEqual(V.active_vehicle_name(env={"CANSAT_VEHICLE": "mk2"}, active_file=self.active),
                         ("mk2", "env"))

    def test_empty_file_falls_back_to_default(self):
        self.active.write_text("\n  \n", encoding="utf-8")
        self.assertEqual(V.active_vehicle_name(env={}, active_file=self.active)[0], "honban")

    def test_bad_name_rejected(self):
        for bad in ("../etc", "a b", "名前", "x" * 40):
            with self.subTest(bad=bad), self.assertRaises(V.VehicleError):
                V.active_vehicle_name(env={"CANSAT_VEHICLE": bad}, active_file=self.active)


class TestLoadProfile(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=HERE)
        self.dir = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def write(self, name, text):
        d = self.dir / name
        d.mkdir()
        (d / "vehicle.toml").write_text(text, encoding="utf-8")

    def test_load(self):
        self.write("old", '[vehicle]\ndescription = "旧"\nrole = "test"\n[overrides]\nPIN_AIN1 = 5\n')
        p = V.load_profile("old", self.dir)
        self.assertEqual((p["description"], p["role"], p["overrides"]), ("旧", "test", {"PIN_AIN1": 5}))
        self.assertEqual(V.list_vehicles(self.dir), ["old"])

    def test_missing_competition_profile_is_tolerated(self):
        p = V.load_profile("honban", self.dir)
        self.assertEqual((p["role"], p["overrides"], p["exists"]), ("competition", {}, False))

    def test_missing_other_profile_is_error(self):
        with self.assertRaises(V.VehicleError):
            V.load_profile("old", self.dir)

    def test_broken_toml_bad_role_unknown_section(self):
        self.write("a", "[overrides\nX = 1")
        self.write("b", '[vehicle]\nrole = "race"\n')
        self.write("c", "[override]\nPIN_AIN1 = 5\n")          # s が抜けている
        for name in ("a", "b", "c"):
            with self.subTest(name=name), self.assertRaises(V.VehicleError):
                V.load_profile(name, self.dir)

    def test_default_role(self):
        self.write("x", "[overrides]\n")
        self.assertEqual(V.load_profile("x", self.dir)["role"], "test")


class TestApplyOverrides(unittest.TestCase):
    def ns(self):
        return {"PIN_AIN1": 6, "CAM_ROTATE_180": True, "FIG8_SPIN_SEC": 8.0, "CAM_BACKEND": "auto",
                "CAM_SIZE": (320, 240), "CAM_HUE_RANGES": ((0, 12), (165, 179)),
                "MAG_DECLINATION_OVERRIDE": None, "WEAK_RATIO": 0.5, "_ROOT_DIR": "x", "VEHICLE_NAME": "h"}

    def test_types(self):
        ns = self.ns()
        ch = V.apply_overrides(ns, {"PIN_AIN1": 5, "CAM_ROTATE_180": False, "FIG8_SPIN_SEC": 12,
                                    "CAM_BACKEND": "numpy", "CAM_SIZE": [640, 480],
                                    "CAM_HUE_RANGES": [[0, 20]], "MAG_DECLINATION_OVERRIDE": -7.5})
        self.assertEqual(ns["PIN_AIN1"], 5)
        self.assertIs(ns["CAM_ROTATE_180"], False)
        self.assertEqual((ns["FIG8_SPIN_SEC"], type(ns["FIG8_SPIN_SEC"])), (12.0, float))
        self.assertEqual(ns["CAM_SIZE"], (640, 480))
        self.assertEqual(ns["CAM_HUE_RANGES"], ((0, 20),))
        self.assertEqual(ns["MAG_DECLINATION_OVERRIDE"], -7.5)
        self.assertEqual(len(ch), 7)
        self.assertIn(("PIN_AIN1", 6, 5), ch)

    def test_same_value_is_not_reported_as_change(self):
        self.assertEqual(V.apply_overrides(self.ns(), {"PIN_AIN1": 6}), [])

    def test_rejects(self):
        cases = {
            "typo": {"PIN_AIN": 5}, "pin as float": {"PIN_AIN1": 5.0}, "pin as string": {"PIN_AIN1": "5"},
            "bool as int": {"CAM_ROTATE_180": 0}, "int as bool": {"PIN_AIN1": True},
            "float as string": {"FIG8_SPIN_SEC": "8"}, "tuple length": {"CAM_SIZE": [320]},
            "tuple as scalar": {"CAM_SIZE": 320}, "protected": {"WEAK_RATIO": 0.3},
            "private": {"_ROOT_DIR": "y"}, "vehicle meta": {"VEHICLE_NAME": "z"}, "lowercase": {"pin": 1},
        }
        for label, ov in cases.items():
            with self.subTest(label=label):
                ns = self.ns()
                with self.assertRaises(V.VehicleError):
                    V.apply_overrides(ns, ov, protected=("WEAK_RATIO",))

    def test_typo_message_suggests_near_names(self):
        try:
            V.apply_overrides(self.ns(), {"PIN_AIN": 5})
        except V.VehicleError as e:
            self.assertIn("PIN_AIN1", str(e))


class TestConfigWithVehicles(unittest.TestCase):
    """実際の config.py を、機体を変えて別プロセスで読み込む。"""

    def test_shipped_profiles_load_and_calibration_is_per_vehicle(self):
        rc, h = config_for("honban")
        self.assertEqual(rc, 0, h)
        rc, o = config_for("old")
        self.assertEqual(rc, 0, o)
        self.assertEqual((h["name"], h["role"], h["n"]), ("honban", "competition", 0),
                         "本番機は config.py の値そのまま (差分なし)")
        self.assertEqual((o["name"], o["role"]), ("old", "test"))
        self.assertTrue(h["calib"].endswith("config/vehicles/honban/bno055_calib.json"))
        self.assertTrue(o["calib"].endswith("config/vehicles/old/bno055_calib.json"))
        self.assertNotEqual(h["calib"], o["calib"])

    def test_unset_means_competition_vehicle(self):
        if V.ACTIVE_FILE.exists():
            self.skipTest("この Pi は config/active_vehicle で機体が指定されている")
        rc, c = config_for(None)
        self.assertEqual((rc, c["name"], c["src"]), (0, "honban", "default"))

    def test_unknown_vehicle_fails_fast(self):
        rc, err = config_for("no_such_vehicle")
        self.assertNotEqual(rc, 0)
        self.assertIn("プロファイルがありません", err)

    def test_overrides_and_derived_values(self):
        d = V.VEHICLES_DIR / "zz_unittest"
        d.mkdir(parents=True, exist_ok=True)
        try:
            (d / "vehicle.toml").write_text(
                '[vehicle]\ndescription = "t"\nrole = "test"\n[overrides]\n'
                'CAM_ROTATE_180 = false\nFALL_TIMEOUT_SEC = 20\nSPEED_WEAK = 0.2\n', encoding="utf-8")
            rc, c = config_for("zz_unittest")
            self.assertEqual(rc, 0, c)
            self.assertEqual((c["rot"], c["fall"], c["n"]), (False, 20, 3))
            self.assertAlmostEqual(c["weak"], 0.25, msg="SPEED_WEAK を変えたら WEAK_RATIO も計算し直す")
            (d / "vehicle.toml").write_text('[overrides]\nPIN_AIN = 5\n', encoding="utf-8")
            rc, err = config_for("zz_unittest")
            self.assertNotEqual(rc, 0)
            self.assertIn("config.py にありません", err)
        finally:
            for f in d.iterdir():
                f.unlink()
            d.rmdir()

    def test_honban_values_equal_plain_config(self):
        self.assertEqual((C.VEHICLE_NAME, C.VEHICLE_OVERRIDES), ("honban", []))
        self.assertEqual((C.PIN_AIN1, C.CAM_ROTATE_180, C.FALL_TIMEOUT_SEC, C.WEAK_RATIO, C.BMP180_REQUIRED),
                         (6, True, 420, 0.5, True))


class RegBNO:
    """レジスタ単位の偽 BNO055 (ページ切り替えつき)。"""
    def __init__(self, uid=bytes(range(1, 17)), fail_uid=False):
        self.page = 0
        self.regs = {0: {}, 1: {0x50 + i: b for i, b in enumerate(uid)}}
        self.fail_uid = fail_uid
        self.page_log = []

    def _write_byte(self, reg, val):
        if reg == 0x07:
            self.page = val
            self.page_log.append(val)
        else:
            self.regs[self.page][reg] = val & 0xFF

    def _read_byte(self, reg):
        if self.page == 1 and self.fail_uid:
            raise OSError("I2C")
        return self.regs[self.page].get(reg, 0)

    def _set_mode(self, m):
        self._write_byte(0x3D, m)

    def setUp(self, operation_mode=None):
        return True


class TestCalibrationPerVehicle(unittest.TestCase):
    def test_unique_id_read_restores_page0(self):
        bno = RegBNO()
        self.assertEqual(bno_calib.read_unique_id(bno), bytes(range(1, 17)).hex())
        self.assertEqual(bno.page_log, [1, 0])
        bad = RegBNO(fail_uid=True)
        self.assertIsNone(bno_calib.read_unique_id(bad))
        self.assertEqual(bad.page, 0, "読めなくても必ずページ 0 に戻す")
        self.assertIsNone(bno_calib.read_unique_id(RegBNO(uid=bytes(16))))
        self.assertIsNone(bno_calib.read_unique_id(RegBNO(uid=b"\xff" * 16)))

    def test_match_rules(self):
        chk = bno_calib.check_profile_matches
        self.assertTrue(chk({"vehicle": "old", "sensor_uid": "aa"}, "old", "aa")[0])
        self.assertFalse(chk({"vehicle": "honban", "sensor_uid": "aa"}, "old", "aa")[0])
        self.assertFalse(chk({"vehicle": "old", "sensor_uid": "aa"}, "old", "bb")[0])
        self.assertTrue(chk({"vehicle": "old", "sensor_uid": "aa"}, "old", None)[0], "ID が読めないときは使う")
        self.assertTrue(chk({}, "old", "aa")[0], "古い形式 (機体名なし) は使う")
        self.assertTrue(chk(None, "old", "aa")[0])

    def test_save_and_load_metadata(self):
        with tempfile.TemporaryDirectory(dir=HERE) as d:
            p = Path(d) / "old" / "bno055_calib.json"
            bno_calib.save_profile(p, SAMPLE, (3, 3, 3, 3), vehicle="old", sensor_uid="abcd")
            data, info, meta = bno_calib.load_profile_full(p)
            self.assertEqual(data, SAMPLE)
            self.assertEqual((meta["vehicle"], meta["sensor_uid"]), ("old", "abcd"))
            self.assertIn("old", info)
            self.assertEqual(bno_calib.load_profile(p)[0], SAMPLE)         # 従来の関数も使える
            self.assertEqual(bno_calib.load_profile_full(Path(d) / "none.json"), (None, "ファイルなし", {}))


class TestPhase0WithVehicle(unittest.TestCase):
    """Phase0 が、別の機体・別のセンサのキャリブレーションを書き込まないこと、BMP180 の扱い。"""
    PATCH = ("BNO055", "BMP180", "LED", "SonarSensor", "MotorController", "MicropyGPS",
             "gps_thread_func", "set_bno_orientation", "LGPIOFactory", "Device", "log")
    CFG = ("BNO_CALIB_FILE", "LOG_DIR", "BMP180_REQUIRED", "VEHICLE_NAME", "VEHICLE_ROLE",
           "VEHICLE_OVERRIDES", "VEHICLE_SOURCE", "VEHICLE_DESCRIPTION")

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=HERE)
        self.dir = Path(self.tmp.name)
        self.saved = {k: getattr(P0, k) for k in self.PATCH}
        self.saved_cfg = {k: getattr(C, k) for k in self.CFG}
        self.logs = []
        self.bno = RegBNO()
        self.bmp_ok = True
        test = self

        class Dummy:
            def __init__(self, *a, **k): pass
            def __getattr__(self, n): return lambda *a, **k: None

        class FakeBMP(Dummy):
            def setUp(self): return test.bmp_ok
            def getTemperature(self): return 20.0
            def getPressure(self): return 101300.0

        P0.BNO055 = type("B", (), {"OPERATION_MODE_NDOF": 0x0C, "__new__": lambda cls: test.bno})
        P0.BMP180 = FakeBMP
        P0.LED = P0.SonarSensor = P0.MotorController = P0.MicropyGPS = Dummy
        P0.LGPIOFactory = Dummy
        P0.Device = type("FakeDevice", (), {})      # 本物の gpiozero.Device.pin_factory を書き換えない
        P0.gps_thread_func = lambda *a, **k: None
        P0.set_bno_orientation = lambda *a, **k: None
        P0.log = lambda m, level="INFO": self.logs.append((level, m))
        C.LOG_DIR = self.dir / "logs"
        C.BNO_CALIB_FILE = self.dir / "bno055_calib.json"
        st.g_bno_profile_loaded = False

    def tearDown(self):
        for k, v in self.saved.items():
            setattr(P0, k, v)
        for k, v in self.saved_cfg.items():
            setattr(C, k, v)
        self.tmp.cleanup()

    def text(self, level=None):
        return "\n".join(m for lv, m in self.logs if level is None or lv == level)

    def offsets_written(self):
        return bytes(self.bno.regs[0].get(0x55 + i, 0) for i in range(22))

    def test_own_calibration_is_loaded(self):
        bno_calib.save_profile(C.BNO_CALIB_FILE, SAMPLE, (3, 3, 3, 3), vehicle=C.VEHICLE_NAME,
                               sensor_uid=bytes(range(1, 17)).hex())
        P0.phase0_init()
        self.assertTrue(st.g_bno_profile_loaded)
        self.assertEqual(self.offsets_written(), SAMPLE)
        self.assertEqual(st.g_bno_uid, bytes(range(1, 17)).hex())

    def test_other_vehicles_calibration_is_not_loaded(self):
        bno_calib.save_profile(C.BNO_CALIB_FILE, SAMPLE, (3, 3, 3, 3), vehicle="other_machine")
        P0.phase0_init()
        self.assertFalse(st.g_bno_profile_loaded)
        self.assertEqual(self.offsets_written(), bytes(22), "センサには何も書き込まない")
        self.assertIn("別の機体", self.text("WARN"))

    def test_other_sensors_calibration_is_not_loaded(self):
        bno_calib.save_profile(C.BNO_CALIB_FILE, SAMPLE, (3, 3, 3, 3), vehicle=C.VEHICLE_NAME,
                               sensor_uid="ff" * 16)
        P0.phase0_init()
        self.assertFalse(st.g_bno_profile_loaded)
        self.assertIn("別の BNO055", self.text("WARN"))

    def test_banner_and_log_name_show_vehicle(self):
        C.VEHICLE_NAME, C.VEHICLE_ROLE, C.VEHICLE_DESCRIPTION = "old", "test", "過去機体"
        C.VEHICLE_OVERRIDES = [("CAM_ROTATE_180", True, False)]
        *_, log_path = P0.phase0_init()
        self.assertTrue(log_path.name.startswith("mission_old_"))
        self.assertIn("機体: old", self.text())
        self.assertIn("CAM_ROTATE_180 = False", self.text())
        self.assertIn("試験用の機体", self.text("WARN"))

    def test_competition_vehicle_has_no_test_warning(self):
        P0.phase0_init()
        self.assertNotIn("試験用の機体", self.text())

    def test_bmp180_required_by_default(self):
        self.bmp_ok = False
        with self.assertRaises(SystemExit):
            P0.phase0_init()

    def test_bmp180_optional_for_vehicle_without_it(self):
        self.bmp_ok = False
        C.BMP180_REQUIRED = False
        bno, bmp, base, *_ = P0.phase0_init()
        self.assertIsNone(bmp)
        self.assertIsNone(base)
        self.assertIn("気圧なしで続行", self.text("WARN"))


class TestMeasureSuggestions(unittest.TestCase):
    def test_typical_vehicle(self):
        out, notes = VM.suggest(70.0, speed_mps=0.5, pulse_rate_dps=60.0)
        self.assertAlmostEqual(out["CAM_SEARCH_TURN_SEC"], 0.5)
        self.assertEqual(out["CAM_SEARCH_PULSES_PER_LAP"], 14)
        self.assertAlmostEqual(out["FIG8_SPIN_SEC"], 6.7)
        self.assertEqual(out["HEADING_COG_MIN_SPEED_MPS"], 0.3)
        self.assertNotIn("STUCK_NOPROGRESS_SEC", out)
        self.assertEqual(notes, [])

    def test_slow_vehicle_gets_longer_timeouts(self):
        out, notes = VM.suggest(10.0, speed_mps=0.1)
        self.assertGreater(out["FIG8_SPIN_SEC"], 40)
        self.assertGreater(out["STUCK_RECOVER_TURN_TIMEOUT_SEC"], C.STUCK_RECOVER_TURN_TIMEOUT_SEC)
        self.assertGreater(out["STUCK_NOPROGRESS_SEC"], C.STUCK_NOPROGRESS_SEC)
        self.assertTrue(any("0.15 m/s 未満" in n for n in notes))

    def test_not_turning(self):
        out, notes = VM.suggest(1.0)
        self.assertEqual(out, {})
        self.assertTrue(notes)

    def test_output_is_valid_overrides(self):
        """目安の値は、そのまま vehicle.toml に書いて読み込める名前・型になっている。"""
        import tomllib
        out, _ = VM.suggest(25.0, speed_mps=0.2)
        ns = {k: getattr(C, k) for k in out}
        V.apply_overrides(ns, tomllib.loads(VM.format_toml(out)))


if __name__ == "__main__":
    unittest.main(verbosity=2)
