"""
test_vehicle.py
===============
機体の切り替え (config.py の VEHICLE と main/mission/vehicle.py)・機体ごとのキャリブレーション
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
import motor_test as MT                   # noqa: E402

SAMPLE = bytes([0xF6, 0xFF, 0x12, 0x00, 0xE8, 0xFF, 0x40, 0x01, 0x9C, 0xFF, 0x20, 0x00,
                0xFE, 0xFF, 0x01, 0x00, 0x00, 0x00, 0xE8, 0x03, 0x8F, 0x02])


def config_for(vehicle, insert=None, outside=None):
    """
    別プロセスで、その機体として config を読み込んだ結果を返す。
    insert を渡すと、config.py のコピーの if VEHICLE == "old": の中にその行を足して読み込む
    outside を渡すと、if の外 (4 文字下げ忘れ) にその行を足す。
    (本物の config.py は書き換えない)。
    """
    pkg_root = ROOT / "main"
    tmp = None
    if insert is not None or outside is not None:
        tmp = tempfile.TemporaryDirectory(dir=HERE)
        pkg = Path(tmp.name) / "mission"
        pkg.mkdir()
        (pkg / "__init__.py").write_text("", encoding="utf-8")
        (pkg / "vehicle.py").write_text((ROOT / "main/mission/vehicle.py").read_text(encoding="utf-8"),
                                        encoding="utf-8")
        src = (ROOT / "main/mission/config.py").read_text(encoding="utf-8")
        marker = 'if VEHICLE == "old":\n'
        assert marker in src
        src = src.replace(marker, marker + "".join(f"    {l}\n" for l in insert or []))
        tail = "# ---- ここから下は触らない"
        assert tail in src
        src = src.replace(tail, "".join(f"{l}\n" for l in outside or []) + tail)
        (pkg / "config.py").write_text(src, encoding="utf-8")
        pkg_root = Path(tmp.name)
    code = ("import json, sys; sys.path.insert(0, %r)\n"
            "from mission import config as C\n"
            "print(json.dumps({'name': C.VEHICLE_NAME, 'src': C.VEHICLE_SOURCE,"
            " 'comp': C.IS_COMPETITION_VEHICLE, 'n': len(C.VEHICLE_OVERRIDES), 'calib': str(C.BNO_CALIB_FILE),"
            " 'weak': C.WEAK_RATIO, 'rot': C.CAM_ROTATE_180, 'fall': C.FALL_TIMEOUT_SEC,"
            " 'size': list(C.CAM_SIZE), 'size_t': type(C.CAM_SIZE).__name__}))") % str(pkg_root)
    env = dict(os.environ)
    env.pop("CANSAT_VEHICLE", None)
    if vehicle is not None:
        env["CANSAT_VEHICLE"] = vehicle
    try:
        p = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, env=env)
    finally:
        if tmp:
            tmp.cleanup()
    return p.returncode, (json.loads(p.stdout.strip().splitlines()[-1]) if p.returncode == 0 else p.stderr)


class TestChoose(unittest.TestCase):
    KNOWN = ("honban", "old")

    def test_config_value_is_used(self):
        self.assertEqual(V.choose("old", self.KNOWN, env={}), ("old", "config"))
        self.assertEqual(V.choose("honban", self.KNOWN, env={"CANSAT_VEHICLE": " "}), ("honban", "config"))

    def test_env_overrides_for_tests(self):
        self.assertEqual(V.choose("old", self.KNOWN, env={"CANSAT_VEHICLE": "honban"}), ("honban", "env"))

    def test_unknown_name_rejected(self):
        for bad in ("olld", "Honban", "", "../x"):
            with self.subTest(bad=bad), self.assertRaises(V.VehicleError):
                V.choose(bad, self.KNOWN, env={})
        with self.assertRaises(V.VehicleError):
            V.choose("honban", self.KNOWN, env={"CANSAT_VEHICLE": "mk9"})


class TestCheckChanges(unittest.TestCase):
    BASE = {"PIN_AIN1": 6, "CAM_ROTATE_180": True, "FIG8_SPIN_SEC": 8.0, "CAM_BACKEND": "auto",
            "CAM_SIZE": (320, 240), "CAM_HUE_RANGES": ((0, 12), (165, 179)),
            "MAG_DECLINATION_OVERRIDE": None, "WEAK_RATIO": 0.5, "_ROOT_DIR": "x"}

    def run_block(self, **changes):
        ns = dict(self.BASE)
        before = V.snapshot(ns)
        ns.update(changes)
        return ns, V.check_changes(ns, before, protected=("WEAK_RATIO",))

    def test_no_change(self):
        self.assertEqual(self.run_block()[1], [])
        self.assertEqual(self.run_block(PIN_AIN1=6)[1], [], "同じ値は違いとして数えない")

    def test_types(self):
        ns, ch = self.run_block(PIN_AIN1=5, CAM_ROTATE_180=False, FIG8_SPIN_SEC=12, CAM_BACKEND="numpy",
                                CAM_SIZE=[640, 480], CAM_HUE_RANGES=((0, 20),), MAG_DECLINATION_OVERRIDE=-7.5)
        self.assertEqual(ns["PIN_AIN1"], 5)
        self.assertEqual((ns["FIG8_SPIN_SEC"], type(ns["FIG8_SPIN_SEC"])), (12.0, float))
        self.assertEqual(ns["CAM_SIZE"], (640, 480))
        self.assertEqual(len(ch), 7)
        self.assertIn(("PIN_AIN1", 6, 5), ch)

    def test_rejects(self):
        cases = {
            "typo": {"PIN_AIN": 5}, "pin as float": {"PIN_AIN1": 5.0}, "pin as string": {"PIN_AIN1": "5"},
            "bool as int": {"CAM_ROTATE_180": 0}, "int as bool": {"PIN_AIN1": True},
            "float as string": {"FIG8_SPIN_SEC": "8"}, "tuple length": {"CAM_SIZE": (320,)},
            "tuple as scalar": {"CAM_SIZE": 320}, "protected": {"WEAK_RATIO": 0.3},
        }
        for label, ch in cases.items():
            with self.subTest(label=label), self.assertRaises(V.VehicleError):
                self.run_block(**ch)

    def test_typo_message_suggests_near_names(self):
        with self.assertRaises(V.VehicleError) as cm:
            self.run_block(PIN_AIN=5)
        self.assertIn("PIN_AIN1", str(cm.exception))

    def test_lowercase_helpers_are_ignored(self):
        self.assertEqual(self.run_block(tmp=1, _x=2)[1], [])


class TestConfigWithVehicles(unittest.TestCase):
    """実際の config.py を、機体を変えて別プロセスで読み込む。"""

    def test_honban_and_old_load_with_separate_calibration(self):
        rc, h = config_for("honban")
        self.assertEqual(rc, 0, h)
        rc, o = config_for("old")
        self.assertEqual(rc, 0, o)
        self.assertEqual((h["name"], h["comp"], h["n"]), ("honban", True, 0),
                         "本番機は config.py の値そのまま (違いなし)")
        self.assertEqual((o["name"], o["comp"]), ("old", False))
        self.assertTrue(h["calib"].endswith("config/bno055_calib_honban.json"))
        self.assertTrue(o["calib"].endswith("config/bno055_calib_old.json"))

    def test_shipped_config_selects_competition_vehicle(self):
        """リポジトリの config.py は本番機を選んでいること (old のまま push していない)。"""
        rc, c = config_for(None)
        self.assertEqual((rc, c["name"], c["src"]), (0, "honban", "config"),
                         'main/mission/config.py の VEHICLE が "honban" になっていません')

    def test_unknown_vehicle_fails_fast(self):
        rc, err = config_for("no_such_vehicle")
        self.assertNotEqual(rc, 0)
        self.assertIn("知らない機体", err)

    def test_old_block_values_and_derived_values(self):
        lines = ["CAM_ROTATE_180 = False", "FALL_TIMEOUT_SEC = 20", "SPEED_WEAK = 0.2", "CAM_SIZE = [640, 480]"]
        rc, c = config_for("old", insert=lines)
        self.assertEqual(rc, 0, c)
        self.assertEqual((c["rot"], c["fall"], c["n"], c["size"], c["size_t"]), (False, 20, 4, [640, 480], "tuple"))
        self.assertAlmostEqual(c["weak"], 0.25, msg="SPEED_WEAK を変えたら WEAK_RATIO も計算し直す")
        rc, h = config_for("honban", insert=lines)
        self.assertEqual((rc, h["rot"], h["fall"], h["n"]), (0, True, 420, 0), "本番機には混ざらない")

    def test_old_block_typo_fails_fast(self):
        rc, err = config_for("old", insert=["PIN_AIN = 5"])
        self.assertNotEqual(rc, 0)
        self.assertIn("config.py にありません", err)
        self.assertIn("PIN_AIN1", err)
        rc, err = config_for("old", insert=["CAM_ROTATE_180 = 0"])
        self.assertIn("型が違います", err)

    def test_unindented_line_is_caught_on_competition_vehicle(self):
        """if の外に書いてしまった (4 文字下げ忘れ) 値で本番機が変わったら止める。"""
        rc, err = config_for("honban", outside=["FIG8_SPIN_SEC = 12.0"])
        self.assertNotEqual(rc, 0)
        self.assertIn("4 文字下げ", err)

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
    CFG = ("BNO_CALIB_FILE", "LOG_DIR", "BMP180_REQUIRED", "VEHICLE_NAME", "IS_COMPETITION_VEHICLE",
           "VEHICLE_OVERRIDES", "VEHICLE_SOURCE")

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
        C.VEHICLE_NAME, C.IS_COMPETITION_VEHICLE = "old", False
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

    def test_output_lines_are_indented(self):
        out, _ = VM.suggest(25.0, speed_mps=0.2)
        self.assertTrue(all(l.startswith("    ") for l in VM.format_lines(out).splitlines()))


class TestMeasureWriteConfig(unittest.TestCase):
    """vehicle_measure.py が、確認のあとで config.py (のコピー) に書き込む部分。"""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=HERE)
        pkg = Path(self.tmp.name) / "mission"
        pkg.mkdir()
        (pkg / "__init__.py").write_text("", encoding="utf-8")
        for name in ("vehicle.py", "config.py"):
            (pkg / name).write_text((ROOT / "main/mission" / name).read_text(encoding="utf-8"), encoding="utf-8")
        self.path = pkg / "config.py"
        self.orig = self.path.read_text(encoding="utf-8")
        self.values, _ = VM.suggest(25.0, speed_mps=0.2)    # 遅い機体の目安 (値が今と違う)

    def tearDown(self):
        self.tmp.cleanup()

    def load(self, vehicle):
        return VM.verify_config(self.path, vehicle, {})

    def read_values(self, vehicle, keys):
        code = ("import json, sys; sys.path.insert(0, %r)\nfrom mission import config as C\n"
                "print(json.dumps({k: getattr(C, k) for k in %r} | {'_n': len(C.VEHICLE_OVERRIDES)}))"
                ) % (str(self.path.parents[1]), list(keys))
        p = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                           env=dict(os.environ, CANSAT_VEHICLE=vehicle))
        self.assertEqual(p.returncode, 0, p.stderr)
        return json.loads(p.stdout.strip().splitlines()[-1])

    def test_write_into_old_block(self):
        ok, msg = VM.write_config(self.values, "old", competition=False, path=self.path)
        self.assertTrue(ok, msg)
        got = self.read_values("old", self.values)
        self.assertEqual({k: got[k] for k in self.values}, self.values)
        self.assertEqual(got["_n"], len(self.values))
        h = self.read_values("honban", self.values)
        self.assertEqual(h["_n"], 0, "本番機の値は変わらない")
        self.assertEqual(h["FIG8_SPIN_SEC"], C.FIG8_SPIN_SEC)
        text = self.path.read_text(encoding="utf-8")
        self.assertIn("    CAM_SEARCH_TURN_SEC = ", text)
        self.assertNotIn("    # CAM_SEARCH_TURN_SEC", text, "# 付きの例の行を置き換える")

    def test_write_twice_replaces_not_duplicates(self):
        VM.write_config(self.values, "old", competition=False, path=self.path)
        v2 = dict(self.values, FIG8_SPIN_SEC=33.0)
        ok, msg = VM.write_config(v2, "old", competition=False, path=self.path)
        self.assertTrue(ok, msg)
        text = self.path.read_text(encoding="utf-8")
        self.assertEqual(text.count("    FIG8_SPIN_SEC = "), 1)
        self.assertEqual(self.read_values("old", ["FIG8_SPIN_SEC"])["FIG8_SPIN_SEC"], 33.0)

    def test_write_competition_values_keeps_comment(self):
        ok, msg = VM.write_config({"FIG8_SPIN_SEC": 6.5}, "honban", competition=True, path=self.path)
        self.assertTrue(ok, msg)
        got = self.read_values("honban", ["FIG8_SPIN_SEC"])
        self.assertEqual((got["FIG8_SPIN_SEC"], got["_n"]), (6.5, 0))
        line = next(l for l in self.path.read_text(encoding="utf-8").splitlines() if l.startswith("FIG8_SPIN_SEC"))
        self.assertIn("6.5", line)
        self.assertIn("#", line, "説明のコメントは残す")

    def test_bad_value_is_rolled_back(self):
        ok, msg = VM.write_config({"PIN_AIN1": 5.0}, "old", competition=False, path=self.path)
        self.assertFalse(ok)
        self.assertIn("元に戻しました", msg)
        self.assertEqual(self.path.read_text(encoding="utf-8"), self.orig)

    def test_unknown_place_is_not_written(self):
        ok, msg = VM.write_config({"NO_SUCH_SETTING": 1}, "honban", competition=True, path=self.path)
        self.assertFalse(ok)
        self.assertEqual(self.path.read_text(encoding="utf-8"), self.orig)

    def test_ask_no_does_not_write(self):
        for answer in ("no", "", "y", "はい"):
            with self.subTest(answer=answer):
                self.assertFalse(VM.ask_and_write(self.values, "old", False, ask=lambda _: answer, path=self.path))
                self.assertEqual(self.path.read_text(encoding="utf-8"), self.orig)

    def test_ask_yes_writes(self):
        self.assertTrue(VM.ask_and_write(self.values, "old", False, ask=lambda _: "yes", path=self.path))
        self.assertNotEqual(self.path.read_text(encoding="utf-8"), self.orig)

class TestMotorTest(unittest.TestCase):
    """tools/motor_test.py が本番と同じピン・同じ部品でモータを動かすこと (実機には触れない)。"""

    def test_no_hardcoded_pins(self):
        src = (ROOT / "tools" / "motor_test.py").read_text(encoding="utf-8")
        self.assertNotIn("Motor(forward=", src, "ピン番号を直接書かず config.py を使う")
        self.assertIn("MotorController", src)

    def test_keys_call_real_motor_methods(self):
        from mission.hardware import MotorController
        for key, (method, _, _) in MT.KEYMAP.items():
            with self.subTest(key=key):
                self.assertTrue(callable(getattr(MotorController, method, None)), method)

    def test_run_loop(self):
        calls = []

        class FakeMotor:
            def __getattr__(self, name):
                return lambda: calls.append(name)

        keys = iter(["w", "a", "x", "d", " ", "s", "q", "w"])
        out = []
        MT.run(FakeMotor(), lambda: next(keys), out=out.append)
        self.assertEqual(calls, ["forward", "turn_left_strong", "turn_right_strong", "stop", "backward", "stop"],
                         "知らないキーは無視し、q で止まり、最後に必ず停止する")
        self.assertIn("右のタイヤだけ", out[1])

    def test_stops_even_on_error(self):
        calls = []

        class FakeMotor:
            def forward(self): calls.append("forward")
            def stop(self): calls.append("stop")

        def keys():
            if calls:
                raise KeyboardInterrupt
            return "w"
        with self.assertRaises(KeyboardInterrupt):
            MT.run(FakeMotor(), keys, out=lambda s: None)
        self.assertEqual(calls, ["forward", "stop"])

    def test_pin_summary_uses_config(self):
        text = MT.pin_summary()
        self.assertIn(f"AIN1={C.PIN_AIN1}", text)
        self.assertIn(f"BIN2={C.PIN_BIN2}", text)


if __name__ == "__main__":
    unittest.main(verbosity=2)
