"""
test_goal_vision.py
===================
sensor/goal_vision.py のユニットテスト (カメラ・GPIO 不要)。

    python3 test/test_goal_vision.py
"""
import colorsys
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sensor"))
import goal_vision as gv  # noqa: E402

W, H = 320, 240
RED = (210, 25, 25)
GRASS = (60, 120, 45)


def solid(rgb, w=W, h=H):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = rgb
    return img


def with_rect(bg, color, x, y, w, h):
    img = solid(bg)
    img[y:y + h, x:x + w] = color
    return img


class TestHSV(unittest.TestCase):
    def test_matches_colorsys(self):
        rng = np.random.default_rng(0)
        px = rng.integers(0, 256, size=(1, 600, 3), dtype=np.uint8)
        h, s, v = gv.rgb_to_hsv_u8(px)
        for i in range(px.shape[1]):
            r, g, b = (int(c) for c in px[0, i])
            eh, es, ev = colorsys.rgb_to_hsv(r / 255, g / 255, b / 255)
            eh, es, ev = eh * 180, es * 255, ev * 255
            dh = abs(int(h[0, i]) - eh) % 180
            self.assertLessEqual(min(dh, 180 - dh), 1.0, (r, g, b))
            self.assertLessEqual(abs(int(s[0, i]) - es), 1.0, (r, g, b))
            self.assertLessEqual(abs(int(v[0, i]) - ev), 1.0, (r, g, b))

    def test_known_values(self):
        px = np.array([[[255, 0, 0], [0, 255, 0], [0, 0, 255], [128, 128, 128]]], dtype=np.uint8)
        h, s, v = gv.rgb_to_hsv_u8(px)
        self.assertEqual(h.tolist(), [[0, 60, 120, 0]])
        self.assertEqual(s.tolist(), [[255, 255, 255, 0]])
        self.assertEqual(v.tolist(), [[255, 255, 255, 128]])


HAVE_CV2 = gv.load_cv2()


class DetectorCases:
    """OpenCV 版 / numpy 版の両方で同じ結果になるべきテスト (下のサブクラスで実行)。"""
    BACKEND = None

    def setUp(self):
        self.det = gv.ColorGoalDetector(backend=self.BACKEND)
        self.assertEqual(self.det.backend, self.BACKEND)

    # ── 単色画像 ──
    def test_solid_red_is_contact(self):
        r = self.det.detect(solid(RED))
        self.assertTrue(r.detected)
        self.assertGreater(r.area_ratio, 0.95)
        self.assertTrue(r.is_contact)
        self.assertTrue(r.is_near)
        self.assertEqual(r.edge_touch, 4)
        self.assertAlmostEqual(r.cx, 0.5, delta=0.02)

    def test_red_hue_wraparound_detected(self):
        # H ≈ 175 (青みのある赤) も赤として拾う
        r = self.det.detect(solid((200, 20, 45)))
        self.assertTrue(r.detected)

    def test_other_solid_colors_not_detected(self):
        cases = {
            "green": (30, 200, 30), "blue": (30, 30, 200), "white": (250, 250, 250),
            "black": (0, 0, 0), "gray": (128, 128, 128), "yellow": (220, 220, 0),
            "orange": (255, 140, 0), "grass": GRASS, "sky": (150, 190, 240),
            "soil_brown": (110, 75, 50), "dark_brown": (100, 60, 40),
            "skin": (230, 180, 150), "dark_red_too_dark": (45, 5, 5),
        }
        for name, rgb in cases.items():
            with self.subTest(name=name):
                r = self.det.detect(solid(rgb))
                self.assertFalse(r.detected, f"{name} {rgb} → {r.summary()}")

    def test_pale_red_in_bright_scene_detected(self):
        # 晴天で白っぽく写った赤 (明るい背景で閾値が下がる)
        img = with_rect((235, 235, 225), (235, 95, 90), 120, 80, 80, 100)
        r = self.det.detect(img)
        self.assertTrue(r.detected, r.summary())

    # ── 位置と操舵 ──
    def test_position_and_steer(self):
        cases = [(220, "RIGHT", 0.75), (40, "LEFT", 60 / 320), (140, "FORWARD", 0.5)]
        for x, expect, cx in cases:
            with self.subTest(x=x):
                img = with_rect(GRASS, RED, x, 90, 40, 60)
                r = self.det.detect(img)
                self.assertTrue(r.detected)
                self.assertAlmostEqual(r.cx, cx, delta=0.04)
                self.assertFalse(r.is_near)
                action, sec = gv.decide_steer(r)
                self.assertEqual(action, expect)
                self.assertGreater(sec, 0)

    def test_turn_pulse_is_proportional_and_clamped(self):
        small = gv.GoalDetection(detected=True, cx=0.65)
        big = gv.GoalDetection(detected=True, cx=0.99)
        _, s1 = gv.decide_steer(small)
        _, s2 = gv.decide_steer(big)
        self.assertLess(s1, s2)
        self.assertLessEqual(s2, 0.30)
        self.assertEqual(gv.decide_steer(gv.GoalDetection()), ("NONE", 0.0))

    def test_near_uses_short_forward_pulse(self):
        img = with_rect(GRASS, RED, 100, 40, 120, 160)   # 面積 25%
        r = self.det.detect(img)
        self.assertTrue(r.is_near)
        self.assertFalse(r.is_contact)
        self.assertEqual(gv.decide_steer(r), ("FORWARD", 0.25))

    def test_small_but_valid_blob(self):
        img = with_rect(GRASS, RED, 150, 100, 16, 24)    # 384px = 0.5%
        r = self.det.detect(img)
        self.assertTrue(r.detected)
        self.assertFalse(r.is_near)

    def test_scattered_noise_rejected(self):
        rng = np.random.default_rng(1)
        img = solid(GRASS)
        ys = rng.integers(0, H, 400)
        xs = rng.integers(0, W, 400)
        img[ys, xs] = RED                                  # 0.5% の点ノイズ
        r = self.det.detect(img)
        self.assertFalse(r.detected, r.summary())

    def test_largest_blob_selected(self):
        img = with_rect(GRASS, RED, 20, 100, 24, 32)
        img[60:180, 220:300] = RED
        r = self.det.detect(img)
        self.assertTrue(r.detected)
        self.assertGreater(r.cx, 0.7)

    def test_contact_when_bottom_covered(self):
        img = solid(GRASS)
        img[60:, :] = RED                                  # 下 75% が赤
        r = self.det.detect(img)
        self.assertTrue(r.is_contact)
        self.assertGreaterEqual(r.edge_touch, 3)

    def test_downscale_large_input(self):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[:] = GRASS
        img[200:300, 400:480] = RED
        r = self.det.detect(img)
        self.assertEqual((r.width, r.height), (320, 240))
        self.assertTrue(r.detected)
        self.assertAlmostEqual(r.cx, 440 / 640, delta=0.04)

    def test_invalid_input(self):
        self.assertFalse(self.det.detect(None).detected)
        self.assertFalse(self.det.detect(np.zeros((240, 320), dtype=np.uint8)).detected)
        self.assertFalse(self.det.detect(np.zeros((4, 4, 3), dtype=np.uint8)).detected)

    def test_rgba_input(self):
        img = np.zeros((H, W, 4), dtype=np.uint8)
        img[..., :3] = RED
        self.assertTrue(self.det.detect(img).detected)


class TestDetectorNumpy(DetectorCases, unittest.TestCase):
    BACKEND = "numpy"


@unittest.skipUnless(HAVE_CV2, "OpenCV 未インストール")
class TestDetectorCV2(DetectorCases, unittest.TestCase):
    BACKEND = "cv2"


def cone(img, cx, bottom, h, base_w, color=RED, top_w=None):
    """台形 (上が細い) のコーンを描く。top_w=None なら頂点幅 = base_w * 0.2。"""
    top_w = base_w * 0.2 if top_w is None else top_w
    for i in range(h):
        y = bottom - h + i
        wid = top_w + (base_w - top_w) * i / max(h - 1, 1)
        img[y, int(round(cx - wid / 2)):int(round(cx + wid / 2)) + 1] = color
    return img


@unittest.skipUnless(HAVE_CV2, "OpenCV 未インストール")
class TestShapeCV2(unittest.TestCase):
    """OpenCV 版だけの形状判定 (コーンらしくない赤を除外する)。"""

    def setUp(self):
        self.det = gv.ColorGoalDetector(backend="cv2")

    def test_hsv_matches_numpy_impl(self):
        rng = np.random.default_rng(2)
        px = rng.integers(0, 256, size=(20, 30, 3), dtype=np.uint8)
        h1, s1, v1 = gv.rgb_to_hsv_u8(px)
        hsv = gv.cv2.cvtColor(px, gv.cv2.COLOR_RGB2HSV)
        dh = np.abs(h1.astype(int) - hsv[..., 0].astype(int)) % 180
        self.assertLessEqual(np.minimum(dh, 180 - dh).max(), 1)
        self.assertLessEqual(np.abs(s1.astype(int) - hsv[..., 1]).max(), 1)
        self.assertLessEqual(np.abs(v1.astype(int) - hsv[..., 2]).max(), 1)

    def test_cone_shape_accepted(self):
        img = cone(solid(GRASS), 200, 200, 90, 50)
        r = self.det.detect(img)
        self.assertTrue(r.detected, r.summary())
        self.assertGreater(r.shape_score, 0.6)
        self.assertAlmostEqual(r.cx, 200 / 320, delta=0.03)
        self.assertGreater(r.aspect, 1.2)

    def test_cone_with_white_band_is_one_region(self):
        img = cone(solid(GRASS), 160, 200, 90, 50)
        img[140:144] = np.where(img[140:144] == RED, 240, img[140:144])   # 白い反射帯
        r = self.det.detect(img)
        self.assertTrue(r.detected)
        self.assertGreater(r.bbox[3], 80, "帯で上下に割れずに 1 つの領域になる")

    def test_flat_red_strip_rejected(self):
        img = with_rect(GRASS, RED, 60, 150, 200, 14)     # 地面の赤いテープ/線
        r = self.det.detect(img)
        self.assertFalse(r.detected, r.summary())

    def test_inverted_triangle_rejected(self):
        img = solid(GRASS)
        for i in range(80):                                # 上が広く下が尖る
            wid = 60 - 55 * i / 79
            img[60 + i, int(160 - wid / 2):int(160 + wid / 2) + 1] = RED
        r = self.det.detect(img)
        self.assertFalse(r.detected, r.summary())

    def test_thin_ring_rejected(self):
        img = with_rect(GRASS, RED, 100, 60, 80, 120)
        img[66:174, 106:174] = GRASS                       # 中抜きの枠
        r = self.det.detect(img)
        self.assertFalse(r.detected, r.summary())

    def test_cone_preferred_over_larger_flat_object(self):
        img = with_rect(GRASS, RED, 10, 200, 150, 25)      # 横長の赤 (面積 3750)
        cone(img, 250, 180, 70, 40)                        # コーン (面積 ~1700)
        r = self.det.detect(img)
        self.assertTrue(r.detected)
        self.assertGreater(r.cx, 0.7, r.summary())
        self.assertEqual(r.n_candidates, 2)   # 候補は 2 つ (帯は形状で落ちる)

    def test_near_large_region_skips_shape(self):
        img = with_rect(GRASS, RED, 40, 30, 240, 80)        # 近すぎて横長に写っても OK
        img[110:200, 40:280] = RED
        r = self.det.detect(img)
        self.assertTrue(r.detected)
        self.assertTrue(r.is_near)
        self.assertEqual(r.shape_score, -1.0)

    def test_edge_cut_region_skips_shape(self):
        img = with_rect(GRASS, RED, 0, 150, 120, 20)       # 画面端で切れた物は形で判断しない
        self.assertTrue(self.det.detect(img).detected)

    def test_far_cone_detected(self):
        img = cone(solid(GRASS), 100, 130, 20, 12)         # 約 9m 先のコーン (高さ 20px)
        r = self.det.detect(img)
        self.assertTrue(r.detected, r.summary())


class TestBackendSelection(unittest.TestCase):
    def test_auto(self):
        d = gv.ColorGoalDetector()
        self.assertEqual(d.backend, "cv2" if HAVE_CV2 else "numpy")

    def test_invalid(self):
        with self.assertRaises(ValueError):
            gv.ColorGoalDetector(backend="foo")


class TestSonarJudge(unittest.TestCase):
    def setUp(self):
        self.cfg = gv.GoalCheckConfig()
        self.near = gv.GoalDetection(detected=True, area_ratio=0.4, is_near=True)
        self.contact = gv.GoalDetection(detected=True, area_ratio=0.9, is_near=True, is_contact=True)
        self.small = gv.GoalDetection(detected=True, area_ratio=0.01)
        self.none = gv.GoalDetection()

    def s(self, *vals):
        return gv.summarize_sonar(vals)

    def test_summarize(self):
        sm = self.s(0.05, None, 0.07, 9.9, 0.06, 0.001)
        self.assertEqual((sm.n_total, sm.n_valid), (6, 3))
        self.assertAlmostEqual(sm.median, 0.06)
        self.assertAlmostEqual(sm.spread, 0.02)
        sm = self.s(0.04, 0.06)
        self.assertAlmostEqual(sm.median, 0.05)
        sm = self.s(None, None)
        self.assertIsNone(sm.median)
        self.assertEqual(sm.n_valid, 0)

    def test_judge_goal_branches(self):
        cases = [
            ("CONFIRMED", self.s(0.05, 0.06, 0.05, 0.07, 0.06), self.near),
            ("CONFIRMED", self.s(0.03, 0.04, 0.03), self.contact),
            ("FAR", self.s(0.30, 0.31, 0.30), self.near),
            ("OBSTACLE", self.s(0.05, 0.05, 0.06), self.small),
            ("BLIND", self.s(None, None, None, None, None), self.contact),
            ("BLIND", self.s(0.02, 1.5, 3.0), self.contact),           # 至近で値が暴れる
            ("NO_VISUAL", self.s(0.05, 0.05, 0.05), self.none),
            ("SONAR_INVALID", self.s(None, 0.05, None, None, None), self.near),
            ("NOISY", self.s(0.05, 0.30, 0.6), self.near),
        ]
        for expect, son, det in cases:
            with self.subTest(expect=expect, son=son.values):
                status, reason = gv.judge_goal(son, det, self.cfg)
                self.assertEqual(status, expect, reason)

    def test_boundary(self):
        st, _ = gv.judge_goal(self.s(0.08, 0.08, 0.08), self.near, self.cfg)
        self.assertEqual(st, "CONFIRMED")
        st, _ = gv.judge_goal(self.s(0.081, 0.081, 0.081), self.near, self.cfg)
        self.assertEqual(st, "FAR")

    def test_judge_backoff(self):
        ok, _ = gv.judge_backoff(self.s(0.10, 0.11, 0.10), self.near, self.cfg)
        self.assertTrue(ok)
        for son, det in [(self.s(0.5, 0.5, 0.5), self.near),       # ゴールが離れている
                         (self.s(None, None, None), self.near),     # 後退しても測れない
                         (self.s(0.1, 0.1, 0.1), self.none),        # カメラで見えない
                         (self.s(0.05, 0.15, 0.19), self.near)]:    # 不安定
            with self.subTest(son=son.values):
                ok, reason = gv.judge_backoff(son, det, self.cfg)
                self.assertFalse(ok, reason)

    def test_judge_after_push(self):
        ok, _ = gv.judge_after_push(self.s(0.03, 0.04, 0.03), self.contact, self.cfg)
        self.assertTrue(ok)
        ok, _ = gv.judge_after_push(self.s(None, None, None), self.contact, self.cfg)
        self.assertTrue(ok)   # 接触 → 不感帯、カメラは至近で捉えている
        ok, _ = gv.judge_after_push(self.s(0.5, 0.5, 0.5), self.contact, self.cfg)
        self.assertFalse(ok)  # ゴールが倒れた/ずれた
        ok, _ = gv.judge_after_push(self.s(None, None, None), self.none, self.cfg)
        self.assertFalse(ok)  # 何も確認できない


class FakePicam:
    """Picamera2 の代わり: "RGB888" 形式 = 配列上 B,G,R の順で返す。"""
    def __init__(self, bgr):
        self.bgr = bgr
        self.calls = 0

    def capture_array(self, name="main"):
        self.calls += 1
        return self.bgr.copy()


class TestCameraWrapper(unittest.TestCase):
    def _cap(self, **kw):
        cap = gv.PiCameraCapture(log=lambda m: None, **kw)
        bgr = np.zeros((2, 2, 3), dtype=np.uint8)
        bgr[0, 0] = (10, 20, 200)          # B,G,R → 赤
        bgr[1, 1] = (200, 20, 10)          # → 青
        cap.cam = FakePicam(bgr)
        return cap

    def test_bgr_to_rgb(self):
        cap = self._cap()
        rgb = cap.capture_rgb(discard=1)
        self.assertEqual(rgb[0, 0].tolist(), [200, 20, 10])
        self.assertEqual(cap.cam.calls, 2)   # 1 枚捨てて 1 枚使う

    def test_swap_and_rotate(self):
        rgb = self._cap(swap_rb=True).capture_rgb(discard=0)
        self.assertEqual(rgb[0, 0].tolist(), [10, 20, 200])
        rgb = self._cap(rotate180=True).capture_rgb(discard=0)
        self.assertEqual(rgb[1, 1].tolist(), [200, 20, 10])

    def test_capture_without_camera(self):
        self.assertIsNone(gv.PiCameraCapture(log=lambda m: None).capture_rgb())


class TestImageIO(unittest.TestCase):
    def test_save_and_load_roundtrip(self):
        img = with_rect(GRASS, RED, 200, 80, 60, 80)
        det = gv.ColorGoalDetector().detect(img)
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as d:
            p = Path(d) / "x.jpg"
            gv.save_rgb_jpeg(img, p, det)
            back = gv.load_image_rgb(p)
            self.assertEqual(back.shape, img.shape)
            self.assertTrue(gv.ColorGoalDetector().detect(back).detected)


if __name__ == "__main__":
    unittest.main(verbosity=2)
