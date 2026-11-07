"""
test_heading_fusion.py
======================
sensor/heading_fusion.py (地磁気 + GPS COG の方位融合) のユニットテスト。

    python3 test/test_heading_fusion.py
"""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sensor"))
import heading_fusion as hf  # noqa: E402


def step(est, t, mag, *, cog=None, speed=0.5, forward=True, gyro=0.0, fix_age=0.2, valid=True):
    return est.update(t, mag, gps_valid=valid, gps_speed_mps=speed,
                      gps_course=cog if cog is not None else 0.0,
                      gps_fix_time=t - fix_age, forward=forward, gyro_z_dps=gyro)


class TestWrap(unittest.TestCase):
    def test_wrap(self):
        self.assertEqual(hf.wrap180(190), -170)
        self.assertEqual(hf.wrap180(-190), 170)
        self.assertEqual(hf.wrap180(180), -180)
        self.assertEqual(hf.wrap360(-10), 350)


class TestHeadingEstimator(unittest.TestCase):
    def setUp(self):
        self.est = hf.HeadingEstimator()

    def drive_straight(self, true_heading, mag_bias, seconds=6.0, t0=0.0, dt=0.1):
        """真の方位 true_heading にまっすぐ走る。地磁気は mag_bias だけずれている。"""
        t = t0
        for i in range(int(seconds / dt)):
            t = t0 + i * dt
            fix = int(t)                         # GPS は 1Hz で更新
            self.est.update(t, (true_heading + mag_bias) % 360, gps_valid=True, gps_speed_mps=0.5,
                            gps_course=true_heading, gps_fix_time=float(fix),
                            forward=True, gyro_z_dps=1.0)
        return t

    def test_mag_only_before_learning(self):
        h = step(self.est, 0.0, 123.0, forward=False)
        self.assertEqual(h, 123.0)
        self.assertEqual(self.est.source, "MAG")

    def test_needs_straight_time_before_cog(self):
        step(self.est, 0.0, 100.0, cog=50.0)
        self.assertEqual(self.est.source, "MAG")           # 直進し始めたばかり
        h = step(self.est, 1.6, 100.0, cog=50.0)
        self.assertEqual(self.est.source, "COG")
        self.assertEqual(h, 50.0)

    def test_learns_offset_and_uses_it_when_turning(self):
        t = self.drive_straight(true_heading=80.0, mag_bias=+40.0)
        self.assertTrue(self.est.offset_ready)
        self.assertAlmostEqual(self.est.offset_deg, -40.0, delta=0.5)
        # 旋回中 (ジャイロ大) は COG を使わず、地磁気 + ずれ
        h = step(self.est, t + 0.1, (200.0 + 40.0) % 360, cog=80.0, gyro=60.0)
        self.assertEqual(self.est.source, "MAG+OFS")
        self.assertAlmostEqual(h, 200.0, delta=0.5)

    def test_offset_across_north(self):
        # 真の方位 350°、地磁気は +30° ずれて 20° と出る (0° をまたぐ)
        self.drive_straight(true_heading=350.0, mag_bias=+30.0)
        self.assertAlmostEqual(self.est.offset_deg, -30.0, delta=0.5)
        h = step(self.est, 100.0, 50.0, forward=False)
        self.assertAlmostEqual(h, 20.0, delta=0.5)

    def test_no_cog_when_slow_stale_invalid_or_not_forward(self):
        self.drive_straight(true_heading=10.0, mag_bias=0.0)
        n = self.est.n_samples
        for kw in (dict(speed=0.1), dict(fix_age=5.0), dict(valid=False), dict(forward=False)):
            with self.subTest(**kw):
                est = hf.HeadingEstimator()
                step(est, 0.0, 100.0, cog=50.0, **kw)
                h = step(est, 2.0, 100.0, cog=50.0, **kw)
                self.assertNotEqual(est.source, "COG")
                self.assertEqual(h, 100.0)
        self.assertEqual(self.est.n_samples, n)

    def test_same_fix_learned_once(self):
        step(self.est, 0.0, 100.0, cog=50.0)
        for t in (1.6, 1.7, 1.8, 1.9):                       # 同じ GPS fix (時刻 1.4)
            self.est.update(t, 100.0, gps_valid=True, gps_speed_mps=0.5, gps_course=50.0,
                            gps_fix_time=1.4, forward=True, gyro_z_dps=0.0)
        self.assertEqual(self.est.n_samples, 1)

    def test_single_mag_spike_ignored(self):
        t = self.drive_straight(true_heading=0.0, mag_bias=20.0)
        before = self.est.offset_deg
        # 1 回だけ地磁気が 100° 跳ねる
        self.est.update(t + 1, 120.0, gps_valid=True, gps_speed_mps=0.5, gps_course=0.0,
                        gps_fix_time=t + 0.9, forward=True, gyro_z_dps=0.0)
        self.assertAlmostEqual(self.est.offset_deg, before, delta=0.01)

    def test_persistent_change_relearned(self):
        t = self.drive_straight(true_heading=0.0, mag_bias=20.0)
        self.drive_straight(true_heading=0.0, mag_bias=-60.0, seconds=12.0, t0=t + 1)
        self.assertAlmostEqual(self.est.offset_deg, 60.0, delta=2.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
