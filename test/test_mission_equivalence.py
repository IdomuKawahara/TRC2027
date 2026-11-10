"""
test_mission_equivalence.py
===========================
分割後の mission パッケージが、分割前 (main6.py) と同じ動作をすることの確認。

main6.py 向けに書かれた既存のシミュレーションテストを **1 文字も変えずに**、
テスト側の `M` を mission_facade.MissionFacade に差し替えて実行する。
  * test_phase5_camera.py     : Phase5 (カメラ最終接近) の 21 件
  * test_main6_improvements.py: Axis Remap / Phase4 走行 / Phase4⇄5 の流れ など 21 件

    python3 test/test_mission_equivalence.py
"""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mission_facade                       # noqa: E402
import test_phase5_camera as T5            # noqa: E402
import test_main6_improvements as T6       # noqa: E402

facade = mission_facade.MissionFacade()
T5.M = facade
T6.M = facade
# test_main6_improvements の World2D / SimMotor は定義時に M を参照しているものがある
T6.SimMotor.apply_diff = facade.MotorController.apply_diff


def suite():
    s = unittest.TestSuite()
    s.addTests(unittest.defaultTestLoader.loadTestsFromModule(T5))
    s.addTests(unittest.defaultTestLoader.loadTestsFromModule(T6))
    return s


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(suite())
    sys.exit(0 if result.wasSuccessful() else 1)
