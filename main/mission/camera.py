"""
camera.py
=========
カメラ・色検知器・超音波ゴール確認設定を config から組み立てる (make_*)。
(main6.py から機械的に分割。動作は変更していない)
元の位置: main6.py 2403〜2427 行
"""

import goal_vision as gv

from . import config as C
from .logger import log

def make_goal_detector(backend: str = None) -> "gv.ColorGoalDetector":
    """設定値セクションの CAM_* から色検知器を作る。"""
    return gv.ColorGoalDetector(
        hue_ranges=C.CAM_HUE_RANGES, s_min=C.CAM_S_MIN, v_min=C.CAM_V_MIN,
        red_margin=C.CAM_RED_MARGIN, red_max_gb_ratio=C.CAM_RED_MAX_GB_RATIO,
        work_width=C.CAM_SIZE[0],
        min_area_ratio=C.CAM_MIN_AREA_RATIO, near_area_ratio=C.CAM_NEAR_AREA_RATIO,
        contact_area_ratio=C.CAM_CONTACT_AREA_RATIO,
        backend=backend or C.CAM_BACKEND, shape_min_score=C.CAM_SHAPE_MIN_SCORE)


def make_goal_check_config() -> "gv.GoalCheckConfig":
    """設定値セクションの GOAL_* から超音波ゴール確認の閾値をまとめる。"""
    return gv.GoalCheckConfig(
        goal_dist_m=C.GOAL_SONAR_DIST_M, min_valid=C.GOAL_SONAR_MIN_VALID,
        max_spread_m=C.GOAL_SONAR_MAX_SPREAD_M,
        backoff_confirm_dist_m=C.GOAL_BACKOFF_CONFIRM_DIST_M,
        push_tolerance_m=C.GOAL_PUSH_TOLERANCE_M)


def make_camera() -> "gv.PiCameraCapture":
    return gv.PiCameraCapture(size=C.CAM_SIZE, full_fov=C.CAM_FULL_FOV,
                              rotate180=C.CAM_ROTATE_180, swap_rb=C.CAM_SWAP_RB,
                              warmup_sec=C.CAM_WARMUP_SEC, log=log)
