"""
mission パッケージ
==================
CanSat ミッションシーケンス (main6.py を分割したもの)。
各フェーズは 1 ファイルずつ、設定値は config.py、フェーズ間で共有する
センサ値などの状態は state.py にまとめている。main/main.py がフェーズの
切り替えだけを行う。

sensor/ のモジュール (BNO055, BMP180, micropyGPS, goal_vision, ...) を
import できるよう、ここで sys.path に sensor/ を追加する。
"""
import sys
from pathlib import Path

_ROOT_DIR = Path(__file__).resolve().parents[2]      # TRC2027/
_SENSOR_DIR = _ROOT_DIR / "sensor"
if str(_SENSOR_DIR) not in sys.path:
    sys.path.insert(0, str(_SENSOR_DIR))
