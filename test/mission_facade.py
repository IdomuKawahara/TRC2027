"""
mission_facade.py
=================
mission パッケージを「1 つのモジュール」のように
扱うためのテスト用アダプタ。

シミュレーションテスト (test_phase5_sim.py, test_phase4_sim.py) は
`M.FINAL_APPROACH_TIMEOUT_SEC = 30` や `M.log = fake` のように 1 つのモジュール M の
属性を直接読み書きする。しかし設定値は mission.config、共有状態は
mission.state、関数は各フェーズモジュールにあり、さらに `log` などは
`from .logger import log` で各モジュールに取り込まれているため、
1 か所を差し替えても他のモジュールには反映されない。

この Facade は
  * 読み出し: config → state → 各モジュール の順に探す
  * 書き込み: その名前を持つ **すべての** mission モジュールに書く
             (log, time, make_camera などの差し替えが全モジュールに効く)
  * 無い名前 (print など): 全モジュールに追加し、del で全部から消す
ことで、分割前と同じ使い勝手でテストを流せるようにする。
"""
import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT / "sensor", ROOT / "main"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from mission import (config, state, logger, hardware, navigation, camera,       # noqa: E402
                     phase0_init, phase1_fall, phase2_separation,
                     phase3_calibration, phase4_gps, phase5_camera)


def load_main_module():
    """main/main.py (フェーズ切り替え) をモジュールとして読み込む (実行はしない)。"""
    spec = importlib.util.spec_from_file_location("mission_main", ROOT / "main" / "main.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class MissionFacade:
    def __init__(self):
        main_mod = load_main_module()
        object.__setattr__(self, "_mods", [config, state, logger, hardware, navigation, camera,
                                           phase0_init, phase1_fall, phase2_separation,
                                           phase3_calibration, phase4_gps, phase5_camera, main_mod])
        object.__setattr__(self, "_extra", {})
        object.__setattr__(self, "main_module", main_mod)

    def __getattr__(self, name):
        if name in self._extra:
            return self._extra[name]
        for m in self._mods:
            if hasattr(m, name):
                return getattr(m, name)
        raise AttributeError(name)

    def __setattr__(self, name, value):
        owners = [m for m in self._mods if hasattr(m, name)]
        if name in self._extra or not owners:
            self._extra[name] = value
            for m in self._mods:
                setattr(m, name, value)
            return
        for m in owners:
            setattr(m, name, value)

    def __delattr__(self, name):
        if name in self._extra:
            del self._extra[name]
            for m in self._mods:
                if hasattr(m, name):
                    delattr(m, name)
            return
        raise AttributeError(name)
