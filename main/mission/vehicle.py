"""
vehicle.py
==========
機体プロファイル: 機体ごとに違う設定 (ピン・取り付け向き・パルス長など) と
キャリブレーション結果を、機体の名前で分けて管理する。

    config/vehicles/<機体名>/vehicle.toml        その機体だけの設定 (config.py との差分)
    config/vehicles/<機体名>/bno055_calib.json   その機体の BNO055 キャリブレーション結果
    config/active_vehicle                        この Pi (SD カード) が今どの機体に載っているか
                                                 (git には入れない。Pi ごとに持つ)

どの機体として動くかは、次の順で決まる:
    1. 環境変数 CANSAT_VEHICLE            (一時的に切り替えるとき・自動テスト)
    2. config/active_vehicle の中身       (tools/vehicle.py use <名前> で書く)
    3. どちらも無ければ DEFAULT_VEHICLE   (= 本番機 "honban")

★ 何も設定していない Pi は必ず本番機として動く。git pull しても機体は切り替わらない。

vehicle.toml の書き方:
    [vehicle]
    description = "説明"
    role = "competition"   # 本番機は "competition"、それ以外は "test"

    [overrides]            # config.py の値のうち、この機体で変えたいものだけ
    CAM_ROTATE_180 = false
    PIN_AIN1 = 5

[overrides] に書けるのは config.py に既にある名前だけ。名前の打ち間違いや型の違い
(数値のところに文字列など) は、起動時にエラーにして止める (黙って無視すると、
違うピンでモータを回すなどの事故になるため)。
"""

import os
import re
import tomllib
from pathlib import Path

_ROOT_DIR = Path(__file__).resolve().parents[2]          # TRC2027/
VEHICLES_DIR = _ROOT_DIR / "config" / "vehicles"
ACTIVE_FILE = _ROOT_DIR / "config" / "active_vehicle"
ENV_VAR = "CANSAT_VEHICLE"
DEFAULT_VEHICLE = "honban"
ROLE_COMPETITION = "competition"
ROLE_TEST = "test"
_NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,32}$")


class VehicleError(ValueError):
    """機体プロファイルの設定ミス (起動時に止めるためのエラー)。"""


def valid_name(name: str) -> bool:
    return bool(_NAME_RE.match(name or ""))


def active_vehicle_name(env=None, active_file=None):
    """今の機体名と、それをどこから決めたかを返す。戻り値: (名前, "env" / "file" / "default")"""
    env = os.environ if env is None else env
    active_file = ACTIVE_FILE if active_file is None else Path(active_file)
    name = (env.get(ENV_VAR) or "").strip()
    source = "env"
    if not name:
        source = "file"
        try:
            name = active_file.read_text(encoding="utf-8").strip().splitlines()[0].strip()
        except (OSError, IndexError):
            name = ""
    if not name:
        return DEFAULT_VEHICLE, "default"
    if not valid_name(name):
        raise VehicleError(f"機体名 {name!r} が不正です (英数字・_・- のみ, 32 文字以内)。"
                           f"{ENV_VAR} または {active_file} を確認してください")
    return name, source


def vehicle_dir(name: str, vehicles_dir=None) -> Path:
    return (VEHICLES_DIR if vehicles_dir is None else Path(vehicles_dir)) / name


def list_vehicles(vehicles_dir=None):
    d = VEHICLES_DIR if vehicles_dir is None else Path(vehicles_dir)
    if not d.is_dir():
        return []
    return sorted(p.name for p in d.iterdir() if (p / "vehicle.toml").is_file())


def load_profile(name: str, vehicles_dir=None) -> dict:
    """
    vehicle.toml を読む。戻り値: {"name", "description", "role", "overrides", "dir", "exists"}
    本番機 (DEFAULT_VEHICLE) のファイルが無い場合だけは「差分なし」として動かす
    (本番で設定ファイルが 1 つ欠けただけで起動できなくなるのを避ける)。
    それ以外の機体でファイルが無ければエラー。
    """
    d = vehicle_dir(name, vehicles_dir)
    path = d / "vehicle.toml"
    if not path.is_file():
        if name == DEFAULT_VEHICLE:
            return {"name": name, "description": "本番機 (vehicle.toml なし: config.py のまま)",
                    "role": ROLE_COMPETITION, "overrides": {}, "dir": d, "exists": False}
        raise VehicleError(f"機体 {name!r} のプロファイルがありません: {path}\n"
                           f"  使える機体: {list_vehicles(vehicles_dir) or 'なし'}\n"
                           f"  新しく作るには: python3 tools/vehicle.py new {name}")
    try:
        with open(path, "rb") as f:
            doc = tomllib.load(f)
    except tomllib.TOMLDecodeError as e:
        raise VehicleError(f"{path} の書き方が正しくありません: {e}") from e
    meta = doc.get("vehicle", {})
    role = meta.get("role", ROLE_COMPETITION if name == DEFAULT_VEHICLE else ROLE_TEST)
    if role not in (ROLE_COMPETITION, ROLE_TEST):
        raise VehicleError(f"{path}: role は \"{ROLE_COMPETITION}\" か \"{ROLE_TEST}\" (今は {role!r})")
    overrides = doc.get("overrides", {})
    if not isinstance(overrides, dict):
        raise VehicleError(f"{path}: [overrides] は「名前 = 値」の表で書いてください")
    extra = set(doc) - {"vehicle", "overrides"}
    if extra:
        raise VehicleError(f"{path}: 知らない項目 {sorted(extra)} があります ([vehicle] と [overrides] だけ使えます)")
    return {"name": name, "description": str(meta.get("description", "")), "role": role,
            "overrides": overrides, "dir": d, "exists": True}


def _to_tuple(v):
    return tuple(_to_tuple(x) for x in v) if isinstance(v, list) else v


def _coerce(key, old, new, where):
    """new を old と同じ型に合わせる。合わなければ VehicleError。"""
    def bad(expect):
        return VehicleError(f"{where}: {key} は {expect} で書いてください (今は {new!r}。config.py の値は {old!r})")
    if isinstance(old, bool):
        if not isinstance(new, bool):
            raise bad("true / false")
        return new
    if isinstance(old, int):
        if isinstance(new, bool) or not isinstance(new, int):
            raise bad("整数")
        return new
    if isinstance(old, float):
        if isinstance(new, bool) or not isinstance(new, (int, float)):
            raise bad("数値")
        return float(new)
    if isinstance(old, str):
        if not isinstance(new, str):
            raise bad("文字列")
        return new
    if isinstance(old, tuple):
        if not isinstance(new, list):
            raise bad("[ ] で囲んだ並び")
        new_t = _to_tuple(new)
        if len(new_t) != len(old) and not (old and isinstance(old[0], tuple)):
            raise bad(f"{len(old)} 個の並び")
        return new_t
    if old is None:
        if isinstance(new, bool) or not isinstance(new, (int, float)):
            raise bad("数値")
        return new
    raise VehicleError(f"{where}: {key} は機体プロファイルでは変えられない種類の値です")


def apply_overrides(namespace: dict, overrides: dict, protected=(), where="vehicle.toml"):
    """
    overrides を namespace (config.py の globals()) に書き込む。
    戻り値: 実際に変わった値の一覧 [(名前, 元の値, 新しい値), ...]
    """
    changed = []
    for key, new in overrides.items():
        if key in protected or not key.isupper() or key.startswith(("_", "VEHICLE_")):
            raise VehicleError(f"{where}: {key} は機体プロファイルでは変えられません")
        if key not in namespace:
            near = [k for k in namespace if k.isupper() and (k.startswith(key[:6]) or key in k)][:5]
            raise VehicleError(f"{where}: {key} という設定は config.py にありません (打ち間違い?)"
                               + (f" 近い名前: {near}" if near else ""))
        old = namespace[key]
        value = _coerce(key, old, new, where)
        if value != old:
            changed.append((key, old, value))
        namespace[key] = value
    return changed
