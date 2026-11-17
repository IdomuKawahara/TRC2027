"""
vehicle.py
==========
config.py の「どの機体として動かすか (VEHICLE)」を支える裏方。
使う人が触るのは config.py だけで、このファイルは編集しなくてよい。

config.py の流れ:
    VEHICLE = "honban"                      ← 一番上。過去機体で試すときだけ "old" に書き換える
    ...(今までの設定)...
    _before = snapshot(globals())            ← ここまでの値 (= 本番機の値) を覚える
    if VEHICLE == "old":                     ← 一番下。過去機体だけ違う値
        FIG8_SPIN_SEC = 12.0
    VEHICLE_OVERRIDES = check_changes(...)   ← 何が変わったかを調べる + 書き間違いを見つける

check_changes() が起動時に止めるもの (事故防止):
    - config.py に無い名前 (打ち間違い。Python はエラーにしないので、値が効かないまま走ってしまう)
    - 型の違い (数値の所に文字、True/False の所に数値 など)
    - 他の値から計算される値 (WEAK_RATIO など) を直接書き換えた
"""
import difflib
import os

COMPETITION = "honban"          # 本番機の名前
ENV_VAR = "CANSAT_VEHICLE"      # 自動テスト用: config.py を書き換えずに機体を指定する


class VehicleError(Exception):
    """config.py の機体の設定に誤りがある (起動時に止める)。"""


def choose(name, known, env=None):
    """
    使う機体名を決める。戻り値: (機体名, "config" か "env")
    通常は config.py の VEHICLE。環境変数 CANSAT_VEHICLE があればそちらを使う (自動テスト用)。
    """
    env = os.environ if env is None else env
    source = "config"
    if env.get(ENV_VAR, "").strip():
        name, source = env[ENV_VAR].strip(), "env"
    if name not in known:
        where = f"環境変数 {ENV_VAR}" if source == "env" else "main/mission/config.py の VEHICLE"
        raise VehicleError(f"{where} = {name!r} は知らない機体です。使える機体: {list(known)}"
                           f"  (新しい機体を足すときは config.py の VEHICLES にも名前を書く)")
    return name, source


def _is_setting(key):
    return key.isupper() and not key.startswith("_")


def snapshot(namespace):
    """今の設定値 (大文字の名前) を写し取る。"""
    return {k: v for k, v in namespace.items() if _is_setting(k)}


def _same_kind(old, new):
    if old is None or new is None:
        return True
    if isinstance(old, bool) or isinstance(new, bool):
        return isinstance(old, bool) and isinstance(new, bool)
    if isinstance(old, (int, float)) and isinstance(new, (int, float)):
        return not (isinstance(old, int) and isinstance(new, float))   # ピン番号などに 5.0 は不可
    if isinstance(old, (tuple, list)) and isinstance(new, (tuple, list)):
        return True
    return type(old) is type(new)


def check_changes(namespace, before, protected=(), where="config.py の if VEHICLE == ... の中"):
    """
    before (snapshot) と比べて、機体ごとの値で変わったものを調べる。
    書き間違いがあれば VehicleError。戻り値: [(名前, 本番機の値, この機体の値), ...]
    int を書いた所が元は float なら float に直す (例: 12 → 12.0)。
    """
    changes = []
    for key in sorted(k for k in namespace if _is_setting(k)):
        new = namespace[key]
        if key not in before:
            near = difflib.get_close_matches(key, list(before), n=3, cutoff=0.6)
            hint = f"  もしかして: {', '.join(near)}" if near else ""
            raise VehicleError(f"{where}: {key} は config.py にありません (名前の打ち間違い?){hint}")
        old = before[key]
        if new is old or (type(new) is type(old) and new == old):
            continue
        if key in protected:
            raise VehicleError(f"{where}: {key} は他の値から計算されるので、直接は変えられません")
        if not _same_kind(old, new):
            raise VehicleError(f"{where}: {key} の型が違います (本番機の値 {old!r} → {new!r})")
        if isinstance(old, float) and isinstance(new, int):
            new = namespace[key] = float(new)
        if isinstance(old, tuple) and isinstance(new, list):
            new = namespace[key] = tuple(new)
        if isinstance(old, tuple) and len(old) != len(new) and all(not isinstance(x, tuple) for x in old):
            raise VehicleError(f"{where}: {key} の個数が違います (本番機の値 {old!r} → {new!r})")
        if new != old:
            changes.append((key, old, new))
    return changes
