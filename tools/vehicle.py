"""
vehicle.py
==========
機体プロファイルの確認・切り替え・新規作成。モータやセンサは動かさない。

    cd ~/TRC2027
    python3 tools/vehicle.py              # 今の機体と、使える機体の一覧
    python3 tools/vehicle.py show [名前]  # その機体の設定 (config.py との違い) とキャリブレーションの有無
    python3 tools/vehicle.py use old      # この Pi を「old」の機体として動かす (config/active_vehicle に書く)
    python3 tools/vehicle.py use honban   # 本番機に戻す
    python3 tools/vehicle.py new 名前     # 新しい機体のプロファイルを作る (old をひな形にコピー)
    python3 tools/vehicle.py check        # 本番前の確認: 本番機の設定になっているか

一時的に別の機体として 1 回だけ動かすとき:
    CANSAT_VEHICLE=old python3 main/main.py

仕組みは main/mission/vehicle.py と docs/VEHICLES.md を参照。
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "main"))

from mission import vehicle as V    # noqa: E402  (ハードウェアには触れない)

TEMPLATE = "old"


def load_config_for(name):
    """その機体として config.py を読み込んだ結果を返す (別プロセスで読むので今の設定に影響しない)。"""
    code = ("import json, sys; sys.path.insert(0, %r)\n"
            "from mission import config as C\n"
            "print(json.dumps({'name': C.VEHICLE_NAME, 'role': C.VEHICLE_ROLE, 'desc': C.VEHICLE_DESCRIPTION,"
            " 'overrides': [[k, repr(o), repr(n)] for k, o, n in C.VEHICLE_OVERRIDES],"
            " 'calib': str(C.BNO_CALIB_FILE)}))") % str(ROOT / "main")
    env = dict(os.environ, **{V.ENV_VAR: name})
    proc = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, env=env)
    if proc.returncode != 0:
        err = [l for l in proc.stderr.strip().splitlines() if l.strip()]
        return None, "\n".join(err[-4:])
    return json.loads(proc.stdout.strip().splitlines()[-1]), ""


def calib_summary(path):
    path = Path(path)
    if not path.exists():
        return "なし (tools/bno055_calibrate.py で作る)"
    try:
        obj = json.loads(path.read_text(encoding="utf-8"))
        uid = obj.get("sensor_uid")
        return (f"あり 保存 {obj.get('saved_at', '?')} 状態 {obj.get('calib_status_sys_gyro_acc_mag')}"
                + (f" センサ …{uid[-8:]}" if uid else ""))
    except Exception as e:
        return f"壊れています ({e})"


def cmd_status(_args):
    try:
        name, source = V.active_vehicle_name()
    except V.VehicleError as e:
        print(f"エラー: {e}")
        return 1
    src = {"env": f"環境変数 {V.ENV_VAR}", "file": "config/active_vehicle",
           "default": "未設定 (既定の本番機)"}[source]
    print(f"今の機体: {name}   [{src}]")
    print("使える機体:")
    for v in V.list_vehicles():
        try:
            p = V.load_profile(v)
            mark = "→" if v == name else "  "
            print(f"  {mark} {v:12s} {'本番機' if p['role'] == V.ROLE_COMPETITION else '試験用'}  "
                  f"{p['description']}  (差分 {len(p['overrides'])} 個)")
        except V.VehicleError as e:
            print(f"     {v:12s} ★設定ミス: {str(e).splitlines()[0]}")
    return 0


def cmd_show(args):
    name = args.name or V.active_vehicle_name()[0]
    cfg, err = load_config_for(name)
    if cfg is None:
        print(f"機体 {name} の設定を読み込めません:\n{err}")
        return 1
    print(f"機体: {cfg['name']}  ({cfg['desc']})  {'本番機' if cfg['role'] == V.ROLE_COMPETITION else '試験用'}")
    print(f"設定ファイル: config/vehicles/{name}/vehicle.toml")
    if cfg["overrides"]:
        print(f"config.py との違い ({len(cfg['overrides'])} 個):")
        for k, old, new in cfg["overrides"]:
            print(f"  {k} = {new}   (本番機: {old})")
    else:
        print("config.py との違い: なし (config.py の値そのまま)")
    print(f"BNO055 キャリブレーション: {calib_summary(cfg['calib'])}")
    return 0


def cmd_use(args):
    name = args.name
    cfg, err = load_config_for(name)        # 設定ミスのある機体には切り替えない
    if cfg is None:
        print(f"機体 {name} には切り替えられません:\n{err}")
        return 1
    V.ACTIVE_FILE.parent.mkdir(parents=True, exist_ok=True)
    if name == V.DEFAULT_VEHICLE:
        if V.ACTIVE_FILE.exists():
            V.ACTIVE_FILE.unlink()          # 未設定 = 本番機。ファイルを残さないのが一番安全
        print(f"本番機 ({name}) に戻しました (config/active_vehicle を削除)")
    else:
        V.ACTIVE_FILE.write_text(name + "\n", encoding="utf-8")
        print(f"この Pi を機体「{name}」として動かします (config/active_vehicle に記録)")
        print("★ 本番機に載せ替える前に必ず: python3 tools/vehicle.py use honban")
    if os.environ.get(V.ENV_VAR):
        print(f"注意: 環境変数 {V.ENV_VAR}={os.environ[V.ENV_VAR]} が設定されていて、そちらが優先されます")
    return cmd_show(argparse.Namespace(name=name))


def cmd_new(args):
    name = args.name
    if not V.valid_name(name):
        print("機体名は英数字・_・- のみ、32 文字以内にしてください")
        return 1
    dst = V.vehicle_dir(name)
    if dst.exists():
        print(f"既にあります: {dst}")
        return 1
    src = V.vehicle_dir(args.copy_from) / "vehicle.toml"
    if not src.is_file():
        print(f"ひな形がありません: {src}")
        return 1
    dst.mkdir(parents=True)
    shutil.copyfile(src, dst / "vehicle.toml")      # キャリブレーション結果はコピーしない (機体ごとに測る)
    print(f"作りました: {dst / 'vehicle.toml'}  (ひな形: {args.copy_from})")
    print(f"  1. このファイルの description と [overrides] を機体に合わせて書く")
    print(f"  2. python3 tools/vehicle.py use {name}")
    print(f"  3. python3 tools/bno055_calibrate.py   (この機体でキャリブレーションを取り直す)")
    return 0


def cmd_check(_args):
    """本番前の確認。本番機の設定で、設定ミスが無く、キャリブレーションがあれば 0。"""
    problems = []
    try:
        name, source = V.active_vehicle_name()
    except V.VehicleError as e:
        print(f"❌ {e}")
        return 1
    cfg, err = load_config_for(name)
    if cfg is None:
        print(f"❌ 機体 {name} の設定を読み込めません:\n{err}")
        return 1
    if cfg["role"] != V.ROLE_COMPETITION:
        problems.append(f"今の機体は試験用の「{name}」です → python3 tools/vehicle.py use honban")
    if os.environ.get(V.ENV_VAR):
        problems.append(f"環境変数 {V.ENV_VAR} が設定されています (本番では外す)")
    if cfg["overrides"]:
        problems.append(f"本番機に config.py との違いが {len(cfg['overrides'])} 個あります "
                        f"(意図したものか確認): {[o[0] for o in cfg['overrides']]}")
    if not Path(cfg["calib"]).exists():
        problems.append("この機体の BNO055 キャリブレーションがありません → python3 tools/bno055_calibrate.py")
    print(f"機体: {name}  キャリブレーション: {calib_summary(cfg['calib'])}")
    if problems:
        for p in problems:
            print(f"❌ {p}")
        return 1
    print("✅ 本番機の設定です")
    return 0


def main():
    ap = argparse.ArgumentParser(description="機体プロファイルの確認・切り替え・新規作成")
    sub = ap.add_subparsers(dest="cmd")
    sub.add_parser("status", help="今の機体と一覧 (既定)")
    p = sub.add_parser("show", help="機体の設定を表示")
    p.add_argument("name", nargs="?")
    p = sub.add_parser("use", help="この Pi をその機体として動かす")
    p.add_argument("name")
    p = sub.add_parser("new", help="新しい機体のプロファイルを作る")
    p.add_argument("name")
    p.add_argument("--copy-from", default=TEMPLATE, help=f"ひな形にする機体 (既定 {TEMPLATE})")
    sub.add_parser("check", help="本番前の確認")
    args = ap.parse_args()
    return {"status": cmd_status, "show": cmd_show, "use": cmd_use, "new": cmd_new,
            "check": cmd_check, None: cmd_status}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
