"""
bno055_calibrate.py
===================
本番前に BNO055 をしっかりキャリブレーションし、その結果を
config/bno055_calib.json に保存するツール。本番 (main/main.py) は起動時 (Phase0) に
このファイルを読み込んで BNO055 に書き戻す。

TRC2027/tools/bno055_calibrate.py

使い方:
    cd ~/TRC2027
    python3 tools/bno055_calibrate.py            # 4 項目すべて 3 になったら自動で保存
    python3 tools/bno055_calibrate.py --partial  # Ctrl+C で止めたとき Gyro/Mag が 3 なら保存
    python3 tools/bno055_calibrate.py --show     # 保存済みのプロファイルを表示するだけ

キャリブレーションのやり方 (機体に取り付けた状態で、機体ごと手で持って行う):
    Gyro : 数秒間、机に置いて動かさない
    Acc  : 6 つの向き (上・下・左・右・前・後ろを上にする) で、それぞれ数秒ずつ静止
    Mag  : 空中で大きく 8 の字を描くように、ゆっくり機体を回す
           (モータ・電池を載せた本番と同じ状態で行うこと。磁気の影響ごと補正される)
    Sys  : 上の 3 つがそろうと自然に 3 になる
    ※ 鉄の机・PC・スマホ・ドライバーなどの近くは避ける
"""

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "main"))

from mission import config as C                    # noqa: E402  (sensor/ も import パスに入る)
from mission.hardware import set_bno_orientation   # noqa: E402
from BNO055 import BNO055                          # noqa: E402
import bno_calib                                   # noqa: E402

HINT = {
    "Sys": "他の 3 つがそろうと上がる",
    "Gyro": "机に置いて動かさない",
    "Acc": "6 方向に向けて数秒ずつ静止",
    "Mag": "空中で 8 の字を描くように回す",
}


def show(path):
    data, info = bno_calib.load_profile(path)
    if data is None:
        print(f"プロファイルなし / 読めない: {path} ({info})")
        return 1
    print(f"{path}\n  {info}")
    for k, v in bno_calib.decode(data).items():
        print(f"  {k:>10s} = {v}")
    return 0


def main():
    ap = argparse.ArgumentParser(description="BNO055 をキャリブレーションして結果を保存する")
    ap.add_argument("--out", default=str(C.BNO_CALIB_FILE), help="保存先")
    ap.add_argument("--partial", action="store_true",
                    help="Ctrl+C で止めたとき、Gyro と Mag が 3 なら保存する (Acc が 3 にならない場合)")
    ap.add_argument("--load", action="store_true", help="保存済みのプロファイルを書き戻してから始める")
    ap.add_argument("--show", action="store_true", help="保存済みのプロファイルを表示して終わる")
    args = ap.parse_args()

    if args.show:
        return show(args.out)

    bno = BNO055()
    if not bno.setUp(operation_mode=BNO055.OPERATION_MODE_NDOF):
        print("BNO055 を初期化できません。配線 (I2C) を確認してください。")
        return 1
    set_bno_orientation(bno, C.BNO_ORIENTATION)
    if args.load:
        data, info = bno_calib.load_profile(args.out)
        if data is not None:
            bno_calib.write_offsets(bno, data)
            print(f"保存済みのプロファイルを書き戻しました ({info})")

    print(__doc__.split("キャリブレーションのやり方")[1])
    print("4 項目すべて 3 になると自動で保存します。やめるときは Ctrl+C。\n")
    status = (0, 0, 0, 0)
    try:
        while True:
            status = tuple(bno.getCalibrationStatus())
            parts = []
            for name, v in zip(("Sys", "Gyro", "Acc", "Mag"), status):
                parts.append(f"{name}={v}{'✓' if v >= 3 else ' '}")
            todo = [HINT[n] for n, v in zip(("Sys", "Gyro", "Acc", "Mag"), status) if v < 3 and n != "Sys"]
            print("\r" + "  ".join(parts) + "   " + (" / ".join(todo) if todo else "そろいました") + " " * 10,
                  end="", flush=True)
            if all(v >= 3 for v in status):
                print()
                break
            time.sleep(0.5)
    except KeyboardInterrupt:
        print()
        if not (args.partial and status[1] >= 3 and status[3] >= 3):
            print(f"保存しませんでした (状態 Sys/Gyro/Acc/Mag = {status})。"
                  + ("" if args.partial else " Acc が上がらない場合は --partial を付けて実行してください。"))
            return 1

    data = bno_calib.read_offsets(bno)
    if not bno_calib.is_plausible(data):
        print("読み出した値が不正です。保存しませんでした。")
        return 1
    bno_calib.save_profile(args.out, data, status, note="tools/bno055_calibrate.py で作成")
    print(f"保存しました: {args.out}")
    show(args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
