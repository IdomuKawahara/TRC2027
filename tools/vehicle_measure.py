"""
vehicle_measure.py
==================
★モータが動きます★  機体の「旋回の速さ」「走る速さ」を実際に測り、その機体に
合う設定値の目安を表示する。

機体が変わると、同じ秒数モータを回しても回る角度・進む距離が変わる。
プログラムには「0.25 秒で約 30° 回る」「8 秒で 1 回転以上する」といった、秒数で
決めている所があるので、機体ごとに測って合わせる必要がある。

    cd ~/TRC2027
    python3 tools/vehicle_measure.py                 # 旋回だけ測る (屋内でも可・その場で左右に 2 秒ずつ回る)
    python3 tools/vehicle_measure.py --forward 5     # 加えて 5 秒前進して速度も測る (屋外・GPS が必要)

機体を床に置き、周りに人や物がないことを確認してから。止めるときは Ctrl+C。
最後に目安の値と今の値を並べて表示し、「config.py に書き込みますか？」と聞く。
yes と答えたときだけ main/mission/config.py に書き込む (それ以外は何も変えない)。
  過去機体など : config.py の一番下の if VEHICLE == "<今の機体>": の中に書く
  本番機       : config.py の上の方の本番機の値を書き換える
書き込んだあと config.py を読み込み直して確かめ、失敗したら元に戻す。
変わった所は git diff main/mission/config.py で見られる。
"""
import argparse
import datetime
import math
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "main"))

from mission import config as C      # noqa: E402
from mission import state as st      # noqa: E402

SEARCH_STEP_DEG = 30.0               # Phase5 の探索で 1 回に回したい角度 (カメラの画角 62° の約半分)


def suggest(turn_rate_dps, speed_mps=None, pulse_rate_dps=None):
    """
    測った値から、設定値の目安を作る (純粋な計算。テストできる)。
      turn_rate_dps : 片輪旋回 (turn_*_strong) を続けたときの回る速さ [deg/s]
      pulse_rate_dps: 短いパルス (約 0.25 秒) での平均の速さ [deg/s]。動き出しが遅い機体では
                      turn_rate_dps より小さい。None なら turn_rate_dps を使う
      speed_mps     : 前進の速さ [m/s] (測っていなければ None)
    戻り値: (値の dict, 注意書きのリスト)
    """
    out, notes = {}, []
    if turn_rate_dps is None or turn_rate_dps < 5.0:
        notes.append("ほとんど回れていません。モータの配線・電池・地面を確認してください")
        return out, notes
    pr = pulse_rate_dps if pulse_rate_dps and pulse_rate_dps >= 5.0 else turn_rate_dps
    turn_sec = min(0.8, max(0.10, SEARCH_STEP_DEG / pr))
    out["CAM_SEARCH_TURN_SEC"] = round(turn_sec, 2)
    out["CAM_SEARCH_PULSES_PER_LAP"] = int(math.ceil(360.0 / (pr * turn_sec))) + 2
    out["CAM_SEARCH_MAX_PULSES_PER_LAP"] = out["CAM_SEARCH_PULSES_PER_LAP"] * 2
    out["FIG8_SPIN_SEC"] = round(max(4.0, 1.3 * 360.0 / turn_rate_dps), 1)   # 1.3 回転ぶん
    out["STUCK_SPIN_GYRO_DPS"] = round(max(10.0, 0.35 * turn_rate_dps), 0)
    if 90.0 / turn_rate_dps > C.STUCK_RECOVER_TURN_TIMEOUT_SEC * 0.5:
        out["STUCK_RECOVER_TURN_TIMEOUT_SEC"] = round(4.0 * 90.0 / turn_rate_dps, 0)
        notes.append("旋回が遅いので、スタック回避の回転タイムアウトを延ばします")
    if speed_mps is not None:
        if speed_mps < 0.05:
            notes.append("前進の速さを測れませんでした (GPS が受かっていない / 進んでいない)")
        else:
            out["HEADING_COG_MIN_SPEED_MPS"] = round(max(0.10, 0.6 * speed_mps), 2)
            need = C.STUCK_NOPROGRESS_DIST_M / speed_mps * 4.0
            if need > C.STUCK_NOPROGRESS_SEC:
                out["STUCK_NOPROGRESS_SEC"] = round(need, 0)
                notes.append("走るのが遅いので、「進んでいない」と判定するまでの時間を延ばします (誤検知の防止)")
            if speed_mps < 0.15:
                notes.append("走る速さが 0.15 m/s 未満です。GPS の速度ではスタックと区別できないので、"
                             "モータ出力・電池・タイヤを見直してください")
    return out, notes


def format_lines(values: dict) -> str:
    """config.py の if VEHICLE == ...: の中にそのまま貼れる形 (4 文字下げ)。"""
    return "\n".join(f"    {k} = {v!r}" for k, v in values.items())


CONFIG_PATH = ROOT / "main" / "mission" / "config.py"
SETTINGS_END = "from . import vehicle as _vehicle"     # config.py でこれより上が本番機の値


class ConfigWriteError(Exception):
    """config.py のどこに書けばよいか分からない (書き込まない)。"""


def apply_to_config_text(text, vehicle, values, competition, stamp):
    """
    config.py の中身 (文字列) に values を書き込んだ新しい文字列を返す (純粋な関数。テストできる)。
      competition=True : 上の方の「名前 = 値」の行の値だけを書き換える (行の説明コメントは残す)
      competition=False: if VEHICLE == "<vehicle>": の中に書く。
                         同じ名前の行 (# 付きの例も含む) があれば置き換え、無ければ pass の前に足す
    """
    lines = text.split("\n")
    if competition:
        end = next((i for i, l in enumerate(lines) if l.startswith(SETTINGS_END)), None)
        if end is None:
            raise ConfigWriteError("config.py の本番機の値の終わりが見つかりません")
        for key, v in values.items():
            pat = re.compile(rf"^({re.escape(key)}\s*=\s*)([^#]*?)(\s*#.*)?$")
            hits = [i for i in range(end) if pat.match(lines[i])]
            if len(hits) != 1:
                raise ConfigWriteError(f"config.py で {key} = ... の行が {len(hits)} 個あります (1 個のはず)")
            m = pat.match(lines[hits[0]])
            lines[hits[0]] = f"{m.group(1)}{v!r}{m.group(3) or ''}"
        return "\n".join(lines)

    head = f'if VEHICLE == "{vehicle}":'
    starts = [i for i, l in enumerate(lines) if l.rstrip() == head]
    if len(starts) != 1:
        raise ConfigWriteError(f"config.py に {head} が {len(starts)} 個あります (1 個のはず)")
    start = starts[0]
    end = next((i for i in range(start + 1, len(lines)) if lines[i].strip() and not lines[i][0].isspace()),
               len(lines))
    for key, v in values.items():
        new_line = f"    {key} = {v!r}    # vehicle_measure.py で測定 ({stamp})"
        pat = re.compile(rf"^    (#\s*)?{re.escape(key)}\s*=")
        hits = [i for i in range(start + 1, end) if pat.match(lines[i])]
        active = [i for i in hits if not lines[i].lstrip().startswith("#")]
        if len(active) > 1:
            raise ConfigWriteError(f'{head} の中に {key} が {len(active)} 個あります')
        if active or hits:
            lines[(active or hits)[0]] = new_line
        else:
            passes = [i for i in range(start + 1, end) if lines[i].strip() == "pass"]
            at = passes[-1] if passes else end
            lines.insert(at, new_line)
            end += 1
    return "\n".join(lines)


def _write_and_drop_cache(path, text):
    """書き込んで、古い .pyc を消す (同じ秒・同じ大きさだと Python が古い方を使ってしまうため)。"""
    import importlib.util
    path.write_text(text, encoding="utf-8")
    try:
        Path(importlib.util.cache_from_source(str(path))).unlink()
    except FileNotFoundError:
        pass


def verify_config(path, vehicle, values):
    """書き込んだ config.py を別のプロセスで読み込み、値が入っているか確かめる。戻り値: (ok, メッセージ)"""
    code = ("import json, sys; sys.path.insert(0, %r)\n"
            "from mission import config as C\n"
            "print(json.dumps({k: getattr(C, k) for k in %r}))") % (str(Path(path).parents[1]), list(values))
    env = dict(os.environ, CANSAT_VEHICLE=vehicle)
    p = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, env=env)
    if p.returncode != 0:
        return False, (p.stderr.strip().splitlines() or ["読み込めません"])[-1]
    got = __import__("json").loads(p.stdout.strip().splitlines()[-1])
    bad = [k for k, v in values.items() if got.get(k) != v]
    return (not bad), (f"値が入っていません: {bad}" if bad else "OK")


def write_config(values, vehicle, competition, path=CONFIG_PATH):
    """config.py に書き込み、読み込み直して確かめる。だめなら元に戻す。戻り値: (ok, メッセージ)"""
    path = Path(path)
    old = path.read_text(encoding="utf-8")
    stamp = datetime.date.today().isoformat()
    try:
        new = apply_to_config_text(old, vehicle, values, competition, stamp)
    except ConfigWriteError as e:
        return False, f"書き込みませんでした: {e}"
    _write_and_drop_cache(path, new)
    ok, msg = verify_config(path, vehicle, values)
    if not ok:
        _write_and_drop_cache(path, old)
        return False, f"書き込んだ config.py が読み込めなかったので元に戻しました: {msg}"
    return True, "書き込みました"


def ask_and_write(values, vehicle, competition, ask=input, path=CONFIG_PATH):
    """今の値と目安を並べて見せ、yes のときだけ書き込む。戻り値: 書き込んだら True"""
    if not values:
        print("書き込む値はありません")
        return False
    where = ("config.py の上の方の【本番機の値】を書き換えます" if competition
             else f'config.py の一番下の if VEHICLE == "{vehicle}": の中に書きます')
    print(f"\n── 書き込む値 ({where}) ──")
    for k, v in values.items():
        print(f"  {k:28s} 今 {getattr(C, k)!r:>8}  →  {v!r}")
    try:
        ans = ask("この値を config.py に書き込みますか？ (yes/no): ").strip().lower()
    except EOFError:
        ans = ""
    if ans != "yes":
        print("書き込みませんでした (config.py は変えていません)")
        return False
    ok, msg = write_config(values, vehicle, competition, path)
    print(msg)
    if ok:
        print("  変わった所: git diff main/mission/config.py   元に戻す: git checkout main/mission/config.py")
    return ok


# ── ここから下は実機 (モータ・BNO055・GPS) を使う ─────────────────────

def measure_turn(bno, motor, direction, sec):
    """direction ("L"/"R") へ sec 秒回り、ジャイロの積分で回った角度 [deg] を返す。"""
    (motor.turn_left_strong if direction == "L" else motor.turn_right_strong)()
    t0 = prev = time.time()
    deg = 0.0
    try:
        while time.time() - t0 < sec:
            time.sleep(0.02)
            now = time.time()
            deg += abs(bno.getGyro()[2]) * (now - prev)
            prev = now
    finally:
        motor.stop()
    time.sleep(1.0)
    return deg


def main():
    ap = argparse.ArgumentParser(description="★モータが動く★ 機体の旋回・前進の速さを測る")
    ap.add_argument("--turn-sec", type=float, default=2.0, help="旋回を測る秒数 (片側)")
    ap.add_argument("--forward", type=float, default=0.0, help="前進して速度を測る秒数 (0 = 測らない。屋外・GPS 必要)")
    ap.add_argument("--yes", action="store_true", help="モータを動かす前の確認を省略 (config.py への書き込みは必ず聞く)")
    args = ap.parse_args()

    print(f"機体: {C.VEHICLE_NAME}")
    print("★ モータが動きます。機体を床に置き、周りに人や物がないことを確認してください ★")
    if not args.yes:
        try:
            if input("実行するには yes と入力: ").strip().lower() != "yes":
                print("中止しました")
                return 1
        except EOFError:
            return 1

    from gpiozero import Device
    from gpiozero.pins.lgpio import LGPIOFactory
    from BNO055 import BNO055
    from micropyGPS import MicropyGPS
    from mission.hardware import MotorController, set_bno_orientation, gps_thread_func
    Device.pin_factory = LGPIOFactory()
    bno = BNO055()
    if not bno.setUp(operation_mode=BNO055.OPERATION_MODE_NDOF):
        print("BNO055 を初期化できません")
        return 1
    set_bno_orientation(bno, C.BNO_ORIENTATION)
    motor = MotorController()
    speed = None
    try:
        time.sleep(1.0)
        rates, pulse_rates = [], []
        for d in ("L", "R"):
            deg = measure_turn(bno, motor, d, args.turn_sec)
            rates.append(deg / args.turn_sec)
            print(f"  {'左' if d == 'L' else '右'}旋回 {args.turn_sec:.1f} s: {deg:.0f}°  → {rates[-1]:.0f} deg/s")
        for d in ("L", "R"):
            deg = measure_turn(bno, motor, d, C.CAM_SEARCH_TURN_SEC)
            pulse_rates.append(deg / C.CAM_SEARCH_TURN_SEC)
            print(f"  {'左' if d == 'L' else '右'}の短いパルス {C.CAM_SEARCH_TURN_SEC:.2f} s: {deg:.0f}°")
        if max(rates) > 0 and min(rates) / max(rates) < 0.6:
            print("  注意: 左右で回る速さが大きく違います (片方のモータ・タイヤ・配線を確認)")
        if args.forward > 0:
            gps_obj = MicropyGPS(local_offset=9, location_formatting='ddm')
            threading.Thread(target=gps_thread_func, args=(gps_obj,), daemon=True).start()
            print("  GPS の受信を待っています (最大 60 秒)...")
            t0 = time.time()
            while not st.g_gps_valid and time.time() - t0 < 60:
                time.sleep(1.0)
            motor.forward()
            samples = []
            t0 = time.time()
            while time.time() - t0 < args.forward:
                time.sleep(0.5)
                if st.g_gps_valid and time.time() - t0 > 2.0:      # 動き出しの 2 秒は除く
                    samples.append(st.g_gps_speed * C.KNOTS_TO_MPS)
            motor.stop()
            speed = sum(samples) / len(samples) if samples else 0.0
            print(f"  前進 {args.forward:.0f} s: 平均 {speed:.2f} m/s (GPS)")
    except KeyboardInterrupt:
        print("\n中断しました")
        return 1
    finally:
        motor.stop()
        motor.close()

    values, notes = suggest(min(rates), speed, min(pulse_rates))
    for n in notes:
        print(f"注意: {n}")
    # 書き込みは必ず聞く (--yes でも省略しない)
    ask_and_write(values, C.VEHICLE_NAME, C.IS_COMPETITION_VEHICLE)
    return 0


if __name__ == "__main__":
    sys.exit(main())
