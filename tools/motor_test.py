"""
motor_test.py
=============
★モータが動きます★  キーボードでモータを動かし、前後・左右の向きを確かめる。

本番のプログラム (main.py) と同じ部品 (mission.hardware.MotorController) と、
main/mission/config.py のピン番号を使う。だから、
  - ここで「前進」で前に進めば、main.py でも前に進む
  - config.py の一番上の VEHICLE を "old" にすれば、過去機体のピン番号で動く
(以前はこのファイルにピン番号を直接書いていて、本番のプログラムと前後が逆になっていた)

    cd ~/TRC2027
    python3 tools/motor_test.py

キー:
    W : 前進              S : 後退
    A : 左旋回 (右モータだけ前進)    D : 右旋回 (左モータだけ前進)
    Z : その場で左回転    C : その場で右回転   (左右のモータを逆に回す)
    スペース : 停止        Q : 終了 (Ctrl+C でも止まる)

向きがおかしいとき (config.py のピン番号を直す。過去機体なら if VEHICLE == "old": の中に書く):
    W で後ろに進む              → 両方のモータで IN1 と IN2 を入れ替える
    A で右モータが後ろに回る    → PIN_AIN1 と PIN_AIN2 を入れ替える
    D で左モータが後ろに回る    → PIN_BIN1 と PIN_BIN2 を入れ替える
    A で左のタイヤが回る (左右が逆) → A の組 (PWMA, AIN1, AIN2) と B の組 (PWMB, BIN1, BIN2) を入れ替える
"""
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "main"))

from mission import config as C      # noqa: E402

# キー → (MotorController のメソッド名, 表示, 正しいときの動き)
KEYMAP = {
    "w": ("forward",           "前進",             "両方のタイヤが前に回り、まっすぐ前に進む"),
    "s": ("backward",          "後退",             "両方のタイヤが後ろに回り、後ろに進む"),
    "a": ("turn_left_strong",  "左旋回",           "右のタイヤだけ前に回り、左に曲がる"),
    "d": ("turn_right_strong", "右旋回",           "左のタイヤだけ前に回り、右に曲がる"),
    "z": ("pivot_left",        "その場で左回転",   "右は前・左は後ろに回り、その場で左に回る"),
    "c": ("pivot_right",       "その場で右回転",   "右は後ろ・左は前に回り、その場で右に回る"),
    " ": ("stop",              "停止",             ""),
}


def pin_summary():
    """今使うピン番号の表示 (config.py の値)。"""
    return (f"右モータ (A): PWMA={C.PIN_PWMA}  AIN1={C.PIN_AIN1}  AIN2={C.PIN_AIN2}\n"
            f"左モータ (B): PWMB={C.PIN_PWMB}  BIN1={C.PIN_BIN1}  BIN2={C.PIN_BIN2}\n"
            f"STBY={C.PIN_STBY}")


def run(motor, get_key, out=print):
    """キーを読んでモータを動かすループ (テストでは偽のモータとキーを渡す)。"""
    try:
        while True:
            key = get_key()
            if key in ("q", "\x03"):          # q / Ctrl+C (raw モードでは文字で来る)
                break
            if key in KEYMAP:
                method, label, expect = KEYMAP[key]
                getattr(motor, method)()
                out(f"{label}" + (f"  … 正しければ: {expect}" if expect else ""))
            time.sleep(0.05)
    finally:
        motor.stop()


def _get_key():
    """キーを 1 文字読む (Enter 不要)。端末でなければ 1 行読む。"""
    try:
        import termios
        import tty
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch.lower()
    except Exception:
        line = sys.stdin.readline()
        if not line:
            return "q"
        return (line.rstrip("\n")[:1] or " ").lower()


def main():
    from gpiozero import Device
    from gpiozero.pins.lgpio import LGPIOFactory
    from mission import hardware

    # MotorController のログ (ソフトスタートの経過など) は多いので、警告だけ表示する
    hardware.log = lambda msg, level="INFO": print(f"[{level}] {msg}\r") if level != "INFO" else None

    print(f"機体: {C.VEHICLE_NAME}  (config.py の VEHICLE)")
    print(pin_summary())
    print("★ モータが動きます。機体を浮かせるか床に置き、周りに人や物がないことを確認してください ★")
    print("W/S: 前進/後退  A/D: 左/右旋回 (片輪)  Z/C: その場で左/右回転  スペース: 停止  Q: 終了")

    Device.pin_factory = LGPIOFactory()
    motor = hardware.MotorController()
    try:
        run(motor, _get_key, out=lambda s: print(s + "\r"))
    except KeyboardInterrupt:
        print("\n中断しました")
    finally:
        motor.stop()
        motor.close()
    print("停止しました")
    return 0


if __name__ == "__main__":
    sys.exit(main())
