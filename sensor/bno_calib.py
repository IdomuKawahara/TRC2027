"""
bno_calib.py
============
BNO055 のキャリブレーション結果 (オフセット・半径 22 バイト) を読み書きし、
ファイルに保存/読込する。

TRC2027/sensor/bno_calib.py

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
なぜ必要か:
    BNO055 は電源を入れるたび (setUp のソフトウェアリセットのたび) に
    キャリブレーションが消える。地上機は姿勢を変えられないので、本番の
    Phase3 では加速度のキャリブレーションが原理的にできず、地磁気も
    モータの近くでスピンするだけでは十分にならないことがある。
    そこで事前に手で持って 8 の字・6 方向に向けてしっかりキャリブレーション
    した結果を保存しておき (test/bno055_calibrate.py)、本番の起動時に
    書き戻す。BNO055 データシート 3.11.4「Reuse of Calibration Profile」の方法。

レジスタ (ページ 0):
    0x55〜0x5A ACC_OFFSET X/Y/Z  0x5B〜0x60 MAG_OFFSET X/Y/Z
    0x61〜0x66 GYR_OFFSET X/Y/Z  0x67〜0x68 ACC_RADIUS  0x69〜0x6A MAG_RADIUS
    読み書きは CONFIG モードで行い、その後 NDOF モードに戻す。
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import datetime
import json
import time
from pathlib import Path

OFFSET_START_REG = 0x55
OFFSET_LEN = 22
OPR_MODE_REG = 0x3D
MODE_CONFIG = 0x00
MODE_NDOF = 0x0C

FIELDS = ("acc_x", "acc_y", "acc_z", "mag_x", "mag_y", "mag_z",
          "gyr_x", "gyr_y", "gyr_z", "acc_radius", "mag_radius")


def _set_mode(bno, mode):
    if hasattr(bno, "_set_mode"):
        bno._set_mode(mode)
    else:
        bno._write_byte(OPR_MODE_REG, mode)
    time.sleep(0.03)            # モード切り替えには 19ms (→CONFIG) / 7ms (→NDOF) 以上必要


def _read_block(bno, reg, n):
    i2c = getattr(bno, "i2c", None)
    if i2c is not None and hasattr(i2c, "read_i2c_block_data"):
        return bytes(i2c.read_i2c_block_data(bno.addr, reg, n))
    return bytes(bno._read_byte(reg + i) for i in range(n))


def read_offsets(bno, restore_mode=MODE_NDOF) -> bytes:
    """今のキャリブレーション結果 (22 バイト) を読む。読んだ後 restore_mode に戻す。"""
    _set_mode(bno, MODE_CONFIG)
    try:
        return _read_block(bno, OFFSET_START_REG, OFFSET_LEN)
    finally:
        _set_mode(bno, restore_mode)


def write_offsets(bno, data: bytes, restore_mode=MODE_NDOF):
    """キャリブレーション結果 (22 バイト) を書き込む。書いた後 restore_mode に戻す。"""
    if len(data) != OFFSET_LEN:
        raise ValueError(f"offsets must be {OFFSET_LEN} bytes, got {len(data)}")
    _set_mode(bno, MODE_CONFIG)
    try:
        for i, b in enumerate(data):
            bno._write_byte(OFFSET_START_REG + i, int(b) & 0xFF)
    finally:
        _set_mode(bno, restore_mode)


def decode(data: bytes) -> dict:
    """22 バイトを符号付き 16bit の各値に分解する (表示・妥当性チェック用)。"""
    vals = []
    for i in range(0, OFFSET_LEN, 2):
        v = data[i] | (data[i + 1] << 8)
        vals.append(v - 0x10000 if v & 0x8000 else v)
    return dict(zip(FIELDS, vals))


def is_plausible(data: bytes) -> bool:
    """明らかにおかしいデータ (全部 0 / 全部 0xFF / 半径が 0 以下) を弾く。"""
    if data is None or len(data) != OFFSET_LEN:
        return False
    if all(b == 0 for b in data) or all(b == 0xFF for b in data):
        return False
    d = decode(data)
    return d["acc_radius"] > 0 and d["mag_radius"] > 0


def save_profile(path, data: bytes, calib_status=None, note: str = ""):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    obj = {
        "offsets": list(data),
        "decoded": decode(data),
        "calib_status_sys_gyro_acc_mag": list(calib_status) if calib_status else None,
        "saved_at": datetime.datetime.now().isoformat(timespec="seconds"),
        "note": note,
    }
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(obj, indent=2, ensure_ascii=False), encoding="utf-8")
    tmp.replace(path)          # 書き込み途中で電源が落ちても壊れたファイルを残さない


def load_profile(path):
    """保存したプロファイルを読む。無い/壊れている/おかしい場合は (None, 理由)。"""
    path = Path(path)
    if not path.exists():
        return None, "ファイルなし"
    try:
        obj = json.loads(path.read_text(encoding="utf-8"))
        data = bytes(int(x) & 0xFF for x in obj["offsets"])
    except Exception as e:
        return None, f"読み込み失敗: {e}"
    if not is_plausible(data):
        return None, "内容が不正"
    return data, f"保存日時 {obj.get('saved_at', '?')} 状態 {obj.get('calib_status_sys_gyro_acc_mag')}"
