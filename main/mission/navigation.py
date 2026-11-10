"""
navigation.py
=============
航法計算: Haversine 距離・目標方位・地磁気からの方位・方位差。
(main6.py から機械的に分割。動作は変更していない)
元の位置: main6.py 1032〜1095 行, 1192〜1214 行
"""

import math

from . import config as C
from . import state as st

# ===========================================================================
# 航法計算 (★変更: Haversine距離 + 大圆初期方位角の標準公式に変更)
# ===========================================================================
# ★変更: 従来は「経度差×cos(lat)×地球半径」「緯度差×地球半径」による
#        平面近似 (等長円筒図法, 十数m規模ではこれでも大きな誤差は出ないが
#        厳密ではない) で距離・方位を求めていた。実運用ログを分析した結果、
#        この近似式自体の数値は手計算とも一致しており大きなバグは無かったが、
#        「精度をさらに上げてほしい」との要望と、GPS誘導の信頼性を少しでも
#        高める目的で、球面三角法に基づく標準的な Haversine 距離公式・
#        大圆(great-circle)初期方位角公式に置き換えた。緯度によって
#        経度1度あたりの実距離が変わる影響も正確に扱えるため、より高緯度や
#        長距離でも精度が保たれる。

def calc_distance(lat, lng) -> float:
    """
    Haversine の公式による現在地から目標地点までの球面距離 [m]。
    平面近似より厳密に地球の曲率を考慮するため、緯度・経度差が
    大きい場合でも誤差が蓄積しにくい。
    """
    lat1 = math.radians(lat)
    lat2 = math.radians(C.TARGET_LAT)
    dlat = math.radians(C.TARGET_LAT - lat)
    dlng = math.radians(C.TARGET_LNG - lng)
    a = (math.sin(dlat / 2.0) ** 2
         + math.cos(lat1) * math.cos(lat2) * math.sin(dlng / 2.0) ** 2)
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return C.EARTH_RADIUS * c

def calc_target_bearing(lat, lng) -> float:
    """
    大圆 (great-circle) 初期方位角の公式による、現在地から目標地点への
    真方位 [度, 0〜360, 北を0として時計回り]。
    """
    lat1 = math.radians(lat)
    lat2 = math.radians(C.TARGET_LAT)
    dlng = math.radians(C.TARGET_LNG - lng)
    x = math.sin(dlng) * math.cos(lat2)
    y = (math.cos(lat1) * math.sin(lat2)
         - math.sin(lat1) * math.cos(lat2) * math.cos(dlng))
    theta = math.atan2(x, y)
    return (math.degrees(theta) + 360.0) % 360.0

def calc_azimuth(mag: list) -> float:
    """
    生の地磁気ベクトルから磁北基準の方位角 [度, 北=0, 東=90, 南=180, 西=270,
    時計回り] を計算する。

    ★変更: 反転系のフラグ (AZIMUTH_MIRROR_FIX) を廃止し、test_GPSrun.py と
    全く同じ「素の」計算式に統一した。フラグを増やして条件分岐で
    細かく補正しようとすると、フラグ同士の組み合わせが増えて逆に
    機体の動きが把握しづらくなる (今回発生した混乱の原因)。取り付け
    向きの補正は BNO_ORIENTATION (Axis Remap, Phase0で1回設定) だけに
    一本化し、方位計算式自体は常に固定・単純に保つ。
    """
    azimuth = 90.0 - math.degrees(math.atan2(mag[1], mag[0]))
    azimuth *= -1
    azimuth += C.MAG_DECLINATION
    return azimuth % 360.0

def calc_direction_diff(azimuth: float, bearing: float) -> float:
    diff = (azimuth - bearing) % 360.0
    if diff > 180.0:
        diff -= 360.0
    return diff


def calc_azimuth_with_front(mag: list) -> float:
    """
    ★変更: 前方オフセット (g_front_offset_deg) の加算は、BNO055 側の
    Axis Remap (set_bno_orientation, Phase0 で1回設定) に一本化した
    ため廃止した。この関数は calc_azimuth() へ渡す前に地磁気ベクトルへ
    EMA (指数移動平均) フィルタを適用する処理のみを担う。

    実機ログ解析により、モータへ高デューティで通電している間、地磁気
    センサの読み取り値が100ms間隔で数十〜100度以上もジャンプするほどの
    ノイズ (モータ電流による磁気干渉と推定) が発生し、これが原因で
    機体方位が暴れて GPS 誘導が発散 (目標から遠ざかる) することが
    確認された。そのため、まず地磁気ベクトルへ EMA フィルタ (g_mag_ema,
    MAG_EMA_ALPHA) を適用してノイズを平滑化した上で calc_azimuth() へ渡す。
    """
    if st.g_mag_ema is None:
        st.g_mag_ema = [float(mag[0]), float(mag[1]), float(mag[2])]
    else:
        st.g_mag_ema[0] = C.MAG_EMA_ALPHA * mag[0] + (1.0 - C.MAG_EMA_ALPHA) * st.g_mag_ema[0]
        st.g_mag_ema[1] = C.MAG_EMA_ALPHA * mag[1] + (1.0 - C.MAG_EMA_ALPHA) * st.g_mag_ema[1]
        st.g_mag_ema[2] = C.MAG_EMA_ALPHA * mag[2] + (1.0 - C.MAG_EMA_ALPHA) * st.g_mag_ema[2]

    return calc_azimuth(st.g_mag_ema)
