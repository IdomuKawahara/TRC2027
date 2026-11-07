"""
heading_fusion.py
=================
地磁気の方位と GPS の進行方位 (COG: Course Over Ground) を組み合わせて、
機体の方位を求める。

TRC2027/sensor/heading_fusion.py

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
背景:
    実機ログで、モータに電流が流れている間は地磁気の方位が数十〜100° 跳ねる
    ことが分かっている。また、モータ・電池の磁気で地磁気の方位には一定の
    ずれ (オフセット) も乗る。一方 GPS の COG は磁気の影響を受けないが、
    ・ある程度の速さで動いていないと値がでたらめ
    ・曲がっている最中は「今の向き」ではなく「少し前からの移動方向」になる
    ・1 秒に 1 回しか更新されない
    という弱点がある。

方式:
    1. まっすぐ前進している (前進指令 & ジャイロの旋回速度が小さい状態が
       straight_min_sec 続いている) & GPS 速度が十分 & COG が新しい
       → COG をそのまま機体の方位として使う (source="COG")
       同時に「COG − 地磁気」のずれを学習する (円周上の指数移動平均)
    2. それ以外 (旋回中・低速・停止中)
       → 地磁気 + 学習したずれ を使う (source="MAG+OFS")
       ずれをまだ学習していなければ地磁気のみ (source="MAG")
    学習中に 1 回だけ大きく外れた値 (地磁気の瞬間的な跳ね) は無視し、
    外れた値が続いたら「ずれ自体が変わった」とみなして学習し直す。
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import math


def wrap180(deg: float) -> float:
    """角度を -180〜+180 に正規化する。"""
    return (deg + 180.0) % 360.0 - 180.0


def wrap360(deg: float) -> float:
    return deg % 360.0


class HeadingEstimator:
    def __init__(self,
                 cog_min_speed_mps=0.30,
                 straight_min_sec=1.5,
                 gyro_max_dps=8.0,
                 cog_max_age_sec=1.5,
                 offset_alpha=0.25,
                 min_offset_samples=3,
                 outlier_deg=45.0,
                 outlier_reset_count=5):
        self.cog_min_speed_mps = cog_min_speed_mps
        self.straight_min_sec = straight_min_sec
        self.gyro_max_dps = gyro_max_dps
        self.cog_max_age_sec = cog_max_age_sec
        self.offset_alpha = offset_alpha
        self.min_offset_samples = min_offset_samples
        self.outlier_deg = outlier_deg
        self.outlier_reset_count = outlier_reset_count

        self.straight_since = None
        self.offset_deg = 0.0          # 学習した「COG − 地磁気」 [deg]
        self.n_samples = 0             # 学習に使ったサンプル数
        self.n_outliers = 0            # 連続した外れ値の数
        self._last_cog_fix = None      # 最後に学習に使った GPS fix の時刻 (同じ fix で二重に学習しない)
        self.source = "MAG"
        self.heading = None

    @property
    def offset_ready(self) -> bool:
        return self.n_samples >= self.min_offset_samples

    def _learn(self, err: float):
        """COG − 地磁気 のずれ err を学習する (円周上の EMA + 外れ値の除外)。"""
        if self.n_samples == 0:
            self.offset_deg = wrap180(err)
            self.n_samples = 1
            return
        if self.offset_ready and abs(wrap180(err - self.offset_deg)) > self.outlier_deg:
            self.n_outliers += 1
            if self.n_outliers < self.outlier_reset_count:
                return                       # 地磁気の瞬間的な跳ね → 無視
            # 外れ値が続いた → ずれそのものが変わった。学習し直す
            self.offset_deg = wrap180(err)
            self.n_samples = 1
            self.n_outliers = 0
            return
        self.n_outliers = 0
        a = self.offset_alpha
        s = (1 - a) * math.sin(math.radians(self.offset_deg)) + a * math.sin(math.radians(err))
        c = (1 - a) * math.cos(math.radians(self.offset_deg)) + a * math.cos(math.radians(err))
        self.offset_deg = math.degrees(math.atan2(s, c))
        self.n_samples += 1

    def update(self, now: float, mag_heading: float, *,
               gps_valid: bool, gps_speed_mps: float, gps_course: float,
               gps_fix_time: float, forward: bool, gyro_z_dps: float) -> float:
        """
        1 ループごとに呼ぶ。戻り値: 機体の方位 [deg, 0〜360, 北=0 時計回り]
          mag_heading : 地磁気から求めた方位 [deg]
          forward     : 直前のループで「前進」を指令していたか
          gyro_z_dps  : ヨー方向の角速度 [deg/s] (符号は問わない)
          gps_fix_time: GPS の値が最後に更新された時刻 (now と同じ時計)
        """
        straight = forward and abs(gyro_z_dps) <= self.gyro_max_dps
        if straight:
            if self.straight_since is None:
                self.straight_since = now
        else:
            self.straight_since = None

        cog_ok = (straight
                  and now - self.straight_since >= self.straight_min_sec
                  and gps_valid
                  and gps_speed_mps >= self.cog_min_speed_mps
                  and gps_fix_time is not None
                  and 0 <= now - gps_fix_time <= self.cog_max_age_sec)

        if cog_ok:
            if gps_fix_time != self._last_cog_fix:
                self._last_cog_fix = gps_fix_time
                self._learn(wrap180(gps_course - mag_heading))
            self.source = "COG"
            self.heading = wrap360(gps_course)
        elif self.offset_ready:
            self.source = "MAG+OFS"
            self.heading = wrap360(mag_heading + self.offset_deg)
        else:
            self.source = "MAG"
            self.heading = wrap360(mag_heading)
        return self.heading

    def summary(self) -> str:
        ofs = f"{self.offset_deg:+.1f}°(n={self.n_samples})" if self.n_samples else "未学習"
        h = f"{self.heading:.1f}°" if self.heading is not None else "---"
        return f"方位={h} src={self.source} 地磁気補正={ofs}"
