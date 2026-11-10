"""
config.py
=========
設定値 (main6.py の「設定値」セクションをそのまま移動。値は変更していない)。
実地に合わせて変更するのはこのファイルだけ。

元の位置: main6.py 248〜249 行 (MAG_EMA_ALPHA), 255〜264 行 (PHASE_NAMES),
          311〜654 行 (設定値セクション), 1097〜1130 行 (BNO_ORIENTATION)
"""

import datetime as _dt
from pathlib import Path

# --- パス ---------------------------------------------------------------------
_MISSION_DIR = Path(__file__).resolve().parent      # TRC2027/main/mission/
_MAIN_DIR  = _MISSION_DIR.parent                     # TRC2027/main/
_ROOT_DIR  = _MAIN_DIR.parent                        # TRC2027/
_SENSOR_DIR = _ROOT_DIR / "sensor"

# ===========================================================================
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#   設定値  ← ここを実地に合わせて変更
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# ===========================================================================

# --- 目標座標 ---------------------------------------------------------------
TARGET_LAT =  38.26052      # 目標緯度  [度]
TARGET_LNG = 140.8544151    # 目標経度  [度]
# ★変更: この距離以内になったら Phase4 (GPS誘導走行) を終了し、
#        Phase5 (超音波センサによる最終接近) へ移行する。
#        最終的な「0距離」への接近は GPS ではなく Phase5 の超音波センサで行う。
PHASE4_TO_5_RADIUS = 2.0    # [m]

# --- 地磁気偏角 (仙台付近) --------------------------------------------------
# ★改善: 会場ごとのプリセットから自動で決める。
#   MAG_SITE に "sendai" / "noshiro" / "tanegashima" を指定するか、"auto" なら
#   TARGET_LAT/LNG に一番近い会場を選ぶ。値は国土地理院「都道府県庁所在地の偏角値」
#   (2020.0 年値, 西偏を負) を元に、5 年あたりの変化量で実行日まで外挿する。
#   能代・種子島は県庁所在地ではないので近隣 (秋田/青森, 鹿児島/那覇) から補間した概算。
#   走行中は GPS の進行方位との差を自動で学習する (heading_fusion) ので、
#   ここの 0.3° 程度の誤差は結果に影響しない。
#   手動で決めたいときは MAG_DECLINATION_OVERRIDE に数値を入れる (プリセットより優先)。
MAG_SITE = "auto"
MAG_SITE_PRESETS = {
    #                 名前      緯度     経度     2020.0 偏角[°]  変化[°/年]  (出典: 国土地理院 資料-3)
    "sendai":      {"name": "仙台",   "lat": 38.27, "lng": 140.87, "decl_2020": -8.6, "rate_per_year": -0.06},
    "noshiro":     {"name": "能代",   "lat": 40.17, "lng": 140.02, "decl_2020": -8.8, "rate_per_year": -0.06},
    "tanegashima": {"name": "種子島", "lat": 30.40, "lng": 130.97, "decl_2020": -6.8, "rate_per_year": -0.08},
}
MAG_DECLINATION_OVERRIDE = None   # [度] 数値を入れるとプリセットを使わずこの値を使う (西偏 → 負値)


def _resolve_mag_declination(site, target_lat, target_lng, override, today=None):
    """プリセット (または override) から、今日の日付での偏角 [度] を求める。
    戻り値: (使った会場のキー or "override", 偏角)"""
    if override is not None:
        return "override", float(override)
    if site == "auto":
        def dist2(p):
            return ((p["lat"] - target_lat) ** 2
                    + ((p["lng"] - target_lng) * 0.8) ** 2)   # 緯度 38° 付近の経度の縮み (cos) を概算
        site = min(MAG_SITE_PRESETS, key=lambda k: dist2(MAG_SITE_PRESETS[k]))
    if site not in MAG_SITE_PRESETS:
        raise ValueError(f"MAG_SITE={site!r} は未定義。{list(MAG_SITE_PRESETS)} か 'auto' を指定")
    p = MAG_SITE_PRESETS[site]
    today = today or _dt.date.today()
    years = (today - _dt.date(2020, 1, 1)).days / 365.25
    return site, round(p["decl_2020"] + p["rate_per_year"] * years, 2)


MAG_SITE_RESOLVED, MAG_DECLINATION = _resolve_mag_declination(
    MAG_SITE, TARGET_LAT, TARGET_LNG, MAG_DECLINATION_OVERRIDE)   # [度]  西偏 → 負値

# --- Phase 1: 落下検知 -------------------------------------------------------
FALL_THRESHOLD       = 3.0  # [m/s²]  合成加速度ノルムがこれ以下 → 落下中
FALL_COUNT_THRESHOLD = 8    # 連続カウント数
FALL_TIMEOUT_SEC     = 7 * 60  # [s]  7 分でタイムアウト

# --- Phase 2: 初期後退 + 停止 + 前進 ------------------------------------------
# ★変更: 従来は PHASE2_BACK_SEC (固定30秒) で後退していたが、パラシュート/
#        サブキャリアからの分離タイミングは機体の落下姿勢や地面との接触状況で
#        毎回変わるため、固定秒数では「分離できていないのに後退をやめる」
#        「分離済みなのに無駄に後退を続ける」の両方のリスクがあった。
#        そこで前方(進行方向とは反対、後退中は「背面」ではなく後退前に
#        機体前方に設置されている超音波センサがサブキャリアの方を向いている
#        前提)の超音波センサで距離を監視し、サブキャリアから機体が分離した
#        瞬間に生じる距離の急増 (PHASE2_SONAR_JUMP_THRESH_M 以上) を検知して
#        その場で後退を打ち切る方式に変更した。
PHASE2_BACK_TIMEOUT_SEC     = 30.0  # [s]  安全タイムアウト (分離検知できない場合の後退上限)
PHASE2_BACK_MIN_SEC         =  3.0  # [s]  後退開始直後はセンサ値が乱れやすいため、
                                     #      この秒数が経過するまでは分離判定を行わない
PHASE2_SONAR_JUMP_THRESH_M  = 0.5   # [m]  直前の有効値からこれ以上距離が急増したら
                                     #      「サブキャリアから分離した」と判定する
PHASE2_SONAR_LOST_TIMEOUT_SEC = 5.0 # [s]  超音波が有効値を返さない状態がこれだけ続いたら
                                     #      警告ログを出す (判定自体は継続)
PHASE2_LOOP_DT       =  0.1  # [s]  後退中の監視ループ周期
PHASE2_PAUSE_SEC     =  5.0  # [s]  後退後の停止秒数 (後退→前進の切り替え待ち)
PHASE2_FWD_AFTER_SEC = 10.0  # [s]  後退・停止後に行う前進秒数

# --- Phase 3: キャリブレーション ----------------------------------------------
# ★変更: 以前は Sys/Gyro/Acc/Mag すべてに同じ CALIB_MIN_LEVEL(=2) を要求していたが、
#        実地では Acc・Gyro のレベルが上がりにくく、それが原因でタイムアウトしていた。
#        地上機は姿勢を大きく変えられず Acc の複数姿勢キャリブレーションが困難なため、
#        軸ごとに現実的な目標レベルを個別に設定できるようにした。
#        (Mag は自律スピンで比較的到達しやすいので従来通り高めのままにしている)
#
# ★重要な追加変更: 実機ログを分析した結果、CALIB_MIN_LEVEL_ACC=1 でも
#        180秒のタイムアウトまで一度も達成できず、Acc=0 のまま停滞し続けて
#        いたことが判明した。BNO055 の加速度センサキャリブレーションは、
#        内部アルゴリズム上「機体を6方向以上の異なる姿勢 (上下反転を含む)
#        で数秒間静止させる」ことを要求する。しかし本機は地上を走行する
#        平面移動ロボットであり、物理的に姿勢を変える手段が無いため、
#        Acc キャリブレーションレベルが 0 から絶対に上がらない
#        (=どれだけ待っても目標に到達できない) という構造的な限界がある。
#        このため CALIB_MIN_LEVEL_ACC は要求しない (=0) ものとし、
#        Sys/Gyro/Mag のみで完了判定を行うように変更した。
#        なお、Acc が未キャリブレーションであっても、Phase3 完了後に
#        測定する静止基準水平加速度 (baseline_horiz_acc) が実行時の
#        オフセットを実質的に補正するため、落下検知・スタック検知への
#        影響は限定的である。
CALIB_MIN_LEVEL_SYS  = 1    # 0〜3  システム全体 (他軸がある程度揃えば自然に上がることが多い)
CALIB_MIN_LEVEL_GYRO = 1    # 0〜3  ★緩和: 静止だけで上がるはずだが、モータ振動の余韻等で
                            #        上がりにくい場合があるため要求を下げた
CALIB_MIN_LEVEL_ACC  = 0    # 0〜3  ★変更: 地上機は複数姿勢を取れず物理的に到達不可能なため、
                            #        Acc キャリブレーションは要求しない (常に条件を満たす)
CALIB_MIN_LEVEL_MAG  = 2    # 0〜3  スピンで比較的到達しやすいので維持
CALIB_TIMEOUT_SEC = 180.0   # [s]  キャリブレーション待機タイムアウト (地上機なので少し長め)
# スタック検知用：水平加速度の「動いているとき」の基準を測る秒数
CALIB_ACC_MEASURE_SEC = 3.0
# 詳細レポートをログに出す周期 (コンソールの1行表示とは別に、まとめて状況報告する)
CALIB_REPORT_INTERVAL_SEC = 5.0
# あるセンサ軸がこの秒数レベル改善しなければ「停滞」警告を出す
CALIB_STALL_WARN_SEC = 20.0
# 静止フェーズ1回あたりの秒数 (Acc/Gyro向け)。
# スピン(Mag向け)と交互に繰り返すことで、特定軸だけが
# 原因で全体が進まなくなるのを防ぐ。
CALIB_STILL_SEC = 6.0
# ★追加: スピン直後にモータを止めても、慣性で機体がすぐには完全静止しない。
#        「静止」区間の頭からその余韻の分だけ経過を捨てる(セトリング)ことで、
#        実際には揺れている間を静止時間としてカウントしてしまい
#        Gyro/Acc のキャリブレーションが進まなくなる問題を防ぐ。
CALIB_SETTLE_SEC = 1.5

# --- Phase 3: 自律回転動作 (地磁気キャリブレーション促進) --------------------
# 平面移動しかできない地上機では、腕で振る「8の字」の代わりに
# その場でのスピン (弱旋回の弧ではなく強旋回による回転) を左右交互に行うことで
# 全方位の地磁気データを収集し、Magキャリブレーションを促進する。
FIG8_ENABLE   = True   # True: モータで自律的に回転動作を行う
FIG8_SPIN_SEC = 8.0    # [s]  片方向にスピンし続ける秒数。機体が最低1回転できる値に調整すること
                       #      (回転が速すぎる/遅すぎる場合はここを実地に合わせて調整)

# --- Phase 3: GPS Fix 安定化・平均化 (★追加) --------------------------------
GPS_MIN_SATS          = 4     # [個]  この衛星捕捉数以上で Fix を信頼する
GPS_CALIB_TIMEOUT_SEC = 60.0  # [s]   GPS Fix 安定化待機の最大秒数
GPS_CALIB_SAMPLES     = 10    # [個]  平均化に使う最大サンプル数
GPS_CALIB_SAMPLE_SEC  = 8.0   # [s]   サンプル収集を行う秒数

# --- LED: フェーズ切り替え通知 -----------------------------------------------
LED_BLINK_SEC      = 10.0   # [s]  フェーズ切り替え時の点滅継続時間
LED_BLINK_INTERVAL = 0.3    # [s]  点滅の ON/OFF 半周期

# --- Phase 4: 誘導走行 -------------------------------------------------------
LOOP_DT      = 0.1          # [s]  制御ループ周期
TIMEOUT_SEC  = 15 * 60      # [s]  走行タイムアウト (15 分)

# 方向制御閾値
ANGLE_DEADBAND    = 10.0    # [度]  この範囲内なら前進
ANGLE_TURN_STRONG = 45.0    # [度]  これ以上の角度差で強旋回

# ★追加: 真逆 (方位角が目標のちょうど180°反対) 付近での旋回方向拮抗対策。
#        diff (azimuth と target_bearing の差) は -180〜+180 の範囲で
#        表現されるため、機体が目標の真逆を向いているとき diff は
#        +180 付近にも -180 付近にもなり得る (符号はセンサノイズ次第で
#        簡単に反転する)。apply_diff() は diff の符号だけで
#        turn_left_strong / turn_right_strong を選ぶため、この境界付近では
#        ノイズのたびに左右の旋回指令が入れ替わり、機体がその場で
#        左右に拮抗して回転できず、結果的に目標と反対方向へ流れて
#        しまう不具合があった。
#        そこで |diff| がこの閾値以上のとき (=ほぼ真逆を向いているとき)
#        は diff の符号によらず必ず右回転 (turn_right_strong) を選ぶ
#        「不感帯」を設け、旋回方向を一意に固定して拮抗を防ぐ。
#        機体が回頭して |diff| がこの閾値を下回ったら、通常通り
#        diff の符号に応じた最短方向への旋回に自動的に切り替わる。
ANGLE_180_PREFER_RIGHT_DEG = 170.0   # [度]  |diff| がこれ以上なら右回転で統一

# スタック検知
# ★変更: 誤検知(実際はスタックしていないのに検知してしまう)対策として、
#        判定条件を全体的に厳しくした。
#          (1) 水平加速度による判定は「前進を指示している間」のみ有効にする。
#              旋回中はその場スピンにより並進加速度がもともと小さくなるため、
#              以前はこれを誤って「動いていない=スタック」と判定していた。
#          (2) 加速度に加えて GPS 速度が十分低いことも要求する (AND条件)。
#              振動などのノイズだけで加速度が一時的に下がった場合の誤検知を防ぐ。
#          (3) 連続判定に必要な時間を 2.0s → 4.0s に延長し、瞬間的な値の
#              揺れで確定しないようにした。
STUCK_HORIZON_ACCEL_THRESH = 0.5   # [m/s²]  水平加速度ノルムがこれ未満 → スタック疑い (前進中のみ判定)
STUCK_GPS_SPEED_THRESH_MPS = 0.15  # [m/s]   ★追加: GPS対地速度がこれ未満も併せて要求
KNOTS_TO_MPS = 0.514444            # [-]     ノット→m/s 変換係数
STUCK_SONAR_DIST_THRESH    = 0.3   # [m]     超音波距離がこれ未満でも → スタック疑い (常時判定)
STUCK_COUNT_THRESHOLD      = 40    # ★変更: 連続カウント数 (20→40, LOOP_DT × N 秒間継続で確定)

# ★変更: 固定秒数の旋回ではなく、地磁気(9軸)から算出した方位の変化量で
#        「ちょうど指定角度回転した」ことを確認してから前進する方式に変更。
STUCK_RECOVER_TURN_DEG          = 90.0   # [度] 初回回復: 直前と反対方向へ回転する角度
                                         #      ★変更 180→90: 180° 回って 10 秒進むと、誘導に戻った後に
                                         #      同じ段差へ同じ向きから突っ込む。90° で横に回り込む方が
                                         #      抜けやすい (再スタックのたびにさらに 90° ずつ回る)
STUCK_RECOVER_RETRY_TURN_DEG    = 90.0   # [度] 前進中に再スタックした場合の追加回転角度
STUCK_RECOVER_TURN_TIMEOUT_SEC  = 15.0   # [s] 回転動作の安全タイムアウト (方位が読めない場合の保険)
STUCK_RECOVER_FWD_SEC           = 10.0   # [s] 回復: 回転後の前進時間 (★変更: 30→10)
STUCK_RECOVER_MAX_RETRIES       = 5      # [回] 前進中の再スタック→90°回転を繰り返す最大回数 (無限ループ防止)

# ★改善: スタック検知の追加条件と、段差型/轍型に分けた回避動作 (mission/stuck.py)
STUCK_NOPROGRESS_SEC     = 12.0   # [s]  前進指令がこれだけ続いても…
STUCK_NOPROGRESS_DIST_M  = 1.0    # [m]  …GPS 位置がこれ以上動かなければスタック (空転でも振動で加速度が出る場合を拾う)
STUCK_SPIN_GYRO_DPS      = 25.0   # [deg/s] 走行指令中・GPS 速度なしでヨー角速度がこれ以上 = 片輪空転でその場回転
STUCK_SPIN_WINDOW        = 60     # [回] 直近この周期 (LOOP_DT × 60 = 6 s) のうち…
STUCK_SPIN_COUNT         = 45     # [回] …これだけ「回っている」周期があれば確定 (誘導の 180° 旋回は約 2.6 s なので区別できる)
STUCK_BACK_SEC           = 1.5    # [s]  段差型: まず後退する時間 (回復中だけ後退を許可)
STUCK_ROTATE_CHECK_SEC   = 3.0    # [s]  回転を始めてこの秒数で…
STUCK_ROTATE_MIN_DEG     = 15.0   # [度] …これだけ回れていなければ「タイヤが地面を掴めていない」= 轍型
STUCK_ROTATE_OK_FRAC     = 0.6    # [-]  目標角度のこの割合以上回れたら「向きを変えられた」とみなす
STUCK_TWIST_CYCLES       = 6      # [回] ツイスト (左右ピボット交互) の回数
STUCK_TWIST_PULSE_SEC    = 0.35   # [s]  ツイスト 1 回のパルス長
STUCK_TWIST_PULSE_GROWTH = 0.5    # [-]  ラウンドごとにパルスを (1 + これ×(ラウンド-1)) 倍に伸ばす
STUCK_ROCK_CYCLES        = 4      # [回] ロッキング (後退⇄前進) の回数
STUCK_ROCK_SEC           = 0.4    # [s]  ロッキング 1 回のパルス長
STUCK_MAX_ROUNDS         = 3      # [回] 回避のラウンド数の上限 (これで抜けられなければ誘導に戻る)
STUCK_PAUSE_SEC          = 0.15   # [s]  パルス間の停止時間 (モータの逆転を緩和)

# ★追加: 走行フェーズで目標までの距離・方位ずれをまとめてログ表示する周期
NAV_REPORT_INTERVAL_SEC = 5.0  # [s]

# --- Phase 4: 前方補正 (★変更) -----------------------------------------------
# ★変更: 以前は前方候補を4通り用意しGPS試行で自動探索する方式だったが、
#        欠陥があった (候補を切り替えても実際には機体を回転させておらず、
#        全候補で同じ方向に直進していただけで判別になっていなかった)。
#        BNO055 の Axis Remap 機能 (BNO_ORIENTATION, set_bno_orientation)
#        で Phase0 の初期化時に一度だけ設定する方式に置き換えたため、
#        Phase4 側で前方候補を探索する処理は不要になった。

# --- Phase 5: 超音波センサによる最終接近 (★変更: 左右スキャン方式) ------------
# Phase4 で目標まで PHASE4_TO_5_RADIUS 以内に入ったら開始する最終フェーズ。
# GPS はこれ以上正確に距離を詰められないため、前方の超音波センサで
# 目標物(ゴールに設置された物体)を検知しながら接近する。
# ★変更: 従来は距離を判断に使わず機械的に左右へ振っていたが、これを
#        「左に旋回しながら距離を測定 → 右に旋回しながら距離を測定 →
#        近かった方向へ旋回して進む」という、超音波センサの値に基づいて
#        能動的に対象物へ機体を向けていく方式に変更した。
#        (超音波センサは指向性があるため、旋回してその方向の距離が短く
#        なるほど、その方向に対象物が存在する可能性が高いという前提)
FINAL_STOP_DIST_M      = 0.05   # [m]  この距離以下で最終接近完了 (ゴール)
FINAL_SCAN_SEC          = 0.3   # [s]  左/右それぞれの旋回スキャンにかける時間
FINAL_APPROACH_TIMEOUT_SEC = 5 * 60  # [s]  Phase5 の安全タイムアウト (5分)
FINAL_NAV_REPORT_INTERVAL_SEC = 3.0  # [s]  距離・方位ずれのまとめログ周期
FINAL_SONAR_LOST_WARN_SEC = 15.0     # [s]  超音波が有効値を返さない状態がこの秒数続いたら警告
FINAL_SCAN_LOOP_DT      = 0.02  # [s]  スキャン中の超音波サンプリング周期
# ※ 上記 FINAL_STOP_DIST_M / FINAL_SCAN_* は、カメラが使えないときの
#   フォールバック (phase5_sonar_fallback) でのみ使う。

# --- Phase 5: カメラ色検知による最終接近 (★追加) -----------------------------
# 参考: YutakaOkutani/cansat-system (lib/cone_detect.py, mission/goal.py, phases/p6.py)
#       FROM-THE-EARTH/F.T.E.15th (NSE2025/Re-Star-t/library/detect_corn.py, main.py)
# カメラ設定
CAM_SIZE            = (320, 240)  # [px]  処理解像度 (小さいほど速く省メモリ)
CAM_FULL_FOV        = True        # True: IMX219 の全画角モード (1640x1232 ビニング) を使う
CAM_ROTATE_180      = True        # ★変更: カメラを上下逆に取り付けるため True (画像を180°回転 = 上下と左右を入れ替え)
                                  #   ※左右だけ入れ替えるとコーンが逆さに写り、形状判定で弾かれるので必ず180°回転にする
CAM_SWAP_RB         = False       # 赤と青が入れ替わって見える場合 True
CAM_WARMUP_SEC      = 1.5         # [s]  起動後の AE/AWB 収束待ち
CAM_FAIL_LIMIT      = 10          # [回] 連続撮影失敗がこれを超えたら超音波フォールバックへ
# 色検知 (既定は赤いカラーコーン。OpenCV と同じ H=0〜179 スケール)
CAM_HUE_RANGES      = ((0, 12), (165, 179))
CAM_S_MIN           = 100         # 彩度の基準下限 (画面の明るさに応じて自動で上下する)
CAM_V_MIN           = 70          # 明度の基準下限
CAM_RED_MARGIN      = 30          # R - max(G,B) の下限。赤以外のゴール色にするときは None
CAM_RED_MAX_GB_RATIO = 0.55       # max(G,B) <= これ×R (茶色い土の除外)。赤以外のゴール色なら None
CAM_MIN_AREA_RATIO  = 0.0015      # これ未満の色領域はノイズとして無視 (320x240 で約115px)
CAM_NEAR_AREA_RATIO = 0.20        # これ以上写ったら近距離 → 超音波で確認に入る
CAM_CONTACT_AREA_RATIO = 0.55     # これ以上写ったら接触寸前
CAM_BACKEND         = "auto"      # "auto": OpenCV があれば OpenCV 版 (形状評価あり)、無ければ numpy 版
CAM_SHAPE_MIN_SCORE = 0.25        # [0〜1] (OpenCV 版のみ) これ未満のコーンらしくない赤は無視。
                                  #        本物のコーンを見落とすようなら下げる (0 で形状判定なし)
# 操舵 (stop-and-go: 1 パルス動いて止まって撮影、を繰り返す)
CAM_SETTLE_SEC      = 0.15        # [s]  停止してから撮影までの待ち (ブレ・揺れ防止)
CAM_CENTER_TOL      = 0.10        # [-]  重心 x のずれ (±0.5) がこれ以内なら正面とみなす
CAM_TURN_MIN_SEC    = 0.08        # [s]  旋回パルスの最短
CAM_TURN_GAIN_SEC   = 0.6         # [s]  ずれ 1.0 あたりの旋回パルス長 (比例)
CAM_TURN_MAX_SEC    = 0.30        # [s]  旋回パルスの最長
CAM_FWD_FAR_SEC     = 0.8         # [s]  遠い時の前進パルス
CAM_FWD_NEAR_SEC    = 0.25        # [s]  近い時の前進パルス
# 探索 (見失ったとき)
CAM_LOST_HOLD_FRAMES   = 2        # [枚] これ以下の見失いは撮り直すだけ (一瞬の誤検知落ち対策)
CAM_SEARCH_TURN_SEC    = 0.25     # [s]  探索時の 1 回の旋回パルス (画角 62° の半分程度回る長さに調整)
CAM_SEARCH_LAP_DEG     = 360.0    # [度] ★追加: BNO055 の地磁気で測ってこれだけ回ったら「1周」
CAM_SEARCH_PULSES_PER_LAP = 24    # [回] 地磁気が読めないときに「約1周」とみなすパルス数 (実機で要調整)
CAM_SEARCH_MAX_PULSES_PER_LAP = 48  # [回] 地磁気で測っていても、これだけ旋回したら1周とみなす (地磁気異常時の保険)
CAM_SEARCH_REPOSITION_SEC = 2.0   # [s]  1周しても見つからない時に前進して視点を変える
CAM_SEARCH_MAX_LAPS    = 6        # [周] (GPS へ戻れないとき) これだけ探しても見つからなければ超音波フォールバックへ

# --- Phase5 → Phase4 への戻り (★追加) ----------------------------------------
# 1周探してもゴールが見えないときは、GPS の誤差でゴールから離れた場所にいる
# 可能性が高い。少し前進して視点を変えたあと Phase4 (GPS 誘導) に戻り、
# GPS でゴールに近づき直してから再び Phase5 に入る。
PHASE5_MAX_RETURNS_TO_GPS = 3     # [回] Phase4 へ戻る最大回数 (これを超えたら Phase5 内で探索を続ける)
PHASE4_REENTRY_MIN_SEC = 10.0     # [s]  Phase5 から戻った Phase4 は、この秒数は GPS 誘導で走ってから
                                  #      でないと Phase5 へ移らない (戻った直後は GPS 上まだ 2m 以内のため、
                                  #      すぐ Phase5 に戻って同じ場所で探し直すだけになるのを防ぐ)
PHASE5_RETURN_GPS = "RETURN_GPS"  # phase5_camera_approach() の戻り値: Phase4 へ戻る
# 障害物 (超音波は近いがカメラ上のゴールは小さい)
CAM_OBSTACLE_TURN_SEC  = 0.6      # [s]  回避の旋回
CAM_OBSTACLE_FWD_SEC   = 1.0      # [s]  回避の前進
# 記録
CAM_SAVE_INTERVAL_SEC  = 5.0      # [s]  この間隔で撮影画像を logs/cam_*/ に保存 (0 で無効)
CAM_SAVE_MAX           = 150      # [枚] 定期保存の上限 (SD カード容量対策)
CAM_SAVE_EVENT_MAX     = 60       # [枚] ★改善: 重要な瞬間 (初めて見えた・見失った・0m 確認) の画像は
                                  #      定期保存の上限とは別枠でこの枚数まで必ず保存する
CAM_REPORT_INTERVAL_SEC = 3.0     # [s]  まとめログの周期

# --- Phase 5: 超音波による 0m 到達確認 (★追加) ------------------------------
# cansat-system mission/goal.py と同じく「停止した状態で」複数回測り、中央値と
# ばらつきで判定する (走行中の 1 回の値では判定しない)。
GOAL_CHECK_TRIGGER_DIST_M = 0.40  # [m]  走行中の超音波がこれ以下 & 正面にゴール → 停止して確認
GOAL_SONAR_DIST_M      = 0.08     # [m]  中央値がこれ以下 → 0m 直前 (★超音波の取り付け位置に合わせて調整)
GOAL_SONAR_SAMPLES     = 5        # [回] 確認時に測る回数
GOAL_SONAR_MIN_VALID   = 3        # [回] 有効値がこれ未満なら判定しない
GOAL_SONAR_MAX_SPREAD_M = 0.05    # [m]  有効値の max-min がこれを超えたら不安定とみなす
GOAL_SONAR_INTERVAL_SEC = 0.06    # [s]  HC-SR04 の測定間隔 (残響を避けるため 60ms 以上)
GOAL_SETTLE_SEC        = 0.3      # [s]  確認前の静止待ち
GOAL_BACKOFF_SEC       = 0.25     # [s]  不感帯対策の後退パルス
GOAL_BACKOFF_CONFIRM_DIST_M = 0.20  # [m]  後退して測り直したときにこれ以下なら目の前にゴールあり
GOAL_FINAL_PUSH_SEC    = 0.30     # [s]  確認後に接触させるための最終前進 (0 で押し込まない)
GOAL_PUSH_TOLERANCE_M  = 0.05     # [m]  押し込み後に GOAL_SONAR_DIST_M + これ を超えたら失敗
GOAL_MAX_BLIND_RETRIES = 4        # [回] 不感帯→後退確認の失敗がこれを超えたら少し下がって再接近
GOAL_MAX_CONTACT_FAR   = 5        # [回] 「カメラは接触寸前なのに超音波は遠い」がこれだけ続いたら
                                  #      カメラと超音波が別の物を見ているとみなし、下がって再接近
GOAL_REAPPROACH_BACK_SEC = 1.0    # [s]  再接近のために下がる時間

# --- ★main6: 方位の融合 (地磁気 + GPS COG) ------------------------------------
HEADING_COG_MIN_SPEED_MPS = 0.30  # [m/s] GPS 速度がこれ以上のときだけ COG を信用する
HEADING_STRAIGHT_MIN_SEC  = 1.5   # [s]   前進 & ジャイロ小 がこれだけ続いたら「直進中」
HEADING_GYRO_MAX_DPS      = 8.0   # [deg/s] ヨー角速度がこれ以下なら曲がっていない
HEADING_COG_MAX_AGE_SEC   = 1.5   # [s]   GPS の値がこれより古ければ COG を使わない
HEADING_OFFSET_ALPHA      = 0.25  # [-]   ずれ学習の EMA 係数 (大きいほど新しい値に早く追従)
HEADING_MIN_OFFSET_SAMPLES = 3    # [回]  これだけ学習してから「地磁気 + ずれ」を使う
HEADING_OUTLIER_DEG       = 45.0  # [deg] 学習済みのずれからこれ以上外れた値は一時的な跳ねとして無視

# --- ★main6: BNO055 キャリブレーションプロファイル ----------------------------
BNO_CALIB_FILE = _ROOT_DIR / "config" / "bno055_calib.json"   # test/bno055_calibrate.py で作成
BNO_CALIB_LOAD = True             # 起動時にプロファイルを書き戻す
BNO_CALIB_AUTOSAVE = True         # Phase3 で Gyro/Mag が 3 になったら上書き保存する
CALIB_TIMEOUT_WITH_PROFILE_SEC = 30.0  # [s] プロファイルを書き戻せたときの Phase3 の待ち時間上限

# --- ★main6: Phase4 中のカメラによるゴール発見 → Phase5 へ早めに切り替え ------
CAM_P4_ENABLE              = True
CAM_P4_ACTIVATE_RADIUS_M   = 10.0  # [m] GPS でこの距離以内に入ったらカメラで見張り始める
CAM_P4_DEACTIVATE_RADIUS_M = 15.0  # [m] この距離より離れたらカメラを止める (行き来でのオンオフ連発を防ぐ)
CAM_P4_INTERVAL_SEC        = 0.25  # [s] 撮影・検知の間隔 (別スレッド)
CAM_P4_THREADED            = True  # True: 別スレッドで撮影 (本番)。False: 制御ループ内で撮影 (テスト用)
CONE_LOCK_FRAMES           = 3     # [枚] 続けて見えたらゴールと確定
CONE_LOCK_CONSISTENCY_DEG  = 20.0  # [deg] 連続するフレームの方向 (機体方位 + 画面上の方向) のばらつき許容
CONE_LOCK_GPS_CHECK_MIN_M  = 4.0   # [m] GPS 距離がこれより遠いときは GPS の目標方位とも照合する
CONE_LOCK_GPS_BEARING_TOL_DEG = 60.0  # [deg] GPS の目標方位とのずれの許容
PHASE4_TIMEOUT_P5_RADIUS_M = 20.0  # [m] Phase4 がタイムアウトしてもこの距離以内なら Phase5 を試す

# --- モータ: ソフトスタート (デューティ比の段階的引き上げ) (★追加) ---------
# 停止状態から動き出す瞬間にいきなり高いデューティ比をかけると、
# 突入電流やホイールスピン・機体への衝撃が大きくなりやすい。
# そこで「動き始め」は従来と同じ RAMP_START_DUTY (80%) から立ち上がり、
# そのまま動作を継続した場合は RAMP_DURATION_SEC 秒かけてなめらかに
# RAMP_END_DUTY (100%) まで引き上げていく方式にした。
# 一度 stop() で完全停止すると、次に動き出す際は再びこのカーブの
# 最初 (RAMP_START_DUTY) からやり直す。
RAMP_START_DUTY      = 0.8   # [-] 動き始め (t=0) のデューティ比 (=従来の MOTOR_SPEED と同じ)
RAMP_END_DUTY        = 1.0   # [-] 連続動作を続けた末の最終デューティ比 (フル出力)
RAMP_DURATION_SEC    = 3.0   # [s] START → END まで引き上げるのにかける時間
RAMP_UPDATE_INTERVAL_SEC = 0.2  # [s] バックグラウンドでデューティ比を再計算・反映する周期

# 弱旋回側 (turn_left_weak / turn_right_weak) の出力は、従来
# MOTOR_SPEED(0.8) に対して SPEED_WEAK(0.4) = 50% の比率だった。
# ソフトスタートを導入した後も同じ比率を保つよう、強旋回側の
SPEED_WEAK  = 0.4                          # [-] 従来の弱旋回側 duty (比率計算の元値として保持)
WEAK_RATIO  = SPEED_WEAK / RAMP_START_DUTY  # [-] 強旋回側に対する弱旋回側の出力比率 (デフォルト 0.5)

# ★削除: PHASE4_MOTOR_REVERSED (Phase4限定のモータ反転フラグ) は撤去した。
#        フラグを増やして個別補正しようとすると、フラグ同士の組み合わせで
#        逆に機体の動きが把握しづらくなる。モータは常に素直に forward=前進 /
#        backward=後退として動作し、取り付け向きの補正は BNO055 の
#        Axis Remap (BNO_ORIENTATION) だけに一本化する。

# モータピン (BCM) ← test_run.py / test_avoid.py と統一
PIN_PWMA = 13
PIN_AIN1 =  6
PIN_AIN2 =  5
PIN_PWMB = 18
PIN_BIN1 = 24
PIN_BIN2 = 23
PIN_STBY = 11

# LED
LED_PIN  = 21               # BCM21 ← test_onoff.py と統一

# GPS
GPS_PORT     = "/dev/serial0"
GPS_BAUDRATE = 9600

# 超音波センサ ← sensor/HC-SR04.py と統一
PIN_TRIG = 8
PIN_ECHO = 7
SONAR_SETTLE    = 0.01      # [s]
SONAR_PULSE     = 10e-6     # [s]
SONAR_TIMEOUT   = 0.03      # [s]
SOUND_SPEED     = 343.0     # [m/s]

# 地球半径
# ★変更: 従来は赤道半径 (6378136.59m) を使った平面近似 (等長円筒図法) で
#        distance/bearing を計算していたが、より標準的で精度の高い
#        Haversine距離 / 大圆初期方位角の式に変更したため、それに対応する
#        地球の平均半径 (WGS84 authalic mean radius に近い一般的な値) を使用する。
EARTH_RADIUS = 6371000.0   # [m]  Haversine計算用の地球平均半径

# ログ
LOG_DIR = _ROOT_DIR / "logs"

# --- 地磁気の平滑化 (元: グローバル共有変数セクション) ----------------------
MAG_EMA_ALPHA = 0.25   # [-] 0〜1。小さいほど平滑化が強い(応答は遅くなる)。
                       #     新規サンプルの反映比率: new = alpha*sample + (1-alpha)*old

# フェーズ番号 → 日本語名。ログに現在何をしているフェーズかを
# ひと目でわかるように併記するために使う。
PHASE_NAMES = {
    0: "初期化",
    1: "落下検知",
    2: "初期後退・停止・前進",
    3: "キャリブレーション/GPS安定化",
    4: "GPS誘導走行",
    5: "カメラ最終接近",
}

# ===========================================================================
# ★変更: BNO055 軸再配置 (Axis Remap) による前方補正
# ===========================================================================
# BNO055 の地磁気ゼロ点(方位0°の向き)と、機体の物理的な前方(モータで
# forward() させたときに実際に進む向き)は、センサの取り付け角度・配線・
# 個体差等によって必ずしも一致しない。
#
# ★変更の経緯: 以前はこれをソフトウェア側で「前方候補 1〜4 を実際に
# 前進走行させ、GPS距離の変化を見て正しい候補を選ぶ」という前方探索
# (Front Search) で解決しようとしていた。しかし、この実装には重大な
# 欠陥があった: 候補を切り替えても set_front_candidate() が内部の
# オフセット変数を書き換えるだけで、機体を実際に回転させる動作を
# 一切行わずに motor.forward() を呼んでいたため、候補1〜4のすべてで
# 機体は完全に同じ物理的方向へ直進していた。そのため候補間の違いは
# GPS測位誤差程度のノイズでしかなく、前方候補を正しく判別できず、
# 誤った候補が採択されて航法計算全体に恒常的なズレが乗ってしまう
# 危険があった。
#
# そこで test_GPSrun.py で採用されていた、BNO055 の Axis Remap 機能
# (レジスタ 0x41=軸配置, 0x42=符号) を使ってセンサの内部基準軸を
# 物理的な取り付け向きに合わせて再配置する方式に変更した。これは
# BNO055 データシートに定義された正式な機能であり、GPS ノイズに
# 依存する推測ではなく、設定した瞬間から常に一貫して正しい方位が
# 得られる。取り付け向きは実機で 1〜4 を順に試し、目標へ向かって
# 前進するものを BNO_ORIENTATION に設定して固定運用する。

# --- BNO055 の取り付け向き設定 (test_GPSrun.py と同じ Axis Remap 方式) ------
# 1 : 正規の向き (0°回転)
# 2 : Z軸周りに 90°回転
# 3 : Z軸周りに 180°回転
# 4 : Z軸周りに 270°回転 (-90°回転)
# ★実機で 1→2→3→4 の順に試し、Phase4 で目標へ正しく向かって前進する
#   設定値をここに固定すること。
BNO_ORIENTATION = 1
