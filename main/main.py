"""
main.py
=======
NSE2026 ミッションシーケンス メインプログラム (フェーズの切り替えのみ)

TRC2027/main/main.py

★ main6.py を mission/ パッケージに分割したもの (動作は同じ)。
   設定値は mission/config.py、各フェーズは mission/phaseN_*.py、
   共有状態は mission/state.py を参照。

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
main2.py からの主な変更 (★main6, 分割前の main6.py で実施):
  [方位の精度]
    * GPS の進行方位 (COG) と地磁気の融合 (sensor/heading_fusion.py)。
      まっすぐ前進中は COG を機体方位として使い、同時に「COG − 地磁気」の
      ずれを学習。旋回中・低速時は「地磁気 + 学習したずれ」を使う。
      モータの磁気による一定のずれ・取り付け誤差・偏角の誤差を自動で打ち消す。
    * BNO055 のキャリブレーション結果 (22 バイト) を保存/読込
      (sensor/bno_calib.py, BNO_CALIB_FILE)。事前に test/bno055_calibrate.py で
      作ったプロファイルを Phase0 で書き戻し、Phase3 の待ち時間を短縮する。
      Phase3 で地磁気・ジャイロが最高レベルになったら自動で上書き保存。
    * 修正: set_bno_orientation() が sensor/BNO055.py の関数名 (_set_mode /
      _write_byte) に対応しておらず、Axis Remap が書き込まれていなかった不具合。
  [Phase4 → Phase5 の切り替え]
    * GPS で CAM_P4_ACTIVATE_RADIUS_M 以内に入ったら、Phase4 の走行中に
      カメラでゴールを見張る (別スレッド)。ゴールが CONE_LOCK_FRAMES 枚続けて
      同じ方向に見え、GPS の目標方位とも大きく食い違わなければ、2m 圏に
      入る前でも Phase5 へ移る (カメラは開いたまま引き継ぐ)。
    * Phase4 がタイムアウトしても、GPS でゴールから
      PHASE4_TIMEOUT_P5_RADIUS_M 以内なら Phase5 を試す。
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
フェーズ構成:
    Phase 0 : 初期化 & SSH 切断耐性確立
                各センサ・モータを初期化する。
                nohup で自分自身が動いているため、SSH が切断されても
                ミッションは継続する。SSH が再接続されたらログが表示される。
                ログは /home/pi/NSE2026/logs/mission_YYYYMMDD_HHMMSS.csv に記録。

    Phase 1 : 落下検知
                BNO055 合成加速度ノルムが FALL_THRESHOLD 以下を
                FALL_COUNT_THRESHOLD 回連続 → 落下確定。
                FALL_TIMEOUT_SEC 経過でタイムアウト強制移行。

    Phase 2 : 超音波分離検知後退 → 停止 (5 秒) → 前進 (10 秒)
                サブキャリア脱出のため後退する。★変更: 固定30秒ではなく、
                前方の超音波センサでサブキャリアとの距離を継続監視し、
                分離した瞬間に生じる距離の急増 (PHASE2_SONAR_JUMP_THRESH_M
                以上) を検知した時点で後退を打ち切る (検知できない場合は
                PHASE2_BACK_TIMEOUT_SEC で安全タイムアウト)。
                後退完了後、5 秒停止して切り替えを落ち着かせた後、確実に
                サブキャリアから離脱・前方へ距離を取るために
                10 秒前進する。

    Phase 3 : キャリブレーション & 誘導走行準備
                BNO055 の地磁気・加速度・ジャイロキャリブレーション待機。
                「静止(セトリング+計測) CALIB_STILL_SEC 秒 (Acc/Gyro向け)
                ⇔ 左右交互に強旋回 FIG8_SPIN_SEC 秒 (Mag向け)」のサイクルを
                最初から回し続けることで、特定軸のキャリブレーションが
                原因で全体が停滞しないようにしている。
                【改良】さらに Acc/Gyro が実地でレベルアップしにくい問題に
                対応するため、軸ごとに現実的な目標レベルを個別設定できる
                ようにした (CALIB_MIN_LEVEL_SYS/GYRO/ACC/MAG。地上機は複数
                姿勢を取れないため Acc は緩め、Mag は比較的到達しやすいので
                従来通り高めに設定)。また、スピン直後は慣性の余韻で機体が
                完全には静止していないため、静止区間の頭に CALIB_SETTLE_SEC
                秒のセトリング(捨て時間)を設け、揺れている間を静止時間として
                誤ってカウントしないようにした。
                CALIB_REPORT_INTERVAL_SEC ごとに詳細な進捗をログへ出力し、
                特定軸が停滞している場合は CALIB_STALL_WARN_SEC ごとに警告する。
                【改良】実機ログ解析の結果、地上機は姿勢を変えられないため
                Acc(加速度)キャリブレーションレベルが構造的に 0 から絶対に
                上がらず、CALIB_TIMEOUT_SEC まで毎回タイムアウトしていたことが
                判明した。そのため CALIB_MIN_LEVEL_ACC を 0 (要求しない) に
                変更し、Sys/Gyro/Mag のみで完了判定するようにした。
                キャリブレーション後、静止基準加速度(スタック検知用)を測定し、
                続けて GPS Fix の安定化・平均化 (衛星数チェック + 複数サンプル
                平均) を行い、誘導走行開始時点での距離・方位を確認する。

    Phase 4 : GPS 誘導走行 (目標地点 PHASE4_TO_5_RADIUS = 2 m 以内まで)
                GPS + 地磁気で目標座標を追跡し前進。
                【改良】実機ログ解析の結果、(1) モータへの高デューティ通電中に
                地磁気センサの値が100ms間隔で数十〜100度以上ジャンプする
                ほどのノイズ(モータ電流による磁気干渉と推定)が発生し、
                機体方位が暴れて目標から遠ざかる事例が確認された。これに
                対応するため、地磁気ベクトルへ EMA(指数移動平均)フィルタ
                (calc_azimuth_with_front 内, MAG_EMA_ALPHA) を適用して
                ノイズを平滑化するようにした。(2) 距離・方位角の計算式を、
                従来の平面近似からより厳密な Haversine 距離公式・大圆初期
                方位角公式に変更し、精度を高めた。
                【安全ロック】Phase4 の開始から終了まで、モータの
                後退方向への回転を絶対に禁止する安全ロックを有効化する。
                backward() が万一 (バグ・将来の改修等で) 呼ばれても、
                MotorController 側で物理的に後退させず停止のみ行う
                (Phase4終了時にロックは解除され、以降のフェーズには影響しない)。
                【改良/修正】方位計算式は test_GPSrun.py と同じ単純な
                固定式 (calc_azimuth) に統一した。以前は「モータ反転
                フラグ (PHASE4_MOTOR_REVERSED)」「方位鏡写しフラグ
                (AZIMUTH_MIRROR_FIX)」など複数の反転系フラグを個別に
                導入していたが、フラグの組み合わせが増えるほど機体の
                動きが把握しづらくなる混乱を招いたため、これらを全て
                撤去し、モータは常に素直に forward=前進/backward=後退、
                方位計算も固定の単純な式で計算するフラットな設計に戻した。
                取り付け向きの補正は BNO055 の Axis Remap 機能
                (BNO_ORIENTATION, Phase0で1回設定) だけに一本化し、
                センサ内部の基準軸を機体の物理的な取り付け向きに合わせて
                再配置する、確実で単純な方式のみを採用する。
                スタック検知 (超音波 + 水平加速度 + GPS速度) で障害物を回避する。
                【改良】実運用でスタックしていないのに誤検知する事例があった
                ため、判定を厳格化した: 水平加速度による判定は「前進」を
                指示している間のみ有効とし (旋回中は元々並進加速度が小さく
                なるため誤検知の原因だった)、さらに GPS 対地速度が十分低い
                ことも AND 条件で要求し (加速度ノイズだけでの誤検知対策)、
                連続確定に必要な時間も 2.0s → 4.0s に延長した。
                スタック回復時は、検知直前に旋回していた方向と逆方向へ
                地磁気(9軸)の方位変化から【ちょうど180°】回転したことを
                確認してから STUCK_RECOVER_FWD_SEC (10秒) 前進する。
                この前進中も引き続きスタック判定を行い、再びスタックを
                検知した場合は 90° 回転してから再度 10 秒前進する
                (STUCK_RECOVER_MAX_RETRIES 回まで繰り返す)。
                走行中は NAV_REPORT_INTERVAL_SEC ごとに、9軸センサから
                算出した目標方位と機体方位のずれ、目標までの距離を
                まとめてログ表示する。
                目標まで PHASE4_TO_5_RADIUS 以内に入ったら Phase4 を終了し
                Phase5 へ移行する (タイムアウト時はそのままミッション終了)。

    Phase 5 : カメラ色検知による最終接近 + 超音波による 0m 到達確認 (★変更)
                Raspberry Pi Camera Module v2 でゴール色 (既定: 赤) を検知し、
                「停止 → 撮影 → 判断 → 短いパルスで旋回/前進」を繰り返す
                stop-and-go 方式でゴールへ近づく (sensor/goal_vision.py)。
                  * 見えない間 : その場で小刻みに旋回して探索 (最後に見えた側へ)。
                                 1周しても見つからなければ少し前進して視点を変える
                  * 見えている : 重心 x のずれに比例した旋回 → 中央なら前進
                  * 近距離     : カメラで大きく写る or 超音波が近い → 停止して
                                 超音波を複数回測り、中央値が GOAL_SONAR_DIST_M
                                 以内 & カメラも近距離 → 最後に押し込んで接触
                                 → 押し込み後にも超音波で再検証して完了
                  * 至近で超音波が測れない (不感帯) 場合は少し後退して測り直し、
                    目の前にゴールがあることを確認してから押し込む
                0m 到達は「超音波で確認できたとき」だけ成功とする。
                カメラが使えない場合は従来の超音波左右スキャン方式
                (phase5_sonar_fallback) に自動で切り替える。
                ★追加: その場で1周 (BNO055 の地磁気で 360° を確認) 探しても
                ゴールが見えなければ、少し前進して Phase4 (GPS 誘導) に戻り、
                GPS で近づき直してから再び Phase5 に入る (最大
                PHASE5_MAX_RETURNS_TO_GPS 回。戻った Phase4 は最低
                PHASE4_REENTRY_MIN_SEC 秒走ってから Phase5 へ移る)。
                カメラは上下逆に取り付けるため画像を 180° 回転して使う
                (CAM_ROTATE_180)。
                FINAL_APPROACH_TIMEOUT_SEC で安全タイムアウト。

    共通 : フェーズが切り替わるたびに LED を LED_BLINK_SEC 秒間点滅させ、
           現在どのフェーズに移行したかを外から視認できるようにする。

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
実行方法 (SSH 切断耐性):
    nohup python3 /home/pi/TRC2027/main/main.py > /home/pi/TRC2027/logs/nohup.log 2>&1 &
    # ↑ これで SSH が切れてもミッションが継続する

モータピン (BCM) ← test_run.py と統一:
    PWMA=13  AIN1=5  AIN2=6
    PWMB=18  BIN1=23 BIN2=24  STBY=11

センサ:
    BNO055   (I2C, NDOF)    ← test_finishv.py / fall.py と統一
    BMP180   (I2C, oss=3)   ← test_finishv.py と統一
    micropyGPS (/dev/serial0, 9600) ← test_GPSrun.py と統一
    HC-SR04  (TRIG=BCM8, ECHO=BCM7) ← sensor/HC-SR04.py と統一
    LED      (BCM21)        ← test_onoff.py と統一
    Camera   (Pi Camera Module v2 / IMX219, Picamera2) ← sensor/goal_vision.py
"""

import time

from mission import config as C
from mission import state as st
from mission.logger import log, save_log
from mission.navigation import calc_distance
from mission.phase0_init import phase0_init
from mission.phase1_fall import phase1_fall_detection
from mission.phase2_separation import phase2_initial_backward
from mission.phase3_calibration import phase3_calibration
from mission.phase4_gps import phase4_guided_run
from mission.phase5_camera import phase5_camera_approach


# ===========================================================================
# メイン
# ===========================================================================

def run_phase4_and_5(bno, sonar, motor, led, start_time, baseline_horiz_acc):
    """
    ★追加: Phase4 (GPS 誘導) → Phase5 (カメラ最終接近) を実行する。
    Phase5 が PHASE5_RETURN_GPS を返したら (1周してもゴールが見えない)、
    Phase4 からやり直す。戻る回数は PHASE5_MAX_RETURNS_TO_GPS 回まで。
    上限に達したら最後の Phase5 は GPS に戻らず、Phase5 内で探索を続ける
    (それでも見つからなければ超音波スキャンへ)。
    戻り値: Phase5 の結果 (True = 0m 到達) / Phase5 に入れなかったら None
    """
    returns = 0
    min_run = 0.0
    while True:
        phase4_result = phase4_guided_run(bno, sonar, motor, led, start_time,
                                          baseline_horiz_acc, min_run_sec=min_run)
        allow_return = returns < C.PHASE5_MAX_RETURNS_TO_GPS
        if phase4_result == "TIMEOUT":
            # ★main6: タイムアウトでもゴールの近くまで来ていれば Phase5 を試す
            dist = calc_distance(st.g_gps_lat, st.g_gps_lng) if st.g_gps_valid else float("inf")
            if dist <= C.PHASE4_TIMEOUT_P5_RADIUS_M:
                log(f"[Main] Phase4 はタイムアウトしましたが、ゴールまで {dist:.1f} m なので "
                    f"Phase5 を試します (GPS には戻りません)", "WARN")
                allow_return = False
            else:
                log(f"[Main] Phase4 がタイムアウト (ゴールまで {dist:.1f} m) のため Phase5 は実行しません。", "WARN")
                return None
        elif phase4_result not in ("NEAR_GOAL", "CONE_FOUND"):
            log(f"[Main] Phase4 の結果が {phase4_result} のため Phase5 は実行しません。", "WARN")
            return None
        result = phase5_camera_approach(sonar, motor, led, start_time, bno=bno,
                                        camera=st.g_p4_camera,
                                        allow_return_to_gps=allow_return)
        if result != C.PHASE5_RETURN_GPS:
            return result
        returns += 1
        min_run = C.PHASE4_REENTRY_MIN_SEC
        log(f"[Main] Phase5 → Phase4 へ戻ります ({returns}/{C.PHASE5_MAX_RETURNS_TO_GPS} 回目)", "WARN")


def main():
    # Phase 0: 初期化
    bno, bmp, base_pressure, sonar, motor, led, log_path = phase0_init()
    start_time = time.time()

    try:
        # Phase 1: 落下検知
        fall_ok = phase1_fall_detection(bno, led, start_time)
        if fall_ok:
            log("落下確認。次フェーズへ移行します。")
        else:
            log("タイムアウトのため強制移行します。", "WARN")

        # Phase 2: 初期後退 (30 s) → 停止 (5 s) → 前進 (10 s)
        phase2_initial_backward(motor, sonar, led, start_time)

        # Phase 3: キャリブレーション (静止⇔スピン サイクル) & GPS 安定化
        baseline_horiz_acc = phase3_calibration(bno, motor, led, start_time)

        # Phase 4 ⇄ Phase 5 (★変更)
        #   Phase 4: GPS 誘導走行 + スタック検知 (前方はPhase0のAxis Remapで補正済み)
        #   Phase 5: 目標付近 (PHASE4_TO_5_RADIUS 以内) まで来たらカメラで最終接近
        #            (0m 到達は超音波で確認。カメラ不調時は超音波スキャンへ自動切替)
        #   Phase5 で1周探してもゴールが見えなければ、少し移動して Phase4 に戻る
        #   (最大 PHASE5_MAX_RETURNS_TO_GPS 回。戻った Phase4 は最低
        #    PHASE4_REENTRY_MIN_SEC 秒は GPS 誘導で走ってから Phase5 へ移る)
        run_phase4_and_5(bno, sonar, motor, led, start_time, baseline_horiz_acc)

        log("=" * 62)
        log("ミッションシーケンス 完了")
        log("=" * 62)

    except KeyboardInterrupt:
        log("Ctrl+C を受信しました。緊急停止します。")
        motor.stop()

    except Exception as e:
        log(f"予期しないエラー: {e}", "ERROR")
        motor.stop()
        raise

    finally:
        motor.close()
        sonar.close()
        led.off()
        led.close()
        save_log(log_path)
        log("全リソース解放完了")


# ===========================================================================
if __name__ == "__main__":
    main()
