"""
test_phase5_live.py
===================
Phase5 (カメラ色検知による最終接近) だけを実機で試すためのテストツール。

TRC2027/test/test_phase5_live.py

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
モード:
  (既定) 検知モード  … カメラ (+ 超音波) だけを使い、モータは一切動かさない。
                       毎フレーム「ゴールがどこに見えるか」「Phase5 なら次に何をするか」
                       を表示する。机の上でも安全に試せる。
  --drive            … main2.py の phase5_camera_approach() をそのまま実行する。
                       ★モータが動く★ 機体を床に置き、周りに人や物がないことを確認すること。

使い方 (詳しくは test/README_phase5_test.md):
    cd ~/TRC2027
    python3 test/test_phase5_live.py                   # 検知モード (超音波あり)
    python3 test/test_phase5_live.py --no-sonar        # 検知モード (カメラだけ)
    python3 test/test_phase5_live.py --no-shape        # コーン以外の赤い物で試すとき
    python3 test/test_phase5_live.py --drive --timeout 60   # 本番と同じ Phase5 (モータ動作)
  終了は Ctrl+C
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import argparse
import datetime
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sensor"))
sys.path.insert(0, str(ROOT / "main"))

import main2 as M          # noqa: E402  (import だけではハードウェアは初期化されない)
import goal_vision as gv   # noqa: E402

# IMX219 を全画角モード (CAM_FULL_FOV=True) で使ったときの水平画角 [deg]
HFOV_DEG = 62.2

BAR_WIDTH = 25             # 位置バーの文字数 (奇数)


# ===========================================================================
# 表示用の計算
# ===========================================================================

def bearing_deg(det: gv.GoalDetection) -> float:
    """画面上の重心 x から、機体正面に対するゴールの方向 [deg] を求める (+ = 右)。"""
    half = math.radians(HFOV_DEG / 2)
    return math.degrees(math.atan((det.cx - 0.5) * 2.0 * math.tan(half)))


def estimate_distance_m(det: gv.GoalDetection, target_h_m: float):
    """bbox の高さ [px] とゴールの実際の高さ [m] から距離を推定する (ピンホールカメラ模型)。
    画面端で切れている/近すぎる場合は推定できないので None。"""
    if not det.detected or det.edge_touch > 0 or det.bbox[3] <= 0 or det.width <= 0:
        return None
    focal_px = (det.width / 2) / math.tan(math.radians(HFOV_DEG / 2))
    return focal_px * target_h_m / det.bbox[3]


def position_bar(det: gv.GoalDetection) -> str:
    """画面の左右のどこにゴールがあるかを文字で表す。
    [ ] = 正面とみなす範囲 (CAM_CENTER_TOL), | = 画面中央, O = ゴールの重心"""
    n = BAR_WIDTH
    bar = ["-"] * n
    mid = n // 2
    tol = int(round(M.CAM_CENTER_TOL * (n - 1)))
    bar[max(0, mid - tol)] = "["
    bar[min(n - 1, mid + tol)] = "]"
    bar[mid] = "|"
    if det.detected:
        pos = int(round(det.cx * (n - 1)))
        bar[max(0, min(n - 1, pos))] = "O"
    return "".join(bar)


def direction_words(det: gv.GoalDetection) -> str:
    if not det.detected:
        return "見えない"
    b = bearing_deg(det)
    if abs(det.offset) <= M.CAM_CENTER_TOL:
        return f"正面 ({b:+.1f}°)"
    return f"{'右' if b > 0 else '左'} {abs(b):.1f}°"


def planned_action(det, sonar_m, lost):
    """main2.phase5_camera_approach() のメインループと同じ条件で、
    Phase5 なら次に何をするかを返す。戻り値: (表示用文字列, 超音波確認が必要か)"""
    if not det.detected:
        if lost <= M.CAM_LOST_HOLD_FRAMES:
            return f"撮り直し(見失い{lost})", False
        return f"探索旋回 {M.CAM_SEARCH_TURN_SEC:.2f}s (1周で無ければ→Phase4)", False
    centered = abs(det.offset) <= M.CAM_CENTER_TOL
    sonar_close = sonar_m is not None and sonar_m <= M.GOAL_CHECK_TRIGGER_DIST_M
    if det.is_contact or (centered and sonar_close) or (centered and det.is_near and sonar_m is None):
        return "停止→超音波で0m確認", True
    action, sec = gv.decide_steer(
        det, center_tol=M.CAM_CENTER_TOL,
        turn_min_sec=M.CAM_TURN_MIN_SEC, turn_gain_sec=M.CAM_TURN_GAIN_SEC,
        turn_max_sec=M.CAM_TURN_MAX_SEC,
        fwd_far_sec=M.CAM_FWD_FAR_SEC, fwd_near_sec=M.CAM_FWD_NEAR_SEC)
    if action == "FORWARD" and sonar_close:
        sec = M.CAM_FWD_NEAR_SEC
    name = {"LEFT": "左へ旋回", "RIGHT": "右へ旋回", "FORWARD": "前進"}[action]
    return f"{name} {sec:.2f}s", False


# ===========================================================================
# 設定の上書き (コマンドライン → main2 の設定値)
# ===========================================================================

def apply_overrides(args):
    if args.no_shape:
        M.CAM_SHAPE_MIN_SCORE = 0.0
    if args.backend:
        M.CAM_BACKEND = args.backend
    if args.no_rotate:
        M.CAM_ROTATE_180 = False
    if args.swap_rb:
        M.CAM_SWAP_RB = True
    if args.hue:
        ranges = []
        for part in args.hue.split(","):
            lo, hi = part.split("-")
            ranges.append((int(lo), int(hi)))
        M.CAM_HUE_RANGES = tuple(ranges)
    if args.timeout:
        M.FINAL_APPROACH_TIMEOUT_SEC = args.timeout


def init_gpio():
    from gpiozero import Device
    from gpiozero.pins.lgpio import LGPIOFactory
    Device.pin_factory = LGPIOFactory()


# ===========================================================================
# 検知モード (モータを動かさない)
# ===========================================================================

def run_detect(args):
    M.phase = 5
    M.log("=" * 62)
    M.log("  Phase5 単体テスト (検知モード: モータは動かしません)")
    M.log("=" * 62)

    sonar = None
    if not args.no_sonar:
        try:
            init_gpio()
            sonar = M.SonarSensor()
        except Exception as e:
            M.log(f"超音波センサを初期化できません ({e}) → カメラだけで続行します", "WARN")
            sonar = None

    detector = M.make_goal_detector()
    cfg = M.make_goal_check_config()
    camera = M.make_camera()
    M.log(f"  色検知バックエンド: {detector.backend}  "
          f"形状判定: {'なし' if M.CAM_SHAPE_MIN_SCORE <= 0 else f'>= {M.CAM_SHAPE_MIN_SCORE}'}")
    M.log(f"  色: H={M.CAM_HUE_RANGES}  近距離: 面積>={M.CAM_NEAR_AREA_RATIO}  "
          f"接触: 面積>={M.CAM_CONTACT_AREA_RATIO}  正面: |ずれ|<={M.CAM_CENTER_TOL}")
    M.log(f"  カメラ画像の180°回転: {'あり (上下逆の取り付け)' if M.CAM_ROTATE_180 else 'なし'}")
    M.log(f"  距離推定に使うゴールの高さ: {args.target_height:.2f} m  "
          f"(コーン以外で試すときは --target-height で実物の高さを指定)")
    if not camera.start():
        M.log(f"カメラを起動できません: {camera.last_error}", "ERROR")
        if sonar:
            sonar.close()
        return 1

    out_dir = ROOT / "test_images" / f"phase5_live_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
    saved = 0
    last_save = 0.0
    last_report = 0.0
    lost = 0
    was_detected = False
    frames = 0
    n_detected = 0

    HDR = (f"{'T[s]':>7}  {'Det':>3}  {'Area':>6}  {'cx':>5}  {'Dir[°]':>7}  {'EstD[m]':>7}  "
           f"{'Sonar[m]':>8}  {'Shape':>5}  {'ms':>4}  {'Position (L ←  → R)':<{BAR_WIDTH}}  Phase5の判断")
    print()
    print("-" * (len(HDR) + 10))
    print(HDR)
    print("-" * (len(HDR) + 10))

    t0 = time.time()
    try:
        while True:
            loop_start = time.time()
            if args.duration and loop_start - t0 > args.duration:
                M.log(f"指定時間 ({args.duration:.0f} s) が経過したので終了します")
                break
            rgb = camera.capture_rgb(discard=0)
            if rgb is None:
                M.log(f"撮影失敗: {camera.last_error}", "WARN")
                time.sleep(0.5)
                continue
            det = detector.detect(rgb)
            frames += 1
            sonar_m = sonar.get_distance_m() if sonar else None
            M.g_sonar_m = sonar_m

            lost = 0 if det.detected else lost + 1
            n_detected += int(det.detected)
            plan, need_check = planned_action(det, sonar_m, lost)

            # 超音波確認 (本番と同じく複数回測って judge_goal で判定。モータは止まったまま)
            check_str = ""
            if need_check and sonar:
                vals = []
                for i in range(M.GOAL_SONAR_SAMPLES):
                    vals.append(sonar.get_distance_m())
                    if i < M.GOAL_SONAR_SAMPLES - 1:
                        time.sleep(M.GOAL_SONAR_INTERVAL_SEC)
                summ = gv.summarize_sonar(vals)
                status, reason = gv.judge_goal(summ, det, cfg)
                check_str = f" ⇒ {status}"
                M.log(f"[Phase5][GOAL確認] {status}: {reason} | {summ.summary()}")

            est = estimate_distance_m(det, args.target_height)
            row = (f"{loop_start - t0:>7.2f}  {int(det.detected):>3d}  {det.area_ratio:>6.3f}  "
                   f"{(f'{det.cx:.2f}' if det.detected else '---'):>5}  "
                   f"{(f'{bearing_deg(det):+.1f}' if det.detected else '---'):>7}  "
                   f"{(f'{est:.2f}' if est is not None else '---'):>7}  "
                   f"{(f'{sonar_m:.3f}' if sonar_m is not None else '---'):>8}  "
                   f"{(f'{det.shape_score:.2f}' if det.shape_score >= 0 else '---'):>5}  "
                   f"{det.proc_ms:>4.0f}  {position_bar(det):<{BAR_WIDTH}}  {plan}{check_str}")
            print(row, flush=True)

            # 見え始め/見失いはログにも残す
            if det.detected and not was_detected:
                M.log(f"[Phase5] ゴールを検知: {direction_words(det)}  {det.summary()}")
            elif not det.detected and was_detected:
                M.log("[Phase5] ゴールを見失いました")
            was_detected = det.detected

            # まとめログ (従来フェーズの NAV レポートと同じ位置づけ)
            if loop_start - last_report >= M.CAM_REPORT_INTERVAL_SEC:
                last_report = loop_start
                est_s = f"{est:.2f} m" if est is not None else "推定不可"
                M.log(f"[Phase5] ゴールの方向={direction_words(det)}  推定距離={est_s}  "
                      f"超音波={'---' if sonar_m is None else f'{sonar_m:.3f} m'}  "
                      f"→ Phase5の判断: {plan}{check_str}")
                M.log_data_status("Phase5 単体テスト (検知のみ)")

            # 画像保存 (検知結果を描き込む)
            save_now = (args.save_every > 0 and loop_start - last_save >= args.save_every) or check_str
            if save_now and saved < args.save_max:
                out_dir.mkdir(parents=True, exist_ok=True)
                tag = "det" if det.detected else "none"
                path = out_dir / f"{saved:03d}_{loop_start - t0:06.1f}s_{tag}.jpg"
                gv.save_rgb_jpeg(rgb, path, det)
                if args.save_mask and detector.last_mask is not None:
                    from PIL import Image
                    Image.fromarray(detector.last_mask.astype("uint8") * 255).save(
                        path.with_name(path.stem + "_mask.png"))
                saved += 1
                last_save = loop_start

            wait = args.interval - (time.time() - loop_start)
            if wait > 0:
                time.sleep(wait)

    except KeyboardInterrupt:
        print()
        M.log("Ctrl+C を受信しました。終了します。")
    finally:
        camera.close()
        if sonar:
            sonar.close()
        M.log("─" * 62)
        M.log(f"[Phase5] 結果: {frames} フレーム中 {n_detected} フレームでゴールを検知")
        if saved:
            M.log(f"[Phase5] 画像 {saved} 枚を保存: {out_dir}")
    return 0


# ===========================================================================
# 走行モード (main2.py の Phase5 をそのまま実行。モータが動く)
# ===========================================================================

def run_drive(args):
    print("\n" + "!" * 62)
    print("  ★ 走行モード: main2.py の Phase5 をそのまま実行します。モータが動きます ★")
    print(f"  タイムアウト {M.FINAL_APPROACH_TIMEOUT_SEC:.0f} 秒 / Ctrl+C で緊急停止")
    print("  機体を床に置き、周囲に人や物がないことを確認してください。")
    print("!" * 62)
    if not args.yes:
        try:
            ans = input("実行するには yes と入力してください: ").strip().lower()
        except EOFError:
            ans = ""
        if ans != "yes":
            print("中止しました。")
            return 1

    M.LED_BLINK_SEC = args.blink_sec
    init_gpio()
    led = M.LED(M.LED_PIN)
    led.on()
    sonar = M.SonarSensor()
    motor = M.MotorController()
    bno = None
    if not args.no_bno:
        try:
            b = M.BNO055()
            if b.setUp(operation_mode=M.BNO055.OPERATION_MODE_NDOF):
                M.set_bno_orientation(b, M.BNO_ORIENTATION)
                bno = b
        except Exception as e:
            M.log(f"BNO055 を初期化できません ({e})", "WARN")
    M.log(f"探索の1周判定: {'BNO055 の地磁気で 360°' if bno else f'旋回 {M.CAM_SEARCH_PULSES_PER_LAP} 回 (BNO055 なし)'}")
    M.LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = M.LOG_DIR / f"phase5_test_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    start = time.time()
    reached = False
    try:
        for i in range(3, 0, -1):
            M.log(f"{i} 秒後に Phase5 を開始します...")
            time.sleep(1.0)
        reached = M.phase5_camera_approach(sonar, motor, led, start, bno=bno)
    except KeyboardInterrupt:
        M.log("Ctrl+C を受信しました。緊急停止します。")
    finally:
        motor.stop()
        motor.close()
        sonar.close()
        led.off()
        led.close()
        M.save_log(log_path)
    if reached == M.PHASE5_RETURN_GPS:
        M.log("[Phase5 テスト] 結果: 1周してもゴールが見つからず、少し移動しました。"
              "本番ならここで Phase4 (GPS 誘導) に戻ります")
    else:
        M.log(f"[Phase5 テスト] 結果: {'0m 到達 (超音波で確認済み)' if reached is True else '未到達'}")
    return 0


# ===========================================================================

def main():
    ap = argparse.ArgumentParser(
        description="Phase5 (カメラ色検知による最終接近) の単体テスト。既定ではモータを動かさない。")
    ap.add_argument("--drive", action="store_true",
                    help="★モータが動く★ main2.py の Phase5 をそのまま実行する")
    ap.add_argument("--yes", action="store_true", help="--drive の確認入力を省略する")
    ap.add_argument("--timeout", type=float, default=0,
                    help="--drive 時の Phase5 タイムアウト [s] (既定: main2 の設定値)")
    ap.add_argument("--blink-sec", type=float, default=2.0,
                    help="--drive 時のフェーズ開始 LED 点滅秒数 (既定 2 秒に短縮)")
    ap.add_argument("--no-sonar", action="store_true", help="超音波センサを使わない (検知モードのみ)")
    ap.add_argument("--no-bno", action="store_true",
                    help="(--drive) BNO055 を使わず、旋回回数で探索の1周を判定する")
    ap.add_argument("--no-shape", action="store_true",
                    help="形状判定を切る (コーン以外の赤い物で試すとき)")
    ap.add_argument("--backend", choices=("auto", "cv2", "numpy"), help="色検知バックエンド")
    ap.add_argument("--hue", help="色相の範囲を上書き 例: 0-12,165-179 (OpenCV の H=0〜179)")
    ap.add_argument("--no-rotate", action="store_true",
                    help="画像を180°回転しない (カメラを機体から外して正立で持って試すとき。"
                         "既定は機体に上下逆に付けた状態に合わせて180°回転)")
    ap.add_argument("--swap-rb", action="store_true", help="赤と青が入れ替わって写る")
    ap.add_argument("--target-height", type=float, default=0.70,
                    help="距離推定に使うゴールの高さ [m] (既定 0.70 = カラーコーン)")
    ap.add_argument("--interval", type=float, default=0.3, help="検知モードの撮影間隔 [s]")
    ap.add_argument("--duration", type=float, default=0, help="検知モードの実行秒数 (0 = Ctrl+C まで)")
    ap.add_argument("--save-every", type=float, default=3.0,
                    help="検知結果を描いた画像を保存する間隔 [s] (0 で保存しない)")
    ap.add_argument("--save-max", type=int, default=50, help="保存する画像の最大枚数")
    ap.add_argument("--save-mask", action="store_true", help="色マスク画像も保存する")
    args = ap.parse_args()

    apply_overrides(args)
    return run_drive(args) if args.drive else run_detect(args)


if __name__ == "__main__":
    sys.exit(main())
