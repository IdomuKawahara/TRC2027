"""
goal_vision.py
==============
カメラ (Raspberry Pi Camera Module v2 / IMX219) の色検知によるゴール検知と、
超音波センサによる「0m ゴール到達」確認の判定ロジック。

TRC2027/sensor/goal_vision.py

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
設計方針:
    * OpenCV (python3-opencv) があれば OpenCV 版、無ければ numpy 版で動く
      (backend="auto")。OpenCV 版は
        - 画素単位のモルフォロジー処理 + connectedComponentsWithStats で正確な領域抽出
        - 輪郭から形状 (縦長か・下が広いか・塗りつぶし率) を評価し、
          コーンらしくない赤 (赤い線・横長の赤い物体など) を除外
        - INTER_AREA 縮小 + ガウシアンぼかしでノイズに強い
      numpy 版は OpenCV が壊れている/未インストールの場合の保険
      (8x8 セル単位の近似ラベリング、形状評価なし)。
    * 色検知の手法は参考リポジトリを踏襲:
        - YutakaOkutani/cansat-system (lib/cone_detect.py):
            赤の HSV 2レンジ (H 0〜16 / 165〜179)、画面の明るさ (V の中央値)
            に応じて S/V 閾値を動かす適応閾値、占有率 + 画面端接触で近距離判定
        - FROM-THE-EARTH/F.T.E.15th (NSE2025/Re-Star-t/library/detect_corn.py):
            最大連結成分の重心 x (0〜1) で左右操舵、小さすぎる成分はノイズ除去、
            画面に対する面積比で「到達」判定
    * 0m 到達の最終判定はカメラだけで決めず、停止した状態で超音波を複数回
      測って中央値・ばらつきを見る (cansat-system mission/goal.py と同じ考え方)。
      判定ロジックは純粋関数 (judge_goal / judge_backoff / judge_after_push)
      にしてあり、実機なしでユニットテストできる。

単体実行 (画像ファイルに対して検知を試す):
    python3 sensor/goal_vision.py test_images/xxx.jpg --save-debug out.jpg
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import math
import threading
import time
from dataclasses import dataclass, field

import numpy as np

# OpenCV は import だけで約 40MB 使う (Pi Zero 2 W 実測) ため、モジュール読み込み時には
# import せず、OpenCV 版の検知器を作るとき (= Phase5 開始時) に初めて読み込む。
cv2 = None


def load_cv2() -> bool:
    """OpenCV を (まだなら) import する。使えれば True。"""
    global cv2
    if cv2 is not None:
        return True
    try:
        import cv2 as _cv2
        cv2 = _cv2
        return True
    except Exception:      # 未インストール / 共有ライブラリ不足など
        return False

# ===========================================================================
# 色空間変換
# ===========================================================================

def rgb_to_hsv_u8(rgb: np.ndarray):
    """
    RGB (uint8, HxWx3) → H, S, V (それぞれ uint8, HxW)。
    スケールは OpenCV と同じ: H = 0〜179 (度/2), S = 0〜255, V = 0〜255。
    """
    img = rgb.astype(np.float32)
    r, g, b = img[..., 0], img[..., 1], img[..., 2]
    maxc = img.max(axis=-1)
    minc = img.min(axis=-1)
    delta = maxc - minc

    v = maxc
    s = np.where(maxc > 0, 255.0 * delta / np.maximum(maxc, 1e-6), 0.0)

    safe = np.maximum(delta, 1e-6)
    h = np.zeros_like(maxc)
    is_r = (maxc == r)
    is_g = (maxc == g) & ~is_r
    is_b = ~is_r & ~is_g
    h = np.where(is_r, ((g - b) / safe) % 6.0, h)
    h = np.where(is_g, (b - r) / safe + 2.0, h)
    h = np.where(is_b, (r - g) / safe + 4.0, h)
    h = np.where(delta > 0, h * 30.0, 0.0)          # 60°/2 = 30 (OpenCV スケール)
    h = np.rint(h) % 180

    return h.astype(np.uint8), np.rint(s).astype(np.uint8), np.rint(v).astype(np.uint8)


# ===========================================================================
# 検知結果
# ===========================================================================

@dataclass
class GoalDetection:
    detected: bool = False
    area_ratio: float = 0.0     # 採用した連結成分の画素数 / 全画素
    frame_ratio: float = 0.0    # 画面全体でゴール色と判定された画素の割合
    cx: float = 0.5             # 重心 x (0=左端, 1=右端)
    cy: float = 0.5             # 重心 y (0=上端, 1=下端)
    bbox: tuple = (0, 0, 0, 0)  # (x, y, w, h) [px, 処理画像上]
    edge_touch: int = 0         # bbox が接している画面の辺の数 (0〜4)
    is_near: bool = False       # 近距離 (超音波確認に入ってよい大きさ)
    is_contact: bool = False    # 画面を覆うほど大きい (接触寸前/接触)
    s_min: int = 0              # 実際に使った S 閾値
    v_min: int = 0              # 実際に使った V 閾値
    proc_ms: float = 0.0
    width: int = 0
    height: int = 0
    backend: str = ""
    shape_score: float = -1.0   # コーンらしさ 0〜1 (-1 = 未評価: numpy 版 / 近距離 / 小さすぎる)
    aspect: float = 0.0         # bbox の 高さ/幅
    n_candidates: int = 0       # 面積下限を超えた色領域の数

    @property
    def offset(self) -> float:
        """画面中央からのずれ (-0.5〜+0.5)。正 = 右にある。"""
        return self.cx - 0.5

    def summary(self) -> str:
        if not self.detected:
            return (f"det=0 frame={self.frame_ratio:.3f} "
                    f"S>={self.s_min} V>={self.v_min} {self.proc_ms:.0f}ms")
        shape = f" shape={self.shape_score:.2f}" if self.shape_score >= 0 else ""
        return (f"det=1 area={self.area_ratio:.3f} cx={self.cx:.2f} cy={self.cy:.2f} "
                f"edge={self.edge_touch} near={int(self.is_near)} contact={int(self.is_contact)}"
                f"{shape} S>={self.s_min} V>={self.v_min} {self.proc_ms:.0f}ms")


# ===========================================================================
# 色検知器
# ===========================================================================

# 赤 (OpenCV H スケール)。cansat-system の default_hsv_ranges を参考に、
# 茶色い土・枯れ草を拾いにくいよう上限を 12 に少し狭めている。
RED_HUE_RANGES = ((0, 12), (165, 179))


class ColorGoalDetector:
    """
    単色ゴール検知器 (backend="auto" | "cv2" | "numpy")。

    共通の色判定:
      2. HSV 変換 → 色相レンジ & S/V 閾値でマスク
         (adaptive_sv=True なら画面の明るさに応じて S/V 閾値を調整)
      3. red_margin / red_max_gb_ratio が指定されていれば
         R - max(G, B) >= red_margin かつ max(G, B) <= red_max_gb_ratio * R も要求
         (茶色の土・肌色・夕方の地面など「彩度はあるが赤くない」ものを除外。
          例: 土 RGB(100,60,40) は H≈10 で色相だけでは赤に入ってしまう)
    OpenCV 版 (_detect_cv2):
      4. オープン (点ノイズ除去) → クローズ (コーンの白帯・影で割れた領域をつなぐ)
      5. connectedComponentsWithStats で領域を列挙し、面積の大きい順に評価
      6. 遠〜中距離で画面端に触れていない領域は形状を評価し (_shape_score)、
         コーンらしくないもの (横長・下がすぼまる等) を除外。
         score = 面積 × (0.5 + 形状) で最良の領域を採用
    numpy 版 (_detect_numpy):
      4. cell×cell 画素のセルに区切り、ゴール色画素が cell_fill 以上のセルを
         ON とした粗いグリッドで 8 近傍ラベリング (点ノイズはセル単位で落ちる)
      5. 最大の連結成分を採用 (形状評価なし)
    どちらも 面積比・重心・bbox・画面端接触・近距離/接触 を返す。
    """

    def __init__(self,
                 hue_ranges=RED_HUE_RANGES,
                 s_min=100, v_min=70,
                 adaptive_sv=True,
                 red_margin=30,
                 red_max_gb_ratio=0.55,
                 work_width=320,
                 cell=8,
                 cell_fill=0.30,
                 min_area_ratio=0.0015,
                 near_area_ratio=0.20,
                 contact_area_ratio=0.55,
                 contact_edge_touch=2,
                 backend="auto",
                 shape_min_score=0.25,
                 shape_min_height_px=14,
                 max_candidates=5):
        self.hue_ranges = tuple(hue_ranges)
        self.s_min = int(s_min)
        self.v_min = int(v_min)
        self.adaptive_sv = adaptive_sv
        self.red_margin = red_margin
        self.red_max_gb_ratio = red_max_gb_ratio
        self.work_width = int(work_width)
        self.cell = int(cell)
        self.cell_fill = float(cell_fill)
        self.min_area_ratio = float(min_area_ratio)
        self.near_area_ratio = float(near_area_ratio)
        self.contact_area_ratio = float(contact_area_ratio)
        self.contact_edge_touch = int(contact_edge_touch)
        self.shape_min_score = float(shape_min_score)
        self.shape_min_height_px = int(shape_min_height_px)
        self.max_candidates = int(max_candidates)
        if backend == "auto":
            backend = "cv2" if load_cv2() else "numpy"
        if backend == "cv2" and not load_cv2():
            raise RuntimeError("backend='cv2' が指定されたが OpenCV (cv2) を import できない")
        if backend not in ("cv2", "numpy"):
            raise ValueError(f"unknown backend: {backend}")
        self.backend = backend
        self.last_mask = None
        if backend == "cv2":
            self._k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            self._k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

    # ── 前処理 ────────────────────────────────────────────────────────
    def _downscale(self, rgb: np.ndarray) -> np.ndarray:
        h, w = rgb.shape[:2]
        if w > self.work_width:
            if self.backend == "cv2":
                nh = max(1, int(round(h * self.work_width / w)))
                return cv2.resize(rgb, (self.work_width, nh), interpolation=cv2.INTER_AREA)
            step = int(np.ceil(w / self.work_width))
            rgb = rgb[::step, ::step]
        return rgb

    def _thresholds(self, v: np.ndarray):
        """cansat-system の __adaptive_sv_threshold と同じ式 (基準値だけ可変)。
        明るい画面 (晴天) ほど閾値を下げ、暗い画面ほど上げる。"""
        if not self.adaptive_sv:
            return self.s_min, self.v_min
        v_med = float(np.median(v))
        s_min = int(np.clip(self.s_min - 10 - 0.20 * (v_med - 120.0), 55.0, 130.0))
        v_min = int(np.clip(self.v_min - 5 - 0.25 * (v_med - 120.0), 35.0, 95.0))
        return s_min, v_min

    def color_mask(self, rgb: np.ndarray):
        """ゴール色の 2 値マスク (bool, HxW) と使った閾値を返す。"""
        if self.backend == "cv2":
            hsv = cv2.cvtColor(cv2.GaussianBlur(rgb, (3, 3), 0), cv2.COLOR_RGB2HSV)
            h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
        else:
            h, s, v = rgb_to_hsv_u8(rgb)
        s_min, v_min = self._thresholds(v)
        hue_ok = np.zeros(h.shape, dtype=bool)
        for lo, hi in self.hue_ranges:
            hue_ok |= (h >= lo) & (h <= hi)
        mask = hue_ok & (s >= s_min) & (v >= v_min)
        if self.red_margin is not None or self.red_max_gb_ratio is not None:
            px = rgb.astype(np.int16)
            r = px[..., 0]
            gb = np.maximum(px[..., 1], px[..., 2])
            if self.red_margin is not None:
                mask &= (r - gb) >= self.red_margin
            if self.red_max_gb_ratio is not None:
                mask &= gb * 100 <= r * int(round(self.red_max_gb_ratio * 100))
        return mask, s_min, v_min

    # ── 連結成分 (粗いグリッド上) ─────────────────────────────────────
    @staticmethod
    def _label_largest(grid_on: np.ndarray, weight: np.ndarray):
        """grid_on (bool, gh×gw) の 8 近傍連結成分のうち weight 合計が最大のものを返す。
        戻り値: (セル座標のリスト [(y, x), ...], weight 合計)"""
        gh, gw = grid_on.shape
        visited = np.zeros_like(grid_on)
        best_cells, best_w = [], 0.0
        ys, xs = np.nonzero(grid_on)
        for y0, x0 in zip(ys.tolist(), xs.tolist()):
            if visited[y0, x0]:
                continue
            stack = [(y0, x0)]
            visited[y0, x0] = True
            cells, wsum = [], 0.0
            while stack:
                y, x = stack.pop()
                cells.append((y, x))
                wsum += float(weight[y, x])
                for dy in (-1, 0, 1):
                    ny = y + dy
                    if ny < 0 or ny >= gh:
                        continue
                    for dx in (-1, 0, 1):
                        nx = x + dx
                        if 0 <= nx < gw and grid_on[ny, nx] and not visited[ny, nx]:
                            visited[ny, nx] = True
                            stack.append((ny, nx))
            if wsum > best_w:
                best_cells, best_w = cells, wsum
        return best_cells, best_w

    # ── メイン ────────────────────────────────────────────────────────
    def detect(self, rgb: np.ndarray) -> GoalDetection:
        t0 = time.perf_counter()
        if rgb is None or rgb.ndim != 3 or rgb.shape[2] < 3:
            return GoalDetection(backend=self.backend)
        rgb = self._downscale(np.ascontiguousarray(rgb[..., :3]))
        H, W = rgb.shape[:2]
        mask, s_min, v_min = self.color_mask(rgb)
        res = GoalDetection(s_min=s_min, v_min=v_min, width=W, height=H, backend=self.backend)
        if self.backend == "cv2":
            self._detect_cv2(mask, res)
        else:
            self.last_mask = mask
            res.frame_ratio = float(mask.sum()) / float(H * W)
            self._detect_numpy(mask, res)
        res.proc_ms = (time.perf_counter() - t0) * 1000.0
        return res

    def _set_flags(self, res: GoalDetection):
        res.is_contact = (res.area_ratio >= self.contact_area_ratio
                          or (res.frame_ratio >= self.contact_area_ratio
                              and res.edge_touch >= self.contact_edge_touch))
        res.is_near = res.is_contact or res.area_ratio >= self.near_area_ratio

    # ── OpenCV 版 ─────────────────────────────────────────────────────
    @staticmethod
    def _shape_score(comp: np.ndarray):
        """
        1 つの領域 (bbox で切り出した 0/1 画像) のコーンらしさを 0〜1 で返す。
        戻り値: (score, 高さ/幅)
          * 縦長さ: カラーコーンは 高さ/幅 ≈ 1.2〜3。横長 (<0.7) は大きく減点
          * 下が広い: 下 1/3 の幅 ≥ 上 1/3 の幅 (逆さまの三角形は大きく減点)
          * 塗りつぶし率: 三角形 ≈ 0.5、矩形 ≈ 1。0.4 未満から減点、0.2 以下 (線・枠状) は 0
        白い反射帯で上下に割れていてもクローズ処理でほぼつながる前提。
        """
        h, w = comp.shape
        aspect = h / max(w, 1)
        if aspect < 0.7:
            s_aspect = max(0.0, (aspect - 0.3) / 0.4) * 0.5
        elif aspect <= 4.0:
            s_aspect = 1.0
        else:
            s_aspect = max(0.0, 1.0 - (aspect - 4.0) / 4.0)

        third = max(1, h // 3)
        top_w = float(np.count_nonzero(comp[:third].any(axis=0)))
        bot_w = float(np.count_nonzero(comp[h - third:].any(axis=0)))
        if bot_w <= 0:
            s_taper = 0.0
        else:
            ratio = top_w / bot_w          # コーン: < 1, 矩形: ≈ 1, 逆三角: > 1
            s_taper = 1.0 if ratio <= 1.1 else max(0.0, 1.0 - (ratio - 1.1) / 0.9)

        fill = float(np.count_nonzero(comp)) / float(h * w)
        s_fill = float(np.clip((fill - 0.2) / 0.2, 0.0, 1.0))

        return float(s_aspect * (0.2 + 0.8 * s_taper) * s_fill), float(aspect)

    def _detect_cv2(self, mask: np.ndarray, res: GoalDetection):
        H, W = mask.shape
        total = float(H * W)
        m = mask.astype(np.uint8) * 255
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, self._k_open)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, self._k_close)
        self.last_mask = m > 0
        res.frame_ratio = float(np.count_nonzero(m)) / total

        n, labels, stats, cents = cv2.connectedComponentsWithStats(m, connectivity=8)
        if n <= 1:
            return
        areas = stats[1:, cv2.CC_STAT_AREA]
        order = np.argsort(areas)[::-1] + 1
        min_px = self.min_area_ratio * total
        best = None
        for idx in order[:self.max_candidates]:
            area = int(stats[idx, cv2.CC_STAT_AREA])
            if area < min_px:
                break
            res.n_candidates += 1
            x, y, w, h = (int(stats[idx, k]) for k in (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP,
                                                       cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
            edge = int(x == 0) + int(y == 0) + int(x + w >= W) + int(y + h >= H)
            area_ratio = area / total
            shape, aspect = -1.0, h / max(w, 1)
            # 近距離・画面端で切れている・小さすぎる領域は形がわからないので評価しない
            if area_ratio < self.near_area_ratio and edge == 0 and h >= self.shape_min_height_px:
                comp = (labels[y:y + h, x:x + w] == idx)
                shape, aspect = self._shape_score(comp)
                if shape < self.shape_min_score:
                    continue
            score = area * (0.5 + (shape if shape >= 0 else 0.5))
            if best is None or score > best[0]:
                best = (score, idx, area_ratio, (x, y, w, h), edge, shape, aspect)
        if best is None:
            return
        _, idx, area_ratio, bbox, edge, shape, aspect = best
        res.detected = True
        res.area_ratio = area_ratio
        res.cx = float(cents[idx][0]) / W
        res.cy = float(cents[idx][1]) / H
        res.bbox = bbox
        res.edge_touch = edge
        res.shape_score = shape
        res.aspect = aspect
        self._set_flags(res)

    # ── numpy 版 (OpenCV が無いときの保険) ────────────────────────────
    def _detect_numpy(self, mask: np.ndarray, res: GoalDetection):
        H, W = mask.shape
        total = float(H * W)
        c = self.cell
        gh, gw = H // c, W // c
        if gh == 0 or gw == 0:
            return
        cells = mask[:gh * c, :gw * c].reshape(gh, c, gw, c).sum(axis=(1, 3))
        grid_on = cells >= (self.cell_fill * c * c)
        best_cells, best_px = self._label_largest(grid_on, cells)

        area_ratio = best_px / total
        if best_cells and area_ratio >= self.min_area_ratio:
            cy_idx = np.array([p[0] for p in best_cells])
            cx_idx = np.array([p[1] for p in best_cells])
            wts = cells[cy_idx, cx_idx].astype(np.float64)
            cx_px = float(((cx_idx + 0.5) * c * wts).sum() / wts.sum())
            cy_px = float(((cy_idx + 0.5) * c * wts).sum() / wts.sum())
            x0, x1 = int(cx_idx.min()), int(cx_idx.max())
            y0, y1 = int(cy_idx.min()), int(cy_idx.max())
            edge = int(x0 == 0) + int(y0 == 0) + int(x1 == gw - 1) + int(y1 == gh - 1)
            res.detected = True
            res.area_ratio = area_ratio
            res.cx = cx_px / (gw * c)
            res.cy = cy_px / (gh * c)
            res.bbox = (x0 * c, y0 * c, (x1 - x0 + 1) * c, (y1 - y0 + 1) * c)
            res.edge_touch = edge
            res.aspect = res.bbox[3] / max(res.bbox[2], 1)
            res.n_candidates = 1
            self._set_flags(res)


# ===========================================================================
# 操舵判断 (F.T.E.15th の 0.4 / 0.6 不感帯方式を比例パルス化)
# ===========================================================================

def decide_steer(det: GoalDetection, center_tol=0.10,
                 turn_min_sec=0.08, turn_gain_sec=0.6, turn_max_sec=0.30,
                 fwd_far_sec=0.8, fwd_near_sec=0.25):
    """
    検知結果から次に出す 1 パルス分のモータ指令を決める。
    戻り値: (動作 "LEFT" / "RIGHT" / "FORWARD" / "NONE", パルス秒数)
      * |offset| > center_tol → ずれに比例した長さだけその場旋回
      * 中央に捉えていれば前進 (近いほど短いパルスで慎重に)
    """
    if not det.detected:
        return "NONE", 0.0
    off = det.offset
    if abs(off) > center_tol:
        sec = min(turn_max_sec, turn_min_sec + turn_gain_sec * (abs(off) - center_tol))
        return ("RIGHT" if off > 0 else "LEFT"), sec
    return "FORWARD", (fwd_near_sec if det.is_near else fwd_far_sec)


# ===========================================================================
# 超音波によるゴール確認
# ===========================================================================

@dataclass
class SonarSummary:
    n_total: int = 0
    n_valid: int = 0
    median: float = None    # 有効値の中央値 [m] (有効値なしなら None)
    spread: float = None    # 有効値の max - min [m]
    values: list = field(default_factory=list)

    def summary(self) -> str:
        med = f"{self.median:.3f}" if self.median is not None else "---"
        spr = f"{self.spread:.3f}" if self.spread is not None else "---"
        return f"sonar valid={self.n_valid}/{self.n_total} med={med}m spread={spr}m"


def summarize_sonar(samples, min_valid_m=0.02, max_valid_m=4.0) -> SonarSummary:
    """None (測定失敗) と HC-SR04 の測定範囲外の値を除いて中央値・ばらつきを求める。"""
    samples = list(samples)
    valid = sorted(float(x) for x in samples
                   if x is not None and min_valid_m <= float(x) <= max_valid_m)
    out = SonarSummary(n_total=len(samples), n_valid=len(valid), values=valid)
    if valid:
        n = len(valid)
        out.median = valid[n // 2] if n % 2 else 0.5 * (valid[n // 2 - 1] + valid[n // 2])
        out.spread = valid[-1] - valid[0]
    return out


@dataclass
class GoalCheckConfig:
    goal_dist_m: float = 0.08        # 超音波がこれ以下 → 0m 直前 (センサの取り付け位置に合わせて調整)
    min_valid: int = 3               # 有効サンプル数の下限
    max_spread_m: float = 0.05       # 有効サンプルのばらつき上限
    backoff_confirm_dist_m: float = 0.20  # 後退して測り直したときにこれ以下なら「目の前にゴールがある」
    push_tolerance_m: float = 0.05   # 最終押し込み後、goal_dist_m + これ を超えたらゴールが動いた/外れた


def judge_goal(sonar: SonarSummary, det: GoalDetection, cfg: GoalCheckConfig):
    """
    停止中に取ったカメラ 1 枚 + 超音波複数サンプルから、0m 直前かどうかを判定する。
    戻り値: (status, reason)
      CONFIRMED     : 超音波が goal_dist_m 以内で安定 & カメラも近距離で捉えている
      FAR           : 超音波は安定しているがまだ遠い → 前進を続ける
      OBSTACLE      : 超音波は近いがカメラ上のゴールは小さい → ゴール以外の障害物
      BLIND         : カメラは接触寸前なのに超音波が測れない (至近距離の不感帯の疑い)
                      → 少し後退して測り直す
      NO_VISUAL     : カメラでゴールが見えていない
      SONAR_INVALID : 超音波の有効値が足りない (カメラもまだ接触寸前ではない)
      NOISY         : 超音波のばらつきが大きく信用できない
    """
    if not det.detected:
        return "NO_VISUAL", "カメラにゴール色が写っていない"
    if sonar.n_valid >= cfg.min_valid:
        if sonar.spread is not None and sonar.spread > cfg.max_spread_m:
            if det.is_contact:
                return "BLIND", f"カメラ接触判定だが超音波が不安定 (spread={sonar.spread:.3f}m)"
            return "NOISY", f"超音波のばらつきが大きい (spread={sonar.spread:.3f}m)"
        if sonar.median <= cfg.goal_dist_m:
            if det.is_near:
                return "CONFIRMED", f"超音波 {sonar.median:.3f}m <= {cfg.goal_dist_m:.3f}m かつカメラ近距離"
            return "OBSTACLE", (f"超音波 {sonar.median:.3f}m と近いがカメラ上のゴールは小さい "
                                f"(area={det.area_ratio:.3f})")
        return "FAR", f"超音波 {sonar.median:.3f}m > {cfg.goal_dist_m:.3f}m"
    if det.is_contact:
        return "BLIND", f"カメラ接触判定だが超音波の有効値 {sonar.n_valid}/{sonar.n_total}"
    return "SONAR_INVALID", f"超音波の有効値不足 {sonar.n_valid}/{sonar.n_total}"


def judge_backoff(sonar: SonarSummary, det: GoalDetection, cfg: GoalCheckConfig):
    """BLIND のあと少し後退して測り直した結果の判定。
    戻り値: (ok, reason)  ok=True なら「目の前にゴールがある」ことを超音波で確認できた。"""
    if not det.detected:
        return False, "後退後にカメラでゴールが見えない"
    if sonar.n_valid < cfg.min_valid:
        return False, f"後退後も超音波の有効値不足 {sonar.n_valid}/{sonar.n_total}"
    if sonar.spread is not None and sonar.spread > cfg.max_spread_m:
        return False, f"後退後の超音波が不安定 (spread={sonar.spread:.3f}m)"
    if sonar.median <= cfg.backoff_confirm_dist_m:
        return True, f"後退後の超音波 {sonar.median:.3f}m <= {cfg.backoff_confirm_dist_m:.3f}m"
    return False, f"後退後の超音波 {sonar.median:.3f}m (ゴールが離れている)"


def judge_after_push(sonar: SonarSummary, det: GoalDetection, cfg: GoalCheckConfig):
    """最終押し込み (接触) 後の検証。
    接触状態では HC-SR04 は不感帯で測れないことが多いので、
      * 超音波が有効 → goal_dist_m + push_tolerance_m 以内であること
      * 超音波が無効 → カメラが接近/接触状態のままであること (不感帯とみなす)
    戻り値: (ok, reason)"""
    limit = cfg.goal_dist_m + cfg.push_tolerance_m
    if sonar.n_valid >= cfg.min_valid and sonar.median is not None:
        if sonar.median <= limit:
            return True, f"押し込み後の超音波 {sonar.median:.3f}m <= {limit:.3f}m"
        return False, f"押し込み後に超音波 {sonar.median:.3f}m (ゴールが動いた/外れた)"
    if det.detected and det.is_near:
        return True, "押し込み後は超音波不感帯 & カメラはゴールを至近で捉えたまま"
    return False, "押し込み後に超音波もカメラもゴールを確認できない"


# ===========================================================================
# 画像入出力 (PIL)
# ===========================================================================

def load_image_rgb(path) -> np.ndarray:
    from PIL import Image
    with Image.open(path) as im:
        return np.asarray(im.convert("RGB"))


def save_rgb_jpeg(rgb: np.ndarray, path, det: GoalDetection = None, quality=80):
    """デバッグ用に画像を保存する。det があれば bbox と重心を描き込む。"""
    from PIL import Image, ImageDraw
    im = Image.fromarray(np.ascontiguousarray(rgb[..., :3]))
    if det is not None and det.detected and det.width:
        sx = im.width / det.width
        sy = im.height / det.height
        x, y, w, h = det.bbox
        d = ImageDraw.Draw(im)
        d.rectangle([x * sx, y * sy, (x + w) * sx - 1, (y + h) * sy - 1], outline=(0, 255, 0), width=2)
        px, py = det.cx * im.width, det.cy * im.height
        d.line([px - 6, py, px + 6, py], fill=(255, 255, 0), width=2)
        d.line([px, py - 6, px, py + 6], fill=(255, 255, 0), width=2)
    im.save(str(path), quality=quality)


# ===========================================================================
# カメラ (Picamera2)
# ===========================================================================

class PiCameraCapture:
    """
    Picamera2 で 1 枚ずつ RGB 配列を取得する薄いラッパ。
      * main の解像度は小さく (既定 320x240) してメモリと処理時間を節約
      * full_fov=True なら IMX219 の 1640x1232 (2x2 ビニング, 全画角) センサモードを
        要求する。何も指定しないと 640x480 モード (中央クロップで画角が狭い) が
        選ばれ、ゴールを画角に入れにくくなるため。
      * Picamera2 の "RGB888" は配列上 B,G,R の順で並ぶため、RGB に並べ替えて返す
        (色が入れ替わって見える場合は swap_rb=True)。
      * 取り付けが上下逆なら rotate180=True。
    """

    def __init__(self, size=(320, 240), full_fov=True, rotate180=False,
                 swap_rb=False, warmup_sec=1.0, log=print):
        self.size = tuple(size)
        self.full_fov = full_fov
        self.rotate180 = rotate180
        self.swap_rb = swap_rb
        self.warmup_sec = warmup_sec
        self.log = log
        self.cam = None
        self.last_error = None

    def start(self) -> bool:
        if self.cam is not None:
            return True          # 起動済み (Phase4 から引き継いだ場合など)
        try:
            from picamera2 import Picamera2
        except Exception as e:
            self.last_error = f"picamera2 import 失敗: {e}"
            self.log(f"[Camera] {self.last_error}")
            return False
        try:
            self.cam = Picamera2()
            cfg = None
            if self.full_fov:
                try:
                    cfg = self.cam.create_video_configuration(
                        main={"size": self.size, "format": "RGB888"},
                        raw={"size": (1640, 1232)}, buffer_count=2)
                    self.cam.configure(cfg)
                except Exception as e:
                    self.log(f"[Camera] 全画角モード設定に失敗 ({e}) → 既定モードで再設定")
                    cfg = None
            if cfg is None:
                cfg = self.cam.create_video_configuration(
                    main={"size": self.size, "format": "RGB888"}, buffer_count=2)
                self.cam.configure(cfg)
            self.cam.start()
            time.sleep(self.warmup_sec)   # AE/AWB の収束待ち
            self.log(f"[Camera] 初期化完了 size={self.size} full_fov={self.full_fov} "
                     f"rotate180={self.rotate180} swap_rb={self.swap_rb}")
            return True
        except Exception as e:
            self.last_error = f"初期化失敗: {e}"
            self.log(f"[Camera] {self.last_error}")
            self.close()
            return False

    def capture_rgb(self, discard=1):
        """1 枚取得して RGB (HxWx3, uint8) を返す。失敗時は None。
        discard 枚を捨ててから取得する (停止前の動いている最中のフレームを避けるため)。"""
        if self.cam is None:
            return None
        try:
            for _ in range(max(0, int(discard))):
                self.cam.capture_array("main")
            arr = self.cam.capture_array("main")
            arr = arr[..., :3]
            if not self.swap_rb:
                arr = arr[..., ::-1]          # BGR → RGB
            if self.rotate180:
                arr = arr[::-1, ::-1]
            self.last_error = None
            return np.ascontiguousarray(arr)
        except Exception as e:
            self.last_error = f"撮影失敗: {e}"
            self.log(f"[Camera] {self.last_error}")
            return None

    def close(self):
        if self.cam is not None:
            try:
                self.cam.stop()
            except Exception:
                pass
            try:
                self.cam.close()
            except Exception:
                pass
            self.cam = None


# ===========================================================================
# Phase4 用: 走行中にゴールを見張る (ConeWatcher) / ゴールと確定する (ConeLock)
# ===========================================================================

HFOV_DEG = 62.2   # IMX219 全画角モードの水平画角 [deg]


def image_bearing_deg(det: GoalDetection, hfov_deg: float = HFOV_DEG) -> float:
    """画面上の重心 x から、カメラ正面に対するゴールの方向 [deg] (+ = 右)。"""
    half = math.radians(hfov_deg / 2)
    return math.degrees(math.atan((det.cx - 0.5) * 2.0 * math.tan(half)))


class ConeWatcher:
    """
    カメラで撮影 → 検知 を interval 秒ごとに繰り返し、最新の検知結果を保持する。
    threaded=True (本番) なら別スレッドで回し、Phase4 の制御ループ (0.1 秒周期)
    を止めない。threaded=False (テスト) なら latest() を呼んだときに必要なら撮影する。
    """

    def __init__(self, camera, detector, interval=0.25, threaded=True, clock=time.time):
        self.camera = camera
        self.detector = detector
        self.interval = interval
        self.threaded = threaded
        self.clock = clock
        self._lock = threading.Lock()
        self._latest = None          # (GoalDetection, 撮影時刻, 通し番号)
        self._seq = 0
        self._last_shot = -1e18
        self._stop = threading.Event()
        self._thread = None
        self.fail_count = 0

    def _shoot(self):
        rgb = self.camera.capture_rgb(discard=0)
        if rgb is None:
            self.fail_count += 1
            return
        self.fail_count = 0
        det = self.detector.detect(rgb)
        with self._lock:
            self._seq += 1
            self._latest = (det, self.clock(), self._seq)

    def _loop(self):
        while not self._stop.is_set():
            t0 = time.time()
            try:
                self._shoot()
            except Exception:
                self.fail_count += 1
            self._stop.wait(max(0.0, self.interval - (time.time() - t0)))

    def start(self) -> bool:
        if not self.camera.start():
            return False
        if self.threaded:
            self._stop.clear()
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()
        return True

    def latest(self):
        """最新の (GoalDetection, 撮影時刻, 通し番号)。まだ無ければ None。"""
        if not self.threaded:
            now = self.clock()
            if now - self._last_shot >= self.interval:
                self._last_shot = now
                self._shoot()
        with self._lock:
            return self._latest

    def stop(self):
        """撮影ループを止める (カメラは閉じない: Phase5 に引き継ぐため)。"""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def close(self):
        self.stop()
        self.camera.close()


class ConeLock:
    """
    Phase4 の走行中に見えた赤い物を「ゴール」と確定してよいかを判断する。
      * confirm_frames 枚続けて見えている
      * 連続するフレームで、地図上の方向 (機体方位 + 画面上の方向) が
        consistency_deg 以内でそろっている (一瞬の誤検知や別々の物を弾く)
      * GPS 上の距離が gps_check_min_dist_m より遠いときは、GPS の目標方位とも
        gps_bearing_tol_deg 以内で合っている (遠くの無関係な赤い物を弾く)。
        近くでは GPS の方位が当てにならないので確認しない。
    """

    def __init__(self, confirm_frames=3, consistency_deg=20.0,
                 gps_check_min_dist_m=4.0, gps_bearing_tol_deg=60.0, hfov_deg=HFOV_DEG):
        self.confirm_frames = confirm_frames
        self.consistency_deg = consistency_deg
        self.gps_check_min_dist_m = gps_check_min_dist_m
        self.gps_bearing_tol_deg = gps_bearing_tol_deg
        self.hfov_deg = hfov_deg
        self.count = 0
        self.last_world_bearing = None
        self.last_reason = ""
        self.world_bearing = None

    def reset(self):
        self.count = 0
        self.last_world_bearing = None

    def update(self, det: GoalDetection, heading_deg: float,
               target_bearing_deg: float = None, gps_dist_m: float = None) -> bool:
        """新しいフレームごとに呼ぶ。ゴールと確定したら True。"""
        if det is None or not det.detected:
            self.reset()
            self.last_reason = "見えない"
            return False
        world = (heading_deg + image_bearing_deg(det, self.hfov_deg)) % 360.0
        if (gps_dist_m is not None and target_bearing_deg is not None
                and gps_dist_m > self.gps_check_min_dist_m):
            d = abs((world - target_bearing_deg + 180.0) % 360.0 - 180.0)
            if d > self.gps_bearing_tol_deg:
                self.reset()
                self.last_reason = f"GPS の目標方位と {d:.0f}° ずれている"
                return False
        if self.last_world_bearing is not None:
            d = abs((world - self.last_world_bearing + 180.0) % 360.0 - 180.0)
            if d > self.consistency_deg:
                self.count = 0
                self.last_reason = f"前のフレームと方向が {d:.0f}° 違う"
        self.last_world_bearing = world
        self.count += 1
        self.world_bearing = world
        if self.count >= self.confirm_frames:
            self.last_reason = f"{self.count} フレーム連続で方位 {world:.0f}° に検知"
            return True
        self.last_reason = f"確認中 {self.count}/{self.confirm_frames}"
        return False


# ===========================================================================
# 単体実行: 画像ファイルに対して検知
# ===========================================================================

if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="画像ファイルに対してゴール色検知を実行する")
    ap.add_argument("images", nargs="+")
    ap.add_argument("--save-debug", help="1枚目の検知結果を描き込んだ画像の保存先")
    ap.add_argument("--save-mask", help="1枚目のマスク画像の保存先")
    args = ap.parse_args()
    det_obj = ColorGoalDetector()
    for i, p in enumerate(args.images):
        img = load_image_rgb(p)
        r = det_obj.detect(img)
        print(f"{p}: shape={img.shape} → {r.summary()} steer={decide_steer(r)}")
        if i == 0 and args.save_debug:
            save_rgb_jpeg(img, args.save_debug, r)
        if i == 0 and args.save_mask:
            from PIL import Image
            Image.fromarray((det_obj.last_mask * 255).astype(np.uint8)).save(args.save_mask)
