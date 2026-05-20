import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
import cv2
import numpy as np
import time
from typing import List, Optional, Tuple
from dataclasses import dataclass

# ==============================================================================
# 1. BIRD'S-EYE PERSPECTIVE TRANSFORM
# ==============================================================================

@dataclass
class PerspectiveTransform:
    src: np.ndarray                    # 4 source pts in image coords (TL, TR, BR, BL)
    dst: np.ndarray                    # 4 destination pts in bird's-eye coords
    M: np.ndarray                      # 3x3 forward warp matrix
    Minv: np.ndarray                   # 3x3 inverse warp matrix
    out_size: Tuple[int, int]          # (w, h) of the bird's-eye canvas

    @classmethod
    def build(cls, src_pts, dst_pts, out_size: Tuple[int, int]) -> "PerspectiveTransform":
        src = np.asarray(src_pts, dtype=np.float32)
        dst = np.asarray(dst_pts, dtype=np.float32)
        M = cv2.getPerspectiveTransform(src, dst)
        Minv = cv2.getPerspectiveTransform(dst, src)
        return cls(src=src, dst=dst, M=M, Minv=Minv, out_size=out_size)

    def warp(self, img: np.ndarray) -> np.ndarray:
        return cv2.warpPerspective(img, self.M, self.out_size, flags=cv2.INTER_LINEAR)

    def unwarp(self, img: np.ndarray, frame_size: Tuple[int, int]) -> np.ndarray:
        return cv2.warpPerspective(img, self.Minv, frame_size, flags=cv2.INTER_LINEAR)

    def unwarp_points(self, pts: np.ndarray) -> np.ndarray:
        """pts: Nx2 (x, y) in bird's-eye coords. Returns Nx2 in image coords."""
        if pts.size == 0:
            return pts.astype(np.int32).reshape(-1, 2)
        a = pts.astype(np.float32).reshape(-1, 1, 2)
        out = cv2.perspectiveTransform(a, self.Minv)
        return out.reshape(-1, 2)

    def warp_points(self, pts: np.ndarray) -> np.ndarray:
        if pts.size == 0:
            return pts.astype(np.int32).reshape(-1, 2)
        a = pts.astype(np.float32).reshape(-1, 1, 2)
        out = cv2.perspectiveTransform(a, self.M)
        return out.reshape(-1, 2)


def build_from_config(cfg, frame_w: int, frame_h: int) -> PerspectiveTransform:
    top_y = int(frame_h * cfg.persp_top_y_frac)
    bot_y = int(frame_h * cfg.persp_bottom_y_frac)

    has_corners = (
        getattr(cfg, "persp_top_left_frac", None) is not None
        and getattr(cfg, "persp_top_right_frac", None) is not None
        and getattr(cfg, "persp_bottom_left_frac", None) is not None
        and getattr(cfg, "persp_bottom_right_frac", None) is not None
    )
    if has_corners:
        tl_x = int(frame_w * cfg.persp_top_left_frac)
        tr_x = int(frame_w * cfg.persp_top_right_frac)
        bl_x = int(frame_w * cfg.persp_bottom_left_frac)
        br_x = int(frame_w * cfg.persp_bottom_right_frac)
    else:
        cx = frame_w // 2
        top_half = int(frame_w * cfg.persp_top_width_frac / 2)
        bot_half = int(frame_w * cfg.persp_bottom_width_frac / 2)
        tl_x, tr_x = cx - top_half, cx + top_half
        bl_x, br_x = cx - bot_half, cx + bot_half

    src = [
        (tl_x, top_y),   # TL
        (tr_x, top_y),   # TR
        (br_x, bot_y),   # BR
        (bl_x, bot_y),   # BL
    ]
    margin = int(frame_w * cfg.persp_dst_margin_frac)
    dst = [
        (margin,              0),
        (frame_w - margin,    0),
        (frame_w - margin,    frame_h),
        (margin,              frame_h),
    ]
    return PerspectiveTransform.build(src, dst, (frame_w, frame_h))

# ==============================================================================
# 2. LANE DETECTION RESULTS OBJECT
# ==============================================================================

@dataclass
class LaneDetection:
    found: bool
    frame_size: Tuple[int, int]              # (w, h) in image coords
    # Bird's-eye sampled polylines, unwarped back to image coords.
    left_polyline: Optional[np.ndarray] = None
    center_polyline: Optional[np.ndarray] = None
    right_polyline: Optional[np.ndarray] = None
    # Segmentation pixels in image coords (for painting onto the frame).
    left_pts: Optional[np.ndarray] = None
    center_pts: Optional[np.ndarray] = None
    right_pts: Optional[np.ndarray] = None
    # Point counts (informational).
    left_points: int = 0
    center_points: int = 0
    right_points: int = 0
    # Bird's-eye-space context, kept for offset calculations.
    bird_size: Tuple[int, int] = (0, 0)      # (w, h) of warp
    car_center_x_bird: int = 0               # where the car-trajectory column lands in bird's-eye
    car_center_x: int = 0                    # same column in original image coords (for overlay)
    left_fit_bird: Optional[np.ndarray] = None
    center_fit_bird: Optional[np.ndarray] = None
    right_fit_bird: Optional[np.ndarray] = None
    left_y_range_bird: Optional[Tuple[int, int]] = None
    center_y_range_bird: Optional[Tuple[int, int]] = None
    right_y_range_bird: Optional[Tuple[int, int]] = None

    def lane_center_offset(self, target_lane: str) -> Optional[float]:
        """Normalized [-1, 1] offset of the target lane's midline from the
        car's trajectory column. Positive = lane midline is to the right."""
        bw, bh = self.bird_size
        if bw == 0:
            return None
        lane = target_lane.upper()
        L = (self.left_fit_bird, self.left_y_range_bird)
        C = (self.center_fit_bird, self.center_y_range_bird)
        R = (self.right_fit_bird, self.right_y_range_bird)

        # Look-ahead sampling fraction
        look = float(getattr(self, "_look_ahead_frac", 0.6))

        def y_eval_pair(yr_a, yr_b):
            y_hi = min(yr_a[1], yr_b[1])
            y_lo = max(yr_a[0], yr_b[0])
            if y_hi <= y_lo:
                return None
            return float(y_hi - look * (y_hi - y_lo))

        def y_eval_one(yr):
            return float(yr[1] - look * (yr[1] - yr[0]))

        def mid_from_pair(pa, pb):
            if pa[0] is None or pb[0] is None:
                return None
            ye = y_eval_pair(pa[1], pb[1])
            if ye is None:
                return None
            a = float(np.polyval(pa[0], ye))
            b = float(np.polyval(pb[0], ye))
            return 0.5 * (a + b)

        mid: Optional[float] = None
        if lane == "L":
            mid = mid_from_pair(L, C) or mid_from_pair(L, R)
        elif lane == "R":
            mid = mid_from_pair(C, R) or mid_from_pair(L, R)
        else:
            mid = mid_from_pair(L, R)

        # Single-lane fallback: only one edge visible. Step `qw` along the lane normal.
        if mid is None:
            qw = self._lane_half_width_bird()
            for pol, y_range, sign_for_L, sign_for_R in [
                (L[0], L[1], +1, +1),   # only L
                (C[0], C[1], -1, +1),   # only C
                (R[0], R[1], -1, -1),   # only R
            ]:
                if pol is None:
                    continue
                ye = y_eval_one(y_range)
                edge_x = float(np.polyval(pol, ye))
                slope = float(np.polyval(np.polyder(pol), ye))
                step_x = qw / float(np.sqrt(1.0 + slope * slope))
                sign = sign_for_L if lane == "L" else sign_for_R
                mid = edge_x + sign * step_x
                break

        if mid is None:
            return None
        half = bw * 0.5
        return (mid - self.car_center_x_bird) / max(1.0, half)

    def _lane_half_width_bird(self) -> float:
        return float(getattr(self, "_lane_half_width_bird_px", 100.0))

# ==============================================================================
# 3. LANE DETECTION PIPELINE
# ==============================================================================

class LaneDetector:
    def __init__(self, cfg):
        self.cfg = cfg
        self._persp: Optional[PerspectiveTransform] = None
        self._frame_size: Optional[Tuple[int, int]] = None

    def detect(self, frame: np.ndarray) -> LaneDetection:
        h, w = frame.shape[:2]
        self._ensure_persp(w, h)

        empty = LaneDetection(found=False, frame_size=(w, h),
                               bird_size=self._persp.out_size,
                               car_center_x_bird=self._car_center_x_bird(w),
                               car_center_x=int(w * self.cfg.car_center_x_frac))
        if self._persp is None:
            return empty

        # 1. Warp to bird's-eye
        bird = self._persp.warp(frame)

        gray = cv2.cvtColor(bird, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 3)

        # Local contrast enhancement
        if self.cfg.clahe_enabled:
            clahe = cv2.createCLAHE(
                clipLimit=self.cfg.clahe_clip,
                tileGridSize=(self.cfg.clahe_tile, self.cfg.clahe_tile),
            )
            gray = clahe.apply(gray)

        # 2. Sheet mask: isolate the white/gray track sheet from background floor
        _, sheet = cv2.threshold(
            gray, self.cfg.sheet_threshold, 255, cv2.THRESH_BINARY
        )
        closek = max(1, self.cfg.sheet_close_kernel)
        ckernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (closek, closek))
        sheet = cv2.morphologyEx(sheet, cv2.MORPH_CLOSE, ckernel)
        
        nn, lbl, stats, _ = cv2.connectedComponentsWithStats(sheet, connectivity=8)
        if nn > 1:
            biggest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
            sheet_mask = (lbl == biggest).astype(np.uint8) * 255
            ek = max(1, self.cfg.sheet_erode_kernel)
            ekernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ek, ek))
            sheet_mask = cv2.erode(sheet_mask, ekernel)
        else:
            sheet_mask = np.zeros_like(gray)

        # 3. Adaptive threshold for dark lane line tapes
        block = self.cfg.adaptive_block_size
        if block % 2 == 0:
            block += 1
        tape_raw = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV,
            block, self.cfg.adaptive_c,
        )
        binary = cv2.bitwise_and(tape_raw, sheet_mask)

        # 4. Small reconnecting dilation
        dk = max(1, self.cfg.dilate_kernel)
        if dk > 1:
            kd = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dk, dk))
            binary = cv2.dilate(binary, kd)

        # 5. Morphological open to drop speckle noise
        bh, bw = binary.shape
        ok = max(1, self.cfg.open_kernel)
        if ok > 1:
            ke = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ok, ok))
            binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, ke)

        # 6. Cluster components by x-centroid
        num, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
        if num <= 1:
            return empty

        pieces: List[dict] = []
        for i in range(1, num):
            _, _, _, _, area = stats[i]
            if area < self.cfg.piece_min_area:
                continue
            comp_mask = (labels == i)
            ys_c, xs_c = np.where(comp_mask)
            pieces.append({
                "xc": float(np.mean(xs_c)),
                "xs": xs_c,
                "ys": ys_c,
            })
        if not pieces:
            return empty

        pieces.sort(key=lambda p: p["xc"])
        clusters: List[List[dict]] = []
        gap = self.cfg.cluster_gap_px_bird
        for p in pieces:
            if not clusters or (p["xc"] - clusters[-1][-1]["xc"]) > gap:
                clusters.append([p])
            else:
                clusters[-1].append(p)

        near_y_min = int(bh * (1.0 - self.cfg.near_car_fit_frac))

        candidates: List[Tuple[int, np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = []
        for cl in clusters:
            xs_all = np.concatenate([p["xs"] for p in cl])
            ys_all = np.concatenate([p["ys"] for p in cl])
            if len(xs_all) < self.cfg.min_lane_points:
                continue
            y_span = int(ys_all.max() - ys_all.min())
            if y_span < int(bh * self.cfg.min_lane_height_frac_bird):
                continue
            near = ys_all >= near_y_min
            if int(np.sum(near)) >= self.cfg.min_lane_points // 2:
                xs_fit = xs_all[near]
                ys_fit = ys_all[near]
            else:
                xs_fit = xs_all
                ys_fit = ys_all
            y_bot = int(ys_all.max())
            band = ys_all >= (y_bot - max(20, y_span // 5))
            start_x = int(np.mean(xs_all[band])) if np.any(band) else int(np.mean(xs_all))
            candidates.append((start_x, xs_fit, ys_fit, xs_all, ys_all))

        if not candidates:
            return empty

        # Take up to 3 candidates (left, center, right lines)
        candidates.sort(key=lambda t: len(t[3]), reverse=True)
        candidates = candidates[:3]
        candidates.sort(key=lambda t: t[0])
        tracked = candidates

        polylines: List[Optional[np.ndarray]] = []
        segs: List[Optional[np.ndarray]] = []
        fits_bird: List[Optional[np.ndarray]] = []
        y_ranges_bird: List[Optional[Tuple[int, int]]] = []
        counts: List[int] = []

        deg = max(1, self.cfg.fit_degree)
        for _, xs_fit, ys_fit, xs_all, ys_all in tracked:
            if len(np.unique(ys_fit)) < deg + 1:
                polylines.append(None); segs.append(None)
                fits_bird.append(None); y_ranges_bird.append(None); counts.append(0)
                continue
            fit = np.polyfit(ys_fit, xs_fit, deg)
            yf0 = int(ys_fit.min()); yf1 = int(ys_fit.max())
            y_samples = np.linspace(yf0, yf1, 48).astype(np.int32)
            x_samples = np.clip(np.polyval(fit, y_samples), 0, bw - 1).astype(np.int32)
            bird_poly = np.column_stack([x_samples, y_samples])
            img_poly = self._persp.unwarp_points(bird_poly).astype(np.int32)
            bird_seg = np.column_stack([xs_all, ys_all]).astype(np.int32)
            img_seg = self._persp.unwarp_points(bird_seg).astype(np.int32)
            polylines.append(img_poly)
            segs.append(img_seg)
            fits_bird.append(fit)
            y_ranges_bird.append((yf0, yf1))
            counts.append(int(len(xs_all)))

        # Assign L/C/R
        left_poly = center_poly = right_poly = None
        left_seg = center_seg = right_seg = None
        left_fit = center_fit = right_fit = None
        left_y = center_y = right_y = None
        lp = cp = rp = 0

        n = sum(1 for f in fits_bird if f is not None)
        if n == 1:
            i = next(k for k, f in enumerate(fits_bird) if f is not None)
            sx = tracked[i][0]
            if sx < bw / 2:
                left_poly, left_seg, left_fit, left_y, lp = (
                    polylines[i], segs[i], fits_bird[i], y_ranges_bird[i], counts[i]
                )
            else:
                right_poly, right_seg, right_fit, right_y, rp = (
                    polylines[i], segs[i], fits_bird[i], y_ranges_bird[i], counts[i]
                )
        elif n == 2:
            idxs = [k for k, f in enumerate(fits_bird) if f is not None]
            left_poly, left_seg, left_fit, left_y, lp = (
                polylines[idxs[0]], segs[idxs[0]], fits_bird[idxs[0]],
                y_ranges_bird[idxs[0]], counts[idxs[0]]
            )
            right_poly, right_seg, right_fit, right_y, rp = (
                polylines[idxs[1]], segs[idxs[1]], fits_bird[idxs[1]],
                y_ranges_bird[idxs[1]], counts[idxs[1]]
            )
        else:
            left_poly, left_seg, left_fit, left_y, lp = (
                polylines[0], segs[0], fits_bird[0], y_ranges_bird[0], counts[0]
            )
            center_poly, center_seg, center_fit, center_y, cp = (
                polylines[1], segs[1], fits_bird[1], y_ranges_bird[1], counts[1]
            )
            right_poly, right_seg, right_fit, right_y, rp = (
                polylines[2], segs[2], fits_bird[2], y_ranges_bird[2], counts[2]
            )

        any_lane = (left_fit is not None) or (center_fit is not None) or (right_fit is not None)
        lane_half_px = float(self.cfg.lane_half_width_bird)
        det_out = LaneDetection(
            found=any_lane,
            frame_size=(w, h),
            left_polyline=left_poly, center_polyline=center_poly, right_polyline=right_poly,
            left_pts=left_seg, center_pts=center_seg, right_pts=right_seg,
            left_points=lp, center_points=cp, right_points=rp,
            bird_size=self._persp.out_size,
            car_center_x_bird=self._car_center_x_bird(w),
            car_center_x=int(w * self.cfg.car_center_x_frac),
            left_fit_bird=left_fit, center_fit_bird=center_fit, right_fit_bird=right_fit,
            left_y_range_bird=left_y, center_y_range_bird=center_y, right_y_range_bird=right_y,
        )
        det_out._lane_half_width_bird_px = lane_half_px
        det_out._look_ahead_frac = float(self.cfg.look_ahead_frac)
        return det_out

    def _ensure_persp(self, w: int, h: int) -> None:
        if self._persp is not None and self._frame_size == (w, h):
            return
        self._persp = build_from_config(self.cfg, w, h)
        self._frame_size = (w, h)

    def _car_center_x_bird(self, frame_w: int) -> int:
        if self._persp is None:
            return frame_w // 2
        car_x_img = int(frame_w * self.cfg.car_center_x_frac)
        bottom_y_img = int(self._frame_size[1] - 1) if self._frame_size else 479
        pts = np.array([[car_x_img, bottom_y_img]], dtype=np.float32)
        warped = self._persp.warp_points(pts)
        return int(warped[0, 0])

# ==============================================================================
# 4. AV-STYLE HUD OVERLAY DRAWING
# ==============================================================================

LEFT_COLOR = (60, 180, 255)
CENTER_COLOR = (220, 120, 255)
RIGHT_COLOR = (255, 180, 60)
DRIVABLE_COLOR = (60, 210, 80)
TARGET_CENTER_COLOR = (0, 255, 255)
TRAJECTORY_COLOR = (80, 255, 160)
HUD_OK = (80, 255, 160)
HUD_WARN = (80, 150, 255)
HUD_TEXT = (235, 235, 235)
HUD_DIM = (160, 160, 160)

def draw_overlay(
    frame: np.ndarray,
    det: LaneDetection,
    steering: Optional[float] = None,
    fps: Optional[float] = None,
    target_lane: str = "R",
    perspective=None,
) -> np.ndarray:
    out = frame.copy()
    if perspective is not None:
        _draw_perspective_src(out, perspective)
    _draw_segmentation(out, det)
    if det.found:
        _draw_drivable(out, det, target_lane, perspective)
    _draw_polylines(out, det)
    _draw_trajectory_marker(out, det)
    _draw_hud_panel(out, det, steering=steering, fps=fps, target_lane=target_lane)
    _draw_heading(out, steering, det.car_center_x)
    return out

def _draw_perspective_src(out: np.ndarray, perspective) -> None:
    pts = perspective.src.astype(np.int32).reshape(-1, 1, 2)
    cv2.polylines(out, [pts], True, HUD_OK, 1, cv2.LINE_AA)

def _paint_pts(out: np.ndarray, pts: Optional[np.ndarray], color: tuple) -> None:
    if pts is None or len(pts) == 0:
        return
    h, w = out.shape[:2]
    xs = pts[:, 0]
    ys = pts[:, 1]
    m = (xs >= 0) & (xs < w) & (ys >= 0) & (ys < h)
    xs, ys = xs[m], ys[m]
    out[ys, xs] = color
    xs2 = np.clip(xs + 1, 0, w - 1)
    out[ys, xs2] = color

def _draw_segmentation(out: np.ndarray, det: LaneDetection) -> None:
    _paint_pts(out, det.left_pts, LEFT_COLOR)
    _paint_pts(out, det.center_pts, CENTER_COLOR)
    _paint_pts(out, det.right_pts, RIGHT_COLOR)

def _draw_polylines(out: np.ndarray, det: LaneDetection) -> None:
    _draw_one_poly(out, det.left_polyline, LEFT_COLOR, "L")
    _draw_one_poly(out, det.center_polyline, CENTER_COLOR, "C")
    _draw_one_poly(out, det.right_polyline, RIGHT_COLOR, "R")

def _draw_one_poly(out: np.ndarray, poly: Optional[np.ndarray], color: tuple, label: str) -> None:
    if poly is None or len(poly) < 2:
        return
    h, w = out.shape[:2]
    clip = poly.copy().astype(np.int32)
    clip[:, 0] = np.clip(clip[:, 0], 0, w - 1)
    clip[:, 1] = np.clip(clip[:, 1], 0, h - 1)
    cv2.polylines(out, [clip], False, color, 2, cv2.LINE_AA)
    anchor = clip[int(np.argmin(clip[:, 1]))]
    _draw_label(out, (int(anchor[0]), int(anchor[1])), label, color)

def _draw_label(out: np.ndarray, anchor: tuple, text: str, color: tuple) -> None:
    x, y = anchor
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
    bx0 = max(0, x - tw // 2 - 5)
    bx1 = bx0 + tw + 10
    by1 = max(th + 6, y - 4)
    by0 = by1 - th - 8
    cv2.rectangle(out, (bx0, by0), (bx1, by1), (20, 20, 20), -1)
    cv2.rectangle(out, (bx0, by0), (bx1, by1), color, 1, cv2.LINE_AA)
    cv2.putText(out, text, (bx0 + 5, by1 - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)

def _pick_target(det: LaneDetection, target_lane: str):
    lane = target_lane.upper()
    if lane == "L":
        a = (det.left_polyline, det.left_fit_bird, det.left_y_range_bird)
        b = (det.center_polyline, det.center_fit_bird, det.center_y_range_bird)
        if b[0] is None:
            b = (det.right_polyline, det.right_fit_bird, det.right_y_range_bird)
    else:
        a = (det.center_polyline, det.center_fit_bird, det.center_y_range_bird)
        if a[0] is None:
            a = (det.left_polyline, det.left_fit_bird, det.left_y_range_bird)
        b = (det.right_polyline, det.right_fit_bird, det.right_y_range_bird)
    return a, b

def _draw_drivable(out: np.ndarray, det: LaneDetection, target_lane: str, perspective) -> None:
    a, b = _pick_target(det, target_lane)
    if a[0] is None or b[0] is None:
        return
    if perspective is not None and a[1] is not None and b[1] is not None:
        y_lo = max(a[2][0], b[2][0])
        y_hi = min(a[2][1], b[2][1])
        if y_hi <= y_lo:
            return
        ys = np.linspace(y_lo, y_hi, 48).astype(np.int32)
        ax = np.polyval(a[1], ys)
        bx = np.polyval(b[1], ys)
        bird_poly = np.concatenate([
            np.column_stack([ax, ys]),
            np.column_stack([bx[::-1], ys[::-1]]),
        ]).astype(np.float32)
        img_poly = perspective.unwarp_points(bird_poly).astype(np.int32)
        layer = out.copy()
        cv2.fillPoly(layer, [img_poly], DRIVABLE_COLOR)
        cv2.addWeighted(layer, 0.22, out, 0.78, 0, out)
        
        # Target centerline
        cx = ((ax + bx) / 2.0).astype(np.float32)
        bird_center = np.column_stack([cx, ys])
        img_center = perspective.unwarp_points(bird_center).astype(np.int32)
        for i in range(0, len(img_center) - 1, 2):
            cv2.line(out, tuple(img_center[i]), tuple(img_center[i + 1]),
                     TARGET_CENTER_COLOR, 2, cv2.LINE_AA)

def _draw_trajectory_marker(out: np.ndarray, det: LaneDetection) -> None:
    h, w = out.shape[:2]
    cx = det.car_center_x if det.car_center_x else w // 2
    cx = int(max(0, min(w - 1, cx)))
    for y0 in range(h - 60, h - 12, 6):
        cv2.line(out, (cx, y0), (cx, y0 + 3), TRAJECTORY_COLOR, 1, cv2.LINE_AA)

def _draw_hud_panel(
    out: np.ndarray,
    det: LaneDetection,
    steering: Optional[float],
    fps: Optional[float],
    target_lane: str,
) -> None:
    x0, y0 = 10, 10
    x1, y1 = 230, 138
    roi = out[y0:y1, x0:x1]
    blk = np.zeros_like(roi)
    cv2.addWeighted(blk, 0.55, roi, 0.45, 0, roi)
    border = HUD_OK if det.found else HUD_WARN
    cv2.rectangle(out, (x0, y0), (x1, y1), border, 1, cv2.LINE_AA)
    status = "LANES LOCKED" if det.found else "SEARCHING"
    cv2.putText(out, status, (x0 + 10, y0 + 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, border, 1, cv2.LINE_AA)
    cv2.putText(out, f"LANE  {target_lane.upper()}", (x0 + 10, y0 + 46),
                cv2.FONT_HERSHEY_SIMPLEX, 0.46, TARGET_CENTER_COLOR, 1, cv2.LINE_AA)
    off = det.lane_center_offset(target_lane)
    off_txt = f"OFFSET {off:+.2f}" if off is not None else "OFFSET   --"
    cv2.putText(out, off_txt, (x0 + 10, y0 + 68),
                cv2.FONT_HERSHEY_SIMPLEX, 0.46, HUD_TEXT, 1, cv2.LINE_AA)
    s_txt = f"STEER  {steering:+.2f}" if steering is not None else "STEER   --"
    cv2.putText(out, s_txt, (x0 + 10, y0 + 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.46, HUD_TEXT, 1, cv2.LINE_AA)
    base_x, base_y = x0 + 10, y0 + 112
    _feature_dot(out, (base_x + 0, base_y), "L", LEFT_COLOR, det.left_fit_bird is not None)
    _feature_dot(out, (base_x + 60, base_y), "C", CENTER_COLOR, det.center_fit_bird is not None)
    _feature_dot(out, (base_x + 120, base_y), "R", RIGHT_COLOR, det.right_fit_bird is not None)
    if fps is not None:
        cv2.putText(out, f"{fps:4.1f} FPS", (out.shape[1] - 90, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, HUD_TEXT, 1, cv2.LINE_AA)

def _feature_dot(out: np.ndarray, pos: tuple, label: str, color: tuple, on: bool) -> None:
    x, y = pos
    if on:
        cv2.circle(out, (x, y), 6, color, -1, cv2.LINE_AA)
    cv2.circle(out, (x, y), 6, color, 1, cv2.LINE_AA)
    cv2.putText(out, label, (x + 10, y + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.46,
                color if on else HUD_DIM, 1, cv2.LINE_AA)

def _draw_heading(out: np.ndarray, steering: Optional[float], car_center_x: int) -> None:
    h, w = out.shape[:2]
    cx = int(max(0, min(w - 1, car_center_x if car_center_x else w // 2)))
    base_y = h - 28
    tip_y = h - 72
    s = 0.0 if steering is None else float(np.clip(steering, -1.0, 1.0))
    tip_x = cx + int(s * 110)
    cv2.arrowedLine(out, (cx, base_y), (tip_x, tip_y),
                    TARGET_CENTER_COLOR, 3, cv2.LINE_AA, tipLength=0.35)
    cv2.circle(out, (cx, base_y), 5, TARGET_CENTER_COLOR, -1, cv2.LINE_AA)
    cv2.circle(out, (cx, base_y), 9, TARGET_CENTER_COLOR, 1, cv2.LINE_AA)

# ==============================================================================
# 5. CONFIGURATION WRAPPER FROM ROS PARAMETERS
# ==============================================================================

class VisionConfig:
    def __init__(self, node: Node):
        self.node = node
        params = {
            'persp_top_y_frac': 0.18,
            'persp_bottom_y_frac': 0.88,
            'persp_top_left_frac': 0.15,
            'persp_top_right_frac': 0.72,
            'persp_bottom_left_frac': -0.05,
            'persp_bottom_right_frac': 1.00,
            'persp_dst_margin_frac': 0.10,
            'car_center_x_frac': 0.50,
            'adaptive_block_size': 31,
            'adaptive_c': 10,
            'dilate_kernel': 3,
            'sheet_threshold': 110,
            'sheet_close_kernel': 25,
            'sheet_erode_kernel': 9,
            'clahe_enabled': True,
            'clahe_clip': 2.5,
            'clahe_tile': 8,
            'open_kernel': 3,
            'piece_min_area': 20,
            'cluster_gap_px_bird': 55,
            'min_lane_height_frac_bird': 0.30,
            'min_lane_points': 120,
            'near_car_fit_frac': 0.60,
            'lane_half_width_bird': 100.0,
            'look_ahead_frac': 0.6,
            'fit_degree': 2,
            'nwindows': 12,
            'window_margin': 55,
            'window_minpix': 40,
            'min_peak_separation': 90,
        }
        for name, default in params.items():
            if not node.has_parameter(name):
                node.declare_parameter(name, default)

    def __getattr__(self, name):
        try:
            return self.node.get_parameter(name).value
        except Exception:
            raise AttributeError(f"VisionConfig has no attribute {name}")

# ==============================================================================
# 6. ROS 2 VISION NODE
# ==============================================================================

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # Declare standard parameters
        if not self.has_parameter('video_device'):
            self.declare_parameter('video_device', 1)
        if not self.has_parameter('target_lane'):
            self.declare_parameter('target_lane', 1)

        self.video_device = self.get_parameter('video_device').value
        self.target_lane = self.get_parameter('target_lane').value

        # Publisher for the error (distance from center)
        self.error_pub = self.create_publisher(Float32, 'lane_error', 10)
        
        # Subscriber to know which lane we should be looking for
        self.target_lane_sub = self.create_subscription(Int32, 'target_lane', self.lane_callback, 10)
        
        # Debug Image Publisher
        self.image_pub = self.create_publisher(Image, 'camera/debug_image', 10)
        self.compressed_image_pub = self.create_publisher(CompressedImage, 'camera/debug_image/compressed', 10)
        self.bridge = CvBridge()
        
        # Initialize LaneDetector
        self.cfg = VisionConfig(self)
        self.detector = LaneDetector(self.cfg)
        
        # Open USB Web Camera
        self.get_logger().info(f"Opening camera index {self.video_device} using CAP_V4L2...")
        self.cap = cv2.VideoCapture(self.video_device, cv2.CAP_V4L2)
        
        # Request the camera hardware to lower its resolution (Saves USB Bandwidth)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Parameter set callback
        self.add_on_set_parameters_callback(self.param_callback)

        self.timer = self.create_timer(0.05, self.process_frame)

    def param_callback(self, params):
        for param in params:
            if param.name == 'target_lane':
                self.target_lane = param.value
                self.get_logger().info(f"target_lane updated to: {self.target_lane}")
            elif param.name == 'video_device':
                self.video_device = param.value
                self.get_logger().info(f"video_device updated to: {self.video_device}. Reopening camera...")
                self.cap.release()
                self.cap = cv2.VideoCapture(self.video_device, cv2.CAP_V4L2)
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        return SetParametersResult(successful=True)

    def lane_callback(self, msg):
        self.target_lane = msg.data

    def process_frame(self):
        t0 = time.time()
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Camera connected, but frame is empty! Wrong camera index.")
            return

        # Force resize in software just in case the camera firmware ignored our hardware request
        frame = cv2.resize(frame, (640, 480))

        # Detect lane polylines & segments
        det = self.detector.detect(frame)

        # Map target_lane (1 = Right lane / R, 2 = Left lane / L)
        target_lane_str = "L" if self.target_lane == 2 else "R"

        error = 0.0
        if det.found:
            offset = det.lane_center_offset(target_lane_str)
            if offset is not None:
                bw = det.bird_size[0]
                half = bw * 0.5
                # Negative offset_pixels = lane is to left of center, so error is positive.
                # Positive offset_pixels = lane is to right of center, so error is negative.
                error = -offset * half
        
        # Publish the error
        self.error_pub.publish(Float32(data=float(error)))

        # Draw beautiful AV HUD overlay
        steering = -det.lane_center_offset(target_lane_str) if det.found else None
        fps = 1.0 / (time.time() - t0 + 1e-6)
        debug_frame = draw_overlay(
            frame,
            det,
            steering=steering,
            fps=fps,
            target_lane=target_lane_str,
            perspective=self.detector._persp
        )

        # Convert OpenCV frame to ROS message and publish raw
        img_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
        self.image_pub.publish(img_msg)

        # Convert to compressed image and publish for web clients
        ret_encoded, jpeg_buffer = cv2.imencode('.jpg', debug_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ret_encoded:
            comp_msg = CompressedImage()
            comp_msg.header.stamp = self.get_clock().now().to_msg()
            comp_msg.header.frame_id = "camera_link"
            comp_msg.format = "jpeg"
            comp_msg.data = jpeg_buffer.tobytes()
            self.compressed_image_pub.publish(comp_msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()