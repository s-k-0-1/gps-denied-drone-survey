#!/usr/bin/env python3
"""
yellow_detector.py  ·  Viman Rakshak  ·  IRoC-U 2026
======================================================
Standalone yellow-tape detector — no ROS dependency.

Tuned for:
  · Thick yellow tape (5–8 cm) on reddish-brown laterite soil
  · D455 color camera at 1280×720, ~3 m altitude
  · Outdoor daylight with variable shadow

Outputs per frame:
  · line_visible      — any yellow tape in frame
  · corner_visible    — L-shaped junction (two ⊥ segments)
  · corner_px         — (u, v) pixel of junction centroid
  · leading_density   — yellow fraction in each directional ROI
  · dominant_angle    — dominant tape angle in frame (°, or None)
  · debug_frame       — annotated BGR for logging / tuning

LINE-ONLY component filter (mission_luma):
  The mask keeps only components that look like real boundary tape —
  BIG, LONG, THIN, ELONGATED and RECTANGULAR — the same five tests and
  values as yellow_boundary_detector (area >= 600 px, length >= 30 % of
  the frame with a 200 px floor, stroke width <= 80 px, aspect ratio
  >= 3.0:1, rectangularity >= 0.40; the boundary-detector numbers
  scaled from proc-scale 0.5 to this detector's full 1280x720 frames).
  Small yellow patches (clothing, scraps, coloured objects) never reach
  the density / distance / corner outputs: a scrap is not a wall.

Offline tuning:
  python3 yellow_detector.py --tune path/to/frame.jpg
  python3 yellow_detector.py --test path/to/frame.jpg
"""

import math
import argparse
from dataclasses import dataclass, field
from typing import Optional, Tuple

import cv2
import numpy as np


# ── Default HSV bounds (OpenCV: H 0-179, S/V 0-255) ──────────────────────────
# Tuned for IRoC-U 2026 arena — outdoor daylight, laterite soil background.
# In direct sunlight yellow tape saturation drops (S≈80-160) vs indoor S≈150+
# Key discriminators vs laterite soil (H≈8-18, S≈40-90, V≈80-140):
#   · H_low=22 excludes orange-red soil
#   · V_low=120 excludes dark soil (tape is bright in sunlight)
DEFAULT_HSV_LOW  = np.array([20,  65, 100], dtype=np.uint8)
DEFAULT_HSV_HIGH = np.array([42, 255, 255], dtype=np.uint8)


@dataclass
class DetectionResult:
    line_visible:    bool  = False
    corner_visible:  bool  = False
    corner_px:       Optional[Tuple[int, int]] = None   # (u, v)
    dominant_angle:  Optional[float] = None             # degrees
    density_fwd:     float = 0.0   # yellow fraction in forward-facing ROI
    density_bck:     float = 0.0
    density_left:    float = 0.0
    density_right:   float = 0.0
    debug_frame:     Optional[np.ndarray] = field(default=None, repr=False)

    def leading_density(self, direction: str) -> float:
        """direction ∈ {'fwd','bck','left','right'}"""
        return {
            'fwd':   self.density_fwd,
            'bck':   self.density_bck,
            'left':  self.density_left,
            'right': self.density_right,
        }[direction]


class YellowDetector:
    """
    Yellow tape detector.  All parameters are tunable at construction time
    and can be updated in-flight via set_params().

    Parameters
    ----------
    hsv_low, hsv_high : np.ndarray shape (3,)
        HSV bounds for yellow.  OpenCV convention (H: 0-179).
    min_area_px : int
        Minimum contour area to consider a real tape segment (filters dust/noise).
    corner_tol_deg : float
        Two line clusters are considered perpendicular if |angle_diff - 90°| < this.
    hough_threshold : int
        HoughLinesP accumulator threshold (lower = more sensitive).
    min_line_len : int
        HoughLinesP minimum segment length in pixels.
    max_line_gap : int
        HoughLinesP maximum gap to bridge within a segment.
    line_filter : bool
        LINE-ONLY mask filter (mission_luma): keep only BIG + LONG +
        THIN + ELONGATED + RECTANGULAR components — real boundary tape.
    min_component_area_px, min_line_length_frac, min_line_length_px,
    max_line_width_px, min_aspect_ratio, min_rectangularity :
        The five line-filter thresholds; defaults mirror
        yellow_boundary_detector's mission_luma values scaled to this
        detector's full-resolution (1280x720) frames.
    debug : bool
        If True, DetectionResult.debug_frame is populated.
    """

    def __init__(
        self,
        hsv_low:        np.ndarray = DEFAULT_HSV_LOW,
        hsv_high:       np.ndarray = DEFAULT_HSV_HIGH,
        min_area_px:    int   = 300,
        corner_tol_deg: float = 20.0,
        hough_threshold:int   = 60,
        min_line_len:   int   = 80,
        max_line_gap:   int   = 30,
        line_filter:    bool  = True,
        min_component_area_px: int   = 600,
        min_line_length_frac:  float = 0.30,
        min_line_length_px:    int   = 200,
        max_line_width_px:     float = 80.0,
        min_aspect_ratio:      float = 3.0,
        min_rectangularity:    float = 0.40,
        debug:          bool  = False,
    ):
        self._hsv_low        = hsv_low.copy()
        self._hsv_high       = hsv_high.copy()
        self._min_area       = min_area_px
        self._corner_tol     = corner_tol_deg
        self._hough_thr      = hough_threshold
        self._min_line_len   = min_line_len
        self._max_line_gap   = max_line_gap
        self._line_filter    = line_filter
        self._min_comp_area  = min_component_area_px
        self._min_len_frac   = min_line_length_frac
        self._min_len_px     = min_line_length_px
        self._max_line_width = max_line_width_px
        self._min_aspect     = min_aspect_ratio
        self._min_rect       = min_rectangularity
        self._debug          = debug

        # CLAHE for V-channel normalisation (handles shadow / overexposure)
        self._clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def set_params(self, **kwargs):
        """Update any constructor parameter by name."""
        mapping = {
            'hsv_low':         '_hsv_low',
            'hsv_high':        '_hsv_high',
            'min_area_px':     '_min_area',
            'corner_tol_deg':  '_corner_tol',
            'hough_threshold': '_hough_thr',
            'min_line_len':    '_min_line_len',
            'max_line_gap':    '_max_line_gap',
            'line_filter':           '_line_filter',
            'min_component_area_px': '_min_comp_area',
            'min_line_length_frac':  '_min_len_frac',
            'min_line_length_px':    '_min_len_px',
            'max_line_width_px':     '_max_line_width',
            'min_aspect_ratio':      '_min_aspect',
            'min_rectangularity':    '_min_rect',
            'debug':           '_debug',
        }
        for k, v in kwargs.items():
            if k in mapping:
                setattr(self, mapping[k], v)

    # ── Public API ────────────────────────────────────────────────────────────

    def detect(self, bgr: np.ndarray) -> DetectionResult:
        """Run full detection pipeline on one BGR frame."""
        mask   = self._make_mask(bgr)
        result = DetectionResult()

        # ── Leading-edge density ──────────────────────────────────────────
        H, W = mask.shape
        rois = {
            'fwd':   mask[:H // 4,     :],          # top strip  → ahead when flying +Y
            'bck':   mask[3*H // 4:,   :],          # bottom     → ahead when flying -Y
            'right': mask[:,   3*W // 4:],           # right edge → ahead when stepping +X
            'left':  mask[:,       :W // 4],         # left  edge
        }
        result.density_fwd   = self._density(rois['fwd'])
        result.density_bck   = self._density(rois['bck'])
        result.density_left  = self._density(rois['left'])
        result.density_right = self._density(rois['right'])

        # ── Any tape visible? ─────────────────────────────────────────────
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big_cnts = [c for c in cnts if cv2.contourArea(c) >= self._min_area]
        result.line_visible = len(big_cnts) > 0
        if not result.line_visible:
            if self._debug:
                result.debug_frame = self._draw_debug(bgr, mask, result, [], [])
            return result

        # ── Hough lines — used for angle estimation and L-check ──────────
        segs = self._hough(mask)
        angles_arr = np.array([self._seg_angle(s) for s in segs]) if segs else np.array([])
        if len(angles_arr):
            result.dominant_angle = float(np.degrees(self._cluster_dominant(angles_arr)))

        # ── Corner detection via convexity defect ─────────────────────────
        # GUARD: only attempt if tape runs in ≥2 distinct directions (≥40°
        # apart).  A single straight tape band — even with wrinkles — has
        # only one angle cluster and must NOT trigger corner detection.
        if len(angles_arr) >= 2 and self._two_angle_clusters(angles_arr, min_diff_deg=40.0):
            corner_px = self._find_corner_defect(big_cnts)
            if corner_px is not None:
                result.corner_visible = True
                result.corner_px      = corner_px

        if self._debug:
            result.debug_frame = self._draw_debug(bgr, mask, result, segs, [])

        return result

    def dist_to_line(
        self,
        bgr: np.ndarray,
        direction: str,
        altitude_m: float = 3.0,
        fx: float = 906.0,
        fy: float = 906.0,
    ) -> float | None:
        """
        Metric distance (metres) from the drone nadir to the nearest yellow
        tape edge in the given direction.

        direction ∈ {'fwd','bck','left','right'}

        Uses a narrow CENTRAL STRIP in each direction to avoid picking up
        perpendicular walls (e.g. a side wall that runs through the full
        frame height must not confuse the forward distance measurement).

          · 'fwd'  — central 40% of width, top half.
                     Nearest = bottom-most yellow pixel (inner tape edge).
                     dist_m = (CY - y_nearest) × altitude / fy
          · 'bck'  — central 40% width, bottom half.
          · 'right'— central 40% of height, right half.
                     dist_m = (x_nearest - CX) × altitude / fx
          · 'left' — central 40% height, left half.

        Returns None if no yellow pixels in that strip.

        Usage — stop 50 cm before the front wall:
            d = det.dist_to_line(bgr, 'fwd', altitude_m=self._z)
            if d is not None and d < 0.50:
                stop_forward()
        """
        mask = self._make_mask(bgr)
        H, W = mask.shape
        CX, CY = W // 2, H // 2

        # Central strip boundaries.
        # fwd/bck use a narrow 20%-wide centre column so perpendicular tape arms
        # (e.g. the L-corner's horizontal arm) can't contaminate the forward
        # distance measurement.  left/right keep the 40%-height strip.
        cx0, cx1 = int(W * 0.40), int(W * 0.60)   # for fwd/bck (was 0.30/0.70)
        cy0, cy1 = int(H * 0.30), int(H * 0.70)   # for left/right

        if direction == 'fwd':
            roi = mask[:CY, cx0:cx1]               # top half, central width strip
            # Require each qualifying row to span ≥25 % of the strip width.
            # A real transverse boundary fills the strip; the longitudinal L-arm
            # (a narrow ~15 px vertical line) does not — so it is ignored.
            min_span = max(5, int((cx1 - cx0) * 0.25))
            counts   = np.count_nonzero(roi, axis=1)  # yellow px per row, shape=(CY,)
            ys       = np.where(counts >= min_span)[0]
            if len(ys) == 0:
                return None
            y_nearest = int(ys.max())              # bottom-most qualifying row
            return float((CY - y_nearest) * altitude_m / fy)

        elif direction == 'bck':
            roi = mask[CY:, cx0:cx1]               # bottom half, central strip
            min_span = max(5, int((cx1 - cx0) * 0.25))
            counts   = np.count_nonzero(roi, axis=1)
            ys       = np.where(counts >= min_span)[0]
            if len(ys) == 0:
                return None
            y_nearest = int(ys.min()) + CY
            return float((y_nearest - CY) * altitude_m / fy)

        elif direction == 'right':
            roi = mask[cy0:cy1, CX:]               # right half, central height strip
            xs  = np.where(roi > 0)[1]
            if len(xs) == 0:
                return None
            x_nearest = int(xs.min()) + CX
            return float((x_nearest - CX) * altitude_m / fx)

        elif direction == 'left':
            roi = mask[cy0:cy1, :CX]               # left half, central height strip
            xs  = np.where(roi > 0)[1]
            if len(xs) == 0:
                return None
            x_nearest = int(xs.max())
            return float((CX - x_nearest) * altitude_m / fx)

        else:
            raise ValueError(f"direction must be fwd/bck/left/right, got {direction!r}")

    def yaw_error_deg(self, bgr: np.ndarray) -> float | None:
        """
        Yaw correction (degrees) needed to fly parallel to the visible tape.

        Returns the dominant tape angle in the image.  Since the camera is
        body-fixed pointing down, image-horizontal = drone right/left axis.
        When dominant_angle = 0° the tape runs left-right in the image and
        the drone is already aligned parallel to it.

        Positive return → tape tilts CCW in image → drone must yaw CW
        Negative return → tape tilts CW  in image → drone must yaw CCW

        Returns None if no tape visible.

        Typical usage in mission:
            err = det.yaw_error_deg(frame)
            if err is not None and abs(err) > yaw_tol_deg:
                yaw_rate = -K_yaw * err   # proportional correction
        """
        mask = self._make_mask(bgr)
        segs = self._hough(mask)
        if not segs:
            return None
        angles = np.array([self._seg_angle(s) for s in segs])
        dominant = self._cluster_dominant(angles)
        return float(np.degrees(dominant))

    def tune(self, bgr: np.ndarray):
        """
        Print HSV statistics for yellow pixels in the frame.
        Use this offline to verify / adjust HSV bounds.
        """
        mask = self._make_mask(bgr)
        hsv  = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        pts  = hsv[mask > 0]
        if len(pts) == 0:
            print("tune: NO yellow pixels found — thresholds may be too tight.")
            print(f"  current low : H={self._hsv_low[0]}  S={self._hsv_low[1]}  V={self._hsv_low[2]}")
            print(f"  current high: H={self._hsv_high[0]}  S={self._hsv_high[1]}  V={self._hsv_high[2]}")
            return
        print(f"tune: {len(pts)} yellow pixels found")
        for i, ch in enumerate(['H', 'S', 'V']):
            print(f"  {ch}: min={pts[:,i].min():3d}  max={pts[:,i].max():3d}"
                  f"  mean={pts[:,i].mean():.1f}  std={pts[:,i].std():.1f}")
        print(f"  current low : H={self._hsv_low[0]}  S={self._hsv_low[1]}  V={self._hsv_low[2]}")
        print(f"  current high: H={self._hsv_high[0]}  S={self._hsv_high[1]}  V={self._hsv_high[2]}")
        # Show what fraction of the frame is masked
        print(f"  mask coverage: {100*np.count_nonzero(mask)/mask.size:.2f}% of frame")

    # ── Internal pipeline ─────────────────────────────────────────────────────

    def _make_mask(self, bgr: np.ndarray) -> np.ndarray:
        """
        BGR → binary yellow mask.
        Steps:
          1. Gaussian blur    — kill grain / sensor noise
          2. HSV conversion
          3. CLAHE on V       — normalise brightness across frame
          4. HSV threshold
          5. Morphological close  (gap fill within tape)
          6. Morphological open   (kill isolated noise blobs)
        """
        blurred = cv2.GaussianBlur(bgr, (5, 5), 0)
        hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Apply CLAHE on the V channel to handle shadows / glare
        h_ch, s_ch, v_ch = cv2.split(hsv)
        v_eq = self._clahe.apply(v_ch)
        hsv_eq = cv2.merge([h_ch, s_ch, v_eq])

        mask = cv2.inRange(hsv_eq, self._hsv_low, self._hsv_high)

        # Close: bridge gaps up to ~15px within the tape stripe
        close_k = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_k)

        # Open: remove isolated blobs smaller than ~5×5px
        open_k = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_k)

        # LINE-ONLY filter (mission_luma): keep only long / thin /
        # elongated / rectangular components — the real boundary tape.
        # Yellow patches on clothing, scraps and coloured objects are
        # erased HERE, so every downstream output (density, distance,
        # dominant angle, corner) sees only real boundary.
        if self._line_filter:
            mask = self._keep_line_components(mask)

        return mask

    def _keep_line_components(self, mask: np.ndarray) -> np.ndarray:
        """Keep only LONG, CONTINUOUS, THIN, ELONGATED, RECTANGULAR yellow
        components (the arena boundary tape); erase everything else.

        Same five tests as yellow_boundary_detector._keep_line_components
        (mission_luma), with thresholds scaled to full resolution:
          BIG         area >= min_component_area_px      (not a speckle)
          LONG        minAreaRect long side >= max(min_line_length_px,
                      min_line_length_frac x larger frame dimension)
          THIN        stroke width (area / half-perimeter)
                      <= max_line_width_px               (tape, not a blob)
          ELONGATED   length/width >= min_aspect_ratio   (strongest test:
                      real tape 15:1-30:1, patches 1:1-2.5:1)
          RECTANGULAR contour area / minAreaRect area >= min_rectangularity
        If nothing qualifies the mask comes back empty — correctly
        reporting no boundary: a scrap is not a wall."""
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return mask
        h, w = mask.shape
        min_len = max(float(self._min_len_px),
                      self._min_len_frac * max(h, w))
        keep = np.zeros_like(mask)
        for c in cnts:
            area = cv2.contourArea(c)
            if area < self._min_comp_area:            # speckle / tiny scrap
                continue
            (_, _), (rw, rh), _ = cv2.minAreaRect(c)
            length = max(rw, rh)
            width = min(rw, rh)
            if length < min_len:                      # too short
                continue
            perim = cv2.arcLength(c, True)
            stroke = area / max(0.5 * perim, 1.0)     # ~ tape thickness
            if stroke > self._max_line_width:         # too fat -> blob
                continue
            aspect = length / width if width > 1.0 else float("inf")
            rect_area = length * width
            rect_fill = area / rect_area if rect_area > 1e-6 else 1.0
            # A STRAIGHT tape run is elongated and fills its minAreaRect:
            straight_ok = (aspect >= self._min_aspect
                           and rect_fill >= self._min_rect)
            # An L-CORNER (two thin connected arms, mission_luma: "a
            # straight tape AND an L-corner both pass") is thin-stroked
            # and LONG but fills only a small part of its bounding rect,
            # so it fails the straight tests. Accept it when its area is
            # consistent with at most two thin arms — a filled patch of
            # the same bounding box has far more area than that.
            l_shape_ok = area <= 1.5 * stroke * (length + width)
            if not (straight_ok or l_shape_ok):
                continue
            cv2.drawContours(keep, [c], -1, 255, thickness=cv2.FILLED)
        return keep

    @staticmethod
    def _density(roi_mask: np.ndarray) -> float:
        if roi_mask.size == 0:
            return 0.0
        return float(np.count_nonzero(roi_mask)) / roi_mask.size

    def _hough(self, mask: np.ndarray):
        """
        Run HoughLinesP on the EDGES of the mask, not the filled blob.
        Running Hough on a solid filled rectangle (thick tape) produces
        hundreds of spurious interior lines — Canny first extracts only
        the tape boundary, giving clean line segments.
        """
        edges = cv2.Canny(mask, 50, 150)
        lines = cv2.HoughLinesP(
            edges,
            rho=1, theta=np.pi / 180,
            threshold=self._hough_thr,
            minLineLength=self._min_line_len,
            maxLineGap=self._max_line_gap,
        )
        if lines is None:
            return []
        return [l[0].tolist() for l in lines]   # list of [x1,y1,x2,y2]

    @staticmethod
    def _seg_angle(seg) -> float:
        """Segment angle in (-π/2, π/2]."""
        x1, y1, x2, y2 = seg
        a = math.atan2(y2 - y1, x2 - x1)
        while a > math.pi / 2:  a -= math.pi
        while a <= -math.pi / 2: a += math.pi
        return a

    @staticmethod
    def _cluster_dominant(angles: np.ndarray) -> float:
        """Return the most common angle cluster centre (circular mean)."""
        if len(angles) == 0:
            return 0.0
        # Use doubled angles to handle wraparound at ±π/2
        doubled = 2 * angles
        cx = float(np.mean(np.cos(doubled)))
        cy = float(np.mean(np.sin(doubled)))
        return math.atan2(cy, cx) / 2

    @staticmethod
    def _two_angle_clusters(angles: np.ndarray, min_diff_deg: float = 40.0) -> bool:
        """
        Return True if `angles` contains at least two segments whose
        orientations differ by ≥ min_diff_deg.

        Angles are normalised to [0, π) so that a segment pointing left
        and one pointing right are treated as the same orientation.
        The wrap-around at π/2 is handled so that a −89° and a +89°
        segment correctly measure as ~2° apart (both nearly vertical).
        """
        # Normalise to [0, π)
        norm = angles % np.pi
        min_diff_rad = np.radians(min_diff_deg)
        for i in range(len(norm)):
            for j in range(i + 1, len(norm)):
                diff = abs(norm[i] - norm[j])
                diff = min(diff, np.pi - diff)   # handle wrap at π/2
                if diff >= min_diff_rad:
                    return True
        return False

    def _find_corner_defect(self, big_cnts):
        """
        Find the inner corner of an L-shape using convexity defects.

        The inner corner is the deepest concavity in the mask contour.
        This approach is immune to which tape edge Hough detected — it
        works purely on the geometry of the filled mask blob.

        Returns (u, v) pixel of the inner corner, or None.
        """
        # Work on the largest contour (the main tape blob)
        contour = max(big_cnts, key=cv2.contourArea)

        # Need enough points for convexity defects
        if len(contour) < 10:
            return None

        # Smooth the contour slightly to suppress morphology bumps
        epsilon = 0.005 * cv2.arcLength(contour, True)
        contour = cv2.approxPolyDP(contour, epsilon, True)

        if len(contour) < 5:
            return None

        try:
            hull_idx = cv2.convexHull(contour, returnPoints=False)
            defects  = cv2.convexityDefects(contour, hull_idx)
        except cv2.error:
            return None

        if defects is None or len(defects) == 0:
            return None

        # Find the defect with the greatest depth — that's the inner corner
        best_depth = 0
        best_pt    = None
        for defect in defects:
            s, e, f, depth_fp = defect[0]
            depth = depth_fp / 256.0      # fixed-point → pixels
            if depth > best_depth:
                best_depth = depth
                best_pt    = tuple(contour[f][0])

        # Require a minimum indentation to confirm an L-shape
        # (straight tape with no corner has near-zero defect depth)
        min_depth_px = 20
        if best_depth < min_depth_px:
            return None

        return best_pt

    def _draw_debug(self, bgr, mask, result, segs, clusters):
        """Draw annotated debug frame."""
        dbg = bgr.copy()
        H, W = bgr.shape[:2]

        # Yellow mask overlay (semi-transparent green)
        overlay = dbg.copy()
        overlay[mask > 0] = (0, 200, 0)
        cv2.addWeighted(overlay, 0.35, dbg, 0.65, 0, dbg)

        # ROI boxes
        cv2.rectangle(dbg, (0, 0),         (W, H//4),   (255, 255, 0), 1)  # fwd
        cv2.rectangle(dbg, (0, 3*H//4),    (W, H),      (0, 255, 255), 1)  # bck
        cv2.rectangle(dbg, (0, 0),         (W//4, H),   (255, 0, 255), 1)  # left
        cv2.rectangle(dbg, (3*W//4, 0),    (W, H),      (0, 165, 255), 1)  # right

        # Hough segments
        colors = [(0, 0, 255), (0, 255, 0)]  # red = grp_a, green = grp_b
        flat_segs  = [s for cl in clusters for s in cl]
        flat_cols  = [colors[i] for i, cl in enumerate(clusters) for _ in cl]
        for seg, col in zip(flat_segs, flat_cols):
            x1, y1, x2, y2 = seg
            cv2.line(dbg, (x1, y1), (x2, y2), col, 2)
        for seg in segs:
            if seg not in flat_segs:
                x1, y1, x2, y2 = seg
                cv2.line(dbg, (x1, y1), (x2, y2), (180, 180, 180), 1)

        # Corner marker
        if result.corner_px:
            u, v = result.corner_px
            cv2.drawMarker(dbg, (u, v), (0, 0, 255),
                           cv2.MARKER_CROSS, 30, 3)
            cv2.circle(dbg, (u, v), 15, (0, 0, 255), 2)

        # HUD
        lines = [
            f"line={'YES' if result.line_visible else 'no '}  "
            f"corner={'YES' if result.corner_visible else 'no '}  "
            f"angle={result.dominant_angle:.1f}d" if result.dominant_angle else
            f"line={'YES' if result.line_visible else 'no '}  corner={'YES' if result.corner_visible else 'no '}",
            f"fwd={result.density_fwd:.3f}  bck={result.density_bck:.3f}",
            f"lft={result.density_left:.3f}  rgt={result.density_right:.3f}",
        ]
        for i, txt in enumerate(lines):
            cv2.putText(dbg, txt, (8, 22 + i * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
            cv2.putText(dbg, txt, (8, 22 + i * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        return dbg


# ── CLI: offline tuning / testing ────────────────────────────────────────────

def _cli_tune(path: str):
    """Sample HSV stats from yellow pixels. Helps set bounds."""
    bgr = cv2.imread(path)
    if bgr is None:
        print(f"Cannot read {path}")
        return
    # Resize to camera resolution for realistic pixel counts
    bgr = cv2.resize(bgr, (1280, 720))
    det = YellowDetector(debug=True)
    det.tune(bgr)


def _cli_test(path: str, save: str = "debug_out.jpg"):
    """Run detection on an image and save the debug frame."""
    bgr = cv2.imread(path)
    if bgr is None:
        print(f"Cannot read {path}")
        return
    bgr = cv2.resize(bgr, (1280, 720))
    det = YellowDetector(debug=True)

    det.tune(bgr)   # also print stats
    result = det.detect(bgr)

    print(f"\nDetection result:")
    print(f"  line_visible   : {result.line_visible}")
    print(f"  corner_visible : {result.corner_visible}")
    print(f"  corner_px      : {result.corner_px}")
    print(f"  dominant_angle : {result.dominant_angle}")
    print(f"  density fwd/bck/left/right: "
          f"{result.density_fwd:.3f} / {result.density_bck:.3f} / "
          f"{result.density_left:.3f} / {result.density_right:.3f}")

    print(f"\nDistance to tape (at 3m altitude):")
    for d in ['fwd', 'bck', 'left', 'right']:
        dist = det.dist_to_line(bgr, d)
        print(f"  {d:5s}: {f'{dist:.3f} m' if dist is not None else 'not visible'}")
    print(f"  yaw_error: {det.yaw_error_deg(bgr)}")

    if result.debug_frame is not None:
        cv2.imwrite(save, result.debug_frame)
        print(f"\nDebug frame saved → {save}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--tune", metavar="IMAGE", help="Print HSV stats for image")
    ap.add_argument("--test", metavar="IMAGE", help="Run full detection and save debug frame")
    ap.add_argument("--out",  metavar="FILE",  default="debug_out.jpg",
                    help="Output path for --test debug frame (default: debug_out.jpg)")
    args = ap.parse_args()

    if args.tune:
        _cli_tune(args.tune)
    elif args.test:
        _cli_test(args.test, args.out)
    else:
        ap.print_help()
