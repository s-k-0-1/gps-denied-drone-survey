#!/usr/bin/env python3
"""
hsv_calibrate — auto-calibrate the yellow HSV range for yellow_boundary_detector.

HOW TO USE
    ros2 launch viman_mission hsv_calibrate.launch.py
  then HOLD the drone ~1–1.5 m directly OVER the yellow line for the whole
  window (default 45 s — the longer window sees more lighting variation,
  which gives a steadier fit). Keep the line filling a good part of the frame;
  slowly drifting a few cm is fine and even helps sample more of the tape.

WHAT IT DOES
  It replicates the detector's EXACT preprocessing (proc_scale + CLAHE on V) so
  the numbers it finds are directly usable by yellow_boundary_detector. Over the
  window it collects the yellow-line pixels, then SMART-FITS a tight HSV range
  that captures the line while rejecting the background. It also scans a few
  (S,V) floors and reports each one's detection %. At the end it prints the best
  hsv_low / hsv_high and (default) writes them into mission_params.yaml, keeping
  a timestamped .bak backup. Rebuild the package afterwards to apply.

QUALITY GUARDS (why the numbers are trustworthy)
  * LINE-SHAPE GATE — per frame, only pixels belonging to an ELONGATED
    component (the tape) enter the histograms; yellowish soil, shoes or
    gloves in view are roundish and get dropped, so they cannot skew the fit.
  * GLARE GUARD — frames where "yellow" floods the image (blown exposure /
    white-balance hunt) are discarded entirely.
  * S/V EDGE MARGIN — the fitted S/V floors are loosened by sv_margin so the
    dimmer, less-saturated tape at the frame EDGE (seen during a real flight
    approach, not in this nadir hover) still detects — the same lesson the
    flight-tuned values in mission_params.yaml encode.
  * FIT-QUALITY REPORT — prints what fraction of the collected line pixels
    the fitted range keeps, and warns if it is low (lighting changed
    mid-window → re-run).
"""

import os
import re
import shutil
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def _best_effort_qos():
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=5)


class HsvCalibrate(Node):
    def __init__(self):
        super().__init__("hsv_calibrate")
        self.declare_parameter("image_topic",
                               "/camera/camera/color/image_raw")
        self.declare_parameter("proc_scale", 0.5)
        self.declare_parameter("use_clahe", True)
        self.declare_parameter("duration_s", 45.0)  # collection window [s] —
                                                     # 45 s sees more lighting
                                                     # variation = steadier fit
        self.declare_parameter("write_yaml", True)
        self.declare_parameter("params_file", os.path.expanduser(
            "~/drone_ws/src/viman_mission/config/mission_params.yaml"))
        # broad yellow PRIOR — only used to grab candidate line pixels
        self.declare_parameter("prior_h_lo", 12)
        self.declare_parameter("prior_h_hi", 55)
        self.declare_parameter("prior_s_min", 35)
        self.declare_parameter("prior_v_min", 45)
        # LINE-SHAPE GATE: keep only candidate pixels that belong to an
        # ELONGATED component (the tape) so yellowish soil / shoes / gloves
        # in frame cannot pollute the histograms.
        self.declare_parameter("line_gate", True)
        self.declare_parameter("line_gate_min_area", 150)   # proc-scale px
        self.declare_parameter("line_gate_aspect", 2.5)     # minAreaRect L/W
        # GLARE GUARD: drop frames where "yellow" floods the image (blown
        # exposure) — such frames only smear the histograms.
        self.declare_parameter("max_coverage_pct", 40.0)
        # Require a healthy sample before writing anything.
        self.declare_parameter("min_frames", 30)
        # Loosen the fitted S/V floors by this margin so dimmer tape at the
        # frame EDGE (during a real approach) still detects in flight.
        self.declare_parameter("sv_margin", 10)

        g = lambda n: self.get_parameter(n).value
        self._topic = str(g("image_topic"))
        self._scale = float(g("proc_scale"))
        self._use_clahe = bool(g("use_clahe"))
        self._duration = float(g("duration_s"))
        self._write_yaml = bool(g("write_yaml"))
        self._params_file = str(g("params_file"))
        self._prior_h = (int(g("prior_h_lo")), int(g("prior_h_hi")))
        self._prior_s = int(g("prior_s_min"))
        self._prior_v = int(g("prior_v_min"))
        self._line_gate = bool(g("line_gate"))
        self._gate_min_area = int(g("line_gate_min_area"))
        self._gate_aspect = float(g("line_gate_aspect"))
        self._max_cov = float(g("max_coverage_pct"))
        self._min_frames = int(g("min_frames"))
        self._sv_margin = int(g("sv_margin"))

        self._bridge = CvBridge()
        self._clahe = (cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
                       if self._use_clahe else None)

        # marginal histograms of candidate (yellow-prior) pixels
        self._hist_h = np.zeros(180, dtype=np.int64)
        self._hist_s = np.zeros(256, dtype=np.int64)
        self._hist_v = np.zeros(256, dtype=np.int64)
        # a small (S,V)-floor scan: [(s,v)] -> accumulated px inside prior H
        self._scan = [(40, 50), (60, 60), (60, 90),
                      (90, 60), (90, 100), (120, 120)]
        self._scan_px = [0] * len(self._scan)
        self._total_px = 0
        self._cand_px = 0
        self._frames = 0
        self._gated_frames = 0     # frames where the line-shape gate applied
        self._skipped_glare = 0    # frames dropped by the glare guard
        self._start = None
        self._done = False

        self.create_subscription(Image, self._topic, self._cb,
                                 _best_effort_qos())
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            "════════════════════════════════════════════════════════════")
        self.get_logger().info(
            "HSV CALIBRATE — hold the drone ~1–1.5 m OVER the yellow line.")
        self.get_logger().info(
            f"Collecting for {self._duration:.0f}s as soon as images arrive…")

    # ── same preprocessing as yellow_boundary_detector ────────────────
    def _preprocess(self, bgr):
        s = self._scale
        small = cv2.resize(bgr, None, fx=s, fy=s) if s != 1.0 else bgr
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        if self._clahe is not None:
            h, sc, v = cv2.split(hsv)
            hsv = cv2.merge([h, sc, self._clahe.apply(v)])
        return hsv

    def _line_pixels(self, prior):
        """Boolean mask of prior pixels that belong to ELONGATED components
        (minAreaRect aspect >= line_gate_aspect, area >= line_gate_min_area)
        — the tape, not scraps / soil patches. None if nothing qualifies
        this frame (the caller then falls back to the raw prior)."""
        mask = prior.astype(np.uint8) * 255
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        keep = None
        for c in cnts:
            area = cv2.contourArea(c)
            if area < self._gate_min_area:
                continue
            (_, _), (rw, rh), _ = cv2.minAreaRect(c)
            w, l = min(rw, rh), max(rw, rh)
            aspect = l / w if w > 1.0 else float("inf")
            if aspect < self._gate_aspect:
                continue
            if keep is None:
                keep = np.zeros_like(mask)
            cv2.drawContours(keep, [c], -1, 255, thickness=cv2.FILLED)
        if keep is None:
            return None
        return (keep > 0) & prior

    def _cb(self, msg: Image):
        if self._done:
            return
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:                       # noqa: BLE001
            self.get_logger().warn(f"cv_bridge: {e}", throttle_duration_sec=5.0)
            return
        if self._start is None:
            self._start = time.monotonic()
        hsv = self._preprocess(bgr)
        H, S, V = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
        prior = ((H >= self._prior_h[0]) & (H <= self._prior_h[1]) &
                 (S >= self._prior_s) & (V >= self._prior_v))
        n = int(prior.sum())
        # GLARE GUARD: "yellow" flooding the frame = blown exposure; the
        # frame would smear the histograms with background, so drop it.
        if 100.0 * n / H.size > self._max_cov:
            self._skipped_glare += 1
            return
        self._total_px += int(H.size)
        self._cand_px += n
        self._frames += 1
        if n == 0:
            return
        # LINE-SHAPE GATE: keep only pixels of elongated components (the
        # tape); fall back to the raw prior if nothing qualifies so no
        # frame is wasted.
        if self._line_gate:
            gated = self._line_pixels(prior)
            if gated is not None:
                prior = gated
                self._gated_frames += 1
                if not prior.any():
                    return
        hs, ss, vs = H[prior], S[prior], V[prior]
        self._hist_h += np.bincount(hs.ravel(), minlength=180)[:180]
        self._hist_s += np.bincount(ss.ravel(), minlength=256)[:256]
        self._hist_v += np.bincount(vs.ravel(), minlength=256)[:256]
        # (S,V)-floor scan inside the prior H band
        for i, (s_f, v_f) in enumerate(self._scan):
            self._scan_px[i] += int(((ss >= s_f) & (vs >= v_f)).sum())

    def _tick(self):
        if self._done:
            return
        if self._start is None:
            self.get_logger().info("Waiting for camera images…",
                                   throttle_duration_sec=3.0)
            return
        remain = self._duration - (time.monotonic() - self._start)
        if remain > 0:
            pct = 100.0 * self._cand_px / max(1, self._total_px)
            hint = ""
            if self._frames >= 10 and pct < 0.2:
                hint = "  << very little yellow — move closer / check light"
            self.get_logger().info(
                f"  … {remain:4.0f}s left | yellow ≈ {pct:5.2f}% of frame "
                f"| frames={self._frames} | line-gated={self._gated_frames}"
                f"{hint}")
            return
        self._finish()

    @staticmethod
    def _pct(hist, p):
        tot = hist.sum()
        if tot == 0:
            return None
        return int(np.searchsorted(np.cumsum(hist), p / 100.0 * tot))

    def _finish(self):
        self._done = True
        if self._hist_h.sum() < 500 or self._frames < self._min_frames:
            self.get_logger().error(
                f"Too little data ({int(self._hist_h.sum())} yellow px over "
                f"{self._frames} frames; need >=500 px and "
                f">={self._min_frames} frames) — was the drone over the "
                "line, in good light? Nothing written. Try again closer / "
                "brighter.")
            rclpy.try_shutdown()
            return

        # SMART-FIT: tight range covering ~96% of the collected line pixels.
        h_lo = max(0, (self._pct(self._hist_h, 2) or 20) - 3)
        h_hi = min(179, (self._pct(self._hist_h, 98) or 42) + 3)
        # S/V floors: 3rd percentile MINUS sv_margin, so the dimmer, less
        # saturated tape at the frame EDGE (seen during a real approach,
        # not in this nadir hover) still passes in flight.
        s_lo = max(0, (self._pct(self._hist_s, 3) or 60) - self._sv_margin)
        v_lo = max(0, (self._pct(self._hist_v, 3) or 80) - self._sv_margin)
        hsv_low = [int(h_lo), int(s_lo), int(v_lo)]
        hsv_high = [int(h_hi), 255, 255]

        # FIT QUALITY: fraction of the collected line pixels the fitted
        # range keeps (per-channel marginal approximation).
        def _inside(hist, lo, hi):
            tot = hist.sum()
            return 100.0 * hist[lo:hi + 1].sum() / tot if tot else 0.0
        q_h = _inside(self._hist_h, hsv_low[0], hsv_high[0])
        q_s = _inside(self._hist_s, hsv_low[1], 255)
        q_v = _inside(self._hist_v, hsv_low[2], 255)
        q_min = min(q_h, q_s, q_v)

        line = "─" * 60
        self.get_logger().info(line)
        self.get_logger().info(
            f"Collected {self._frames} frames, "
            f"{int(self._hist_h.sum())} yellow-line pixels "
            f"(line-shape gate on {self._gated_frames}/{self._frames} frames"
            + (f", {self._skipped_glare} glare frames dropped"
               if self._skipped_glare else "") + ").")
        self.get_logger().info(
            f"Fit keeps H {q_h:.1f}% / S {q_s:.1f}% / V {q_v:.1f}% of the "
            "collected line pixels"
            + ("" if q_min >= 90.0 else
               " — LOW: lighting likely changed mid-window; consider "
               "re-running the calibration"))
        self.get_logger().info(
            "(S,V)-floor scan — detection % of frame (H band "
            f"{self._prior_h[0]}–{self._prior_h[1]}):")
        for (s_f, v_f), px in zip(self._scan, self._scan_px):
            pct = 100.0 * px / max(1, self._total_px)
            self.get_logger().info(
                f"    S≥{s_f:3d} V≥{v_f:3d}  ->  {pct:6.3f}%")
        self.get_logger().info(line)
        self.get_logger().info("BEST (smart-fit, tight to your line):")
        self.get_logger().info(f"    hsv_low:  {hsv_low}")
        self.get_logger().info(f"    hsv_high: {hsv_high}")
        self.get_logger().info(line)

        if self._write_yaml:
            self._update_yaml(hsv_low, hsv_high)
        else:
            self.get_logger().info(
                "write_yaml=false — copy the values above into "
                "mission_params.yaml yourself.")
        rclpy.try_shutdown()

    def _update_yaml(self, low, high):
        path = self._params_file
        if not path or not os.path.isfile(path):
            self.get_logger().warn(
                f"params_file not found ({path}) — printed the values above; "
                "paste them into mission_params.yaml manually.")
            return
        try:
            with open(path, "r") as f:
                text = f.read()
            bak = f"{path}.{datetime.now().strftime('%Y%m%d_%H%M%S')}.bak"
            shutil.copy2(path, bak)

            def repl(m, vals):
                return f"{m.group(1)}[{vals[0]}, {vals[1]}, {vals[2]}]{m.group(2)}"

            new = re.sub(r"^(\s*hsv_low:\s*)\[[^\]]*\](.*)$",
                         lambda m: repl(m, low), text, count=1, flags=re.M)
            new = re.sub(r"^(\s*hsv_high:\s*)\[[^\]]*\](.*)$",
                         lambda m: repl(m, high), new, count=1, flags=re.M)
            if new == text:
                self.get_logger().warn(
                    "Could not find hsv_low/hsv_high lines to update — values "
                    "printed above; paste them in manually.")
                return
            with open(path, "w") as f:
                f.write(new)
            self.get_logger().info(f"Wrote hsv_low/hsv_high into: {path}")
            self.get_logger().info(f"Backup saved: {bak}")
            self.get_logger().info(
                "Now rebuild to apply:  colcon build --packages-select "
                "viman_mission && source install/setup.bash")
        except Exception as e:                       # noqa: BLE001
            self.get_logger().error(
                f"YAML write failed: {e} — values printed above, paste manually.")


def main():
    rclpy.init()
    node = HsvCalibrate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:                            # noqa: BLE001
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
