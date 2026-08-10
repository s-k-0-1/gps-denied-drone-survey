#!/usr/bin/env python3
"""
whycon_detector — self-contained circular-marker detector.
Team Viman Rakshak / IRoC-U 2026.

Drop-in stand-in for a WhyCon ROS node when no whycon package is available.
Detects the high-contrast circular landing marker (a dark disc on light
paper, 41 cm outer diameter) in the downward D455 colour image and publishes
its pose so `whycode_mission` can consume it unchanged.

Publishes
─────────
  /whycon/poses   geometry_msgs/PoseArray
      pose.position  marker centre in the CAMERA OPTICAL frame (metres):
                     x = right, y = down, z = depth (toward the ground)
      pose.orientation  rotation about the optical axis from the black
                        CROSS lines on the pad (squares the drone to the
                        pad). Identity if publish_yaw is false or no cross
                        is found → whycode_mission then skips yaw-align.
  /whycon/image_debug  sensor_msgs/Image (only if publish_debug:=true)

How the metric pose is computed
───────────────────────────────
  Z = f · D / d_px        (D = real diameter 0.41 m, d_px = disc diameter px)
  X = (u − cx) · Z / fx,  Y = (v − cy) · Z / fy
  Intrinsics (fx, fy, cx, cy) come from /camera/camera/color/camera_info.

Yaw caveat
──────────
  The cross is 4-fold symmetric, so this gives heading only modulo 90° — it
  squares the drone to the pad but cannot tell "front" from "side". True
  WhyCode front/back needs decoding the gear ring (not done here).

Run:
  ros2 run viman_mission whycon_detector \
      --ros-args --params-file <mission_params.yaml>
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import CameraInfo, Image


class WhyconDetector(Node):

    def __init__(self):
        super().__init__("whycon_detector")

        self.declare_parameters("", [
            ("image_topic",        "/camera/camera/color/image_raw"),
            ("camera_info_topic",  "/camera/camera/color/camera_info"),
            ("output_topic",       "/whycon/poses"),
            ("marker_diameter_m",  0.41),    # 41 cm outer disc
            ("dark_max_v",         90),      # gray < this = candidate dark pixel
            ("min_radius_px",      18),
            ("max_radius_px",      600),
            ("hough_param1",       120),     # Canny high threshold for circle edges
            ("hough_param2",       35),      # accumulator: LOWER = detect more circles
            ("min_dark_frac",      0.40),    # disc ring must be >= this fraction dark
            ("min_center_contrast", 25.0),   # centre must be >= this much brighter than
                                             # the ring (the white WhyCon centre) — this
                                             # rejects solid dark blobs / clutter
            ("proc_scale",         0.5),     # detect at this scale for speed (0.5=half)
            ("publish_yaw",        True),    # cross-line heading (mod 90°)
            ("publish_debug",      False),   # annotated /whycon/image_debug topic
            ("debug_save_path",    ""),      # headless: write latest annotated
                                             # JPEG here every log_period_s
                                             # (e.g. /tmp/whycon_debug.jpg)
            ("log_period_s",       2.0),
        ])

        def gp(n): return self.get_parameter(n).value
        self._D            = float(gp("marker_diameter_m"))
        self._dark_max     = int(gp("dark_max_v"))
        self._min_r        = int(gp("min_radius_px"))
        self._max_r        = int(gp("max_radius_px"))
        self._hp1          = int(gp("hough_param1"))
        self._hp2          = int(gp("hough_param2"))
        self._min_dark     = float(gp("min_dark_frac"))
        self._min_contrast = float(gp("min_center_contrast"))
        self._proc_scale   = float(gp("proc_scale"))
        self._publish_yaw  = bool(gp("publish_yaw"))
        self._publish_dbg  = bool(gp("publish_debug"))
        self._save_path    = str(gp("debug_save_path"))
        self._log_period   = float(gp("log_period_s"))
        self._diag         = "startup"   # last-frame reason, for headless tuning

        self._bridge = CvBridge()
        self._fx = self._fy = self._cx = self._cy = None

        self.create_subscription(
            CameraInfo, gp("camera_info_topic"), self._cb_info, 10)
        self.create_subscription(
            Image, gp("image_topic"), self._cb_image, 10)
        self._pub = self.create_publisher(PoseArray, gp("output_topic"), 10)
        self._dbg_pub = (self.create_publisher(Image, "/whycon/image_debug", 1)
                         if self._publish_dbg else None)

        self._frames = 0
        self._hits   = 0
        self._t0     = self.get_clock().now()

        self.get_logger().info(
            f"whycon_detector up — marker {self._D:.2f} m | "
            f"in '{gp('image_topic')}' → out '{gp('output_topic')}' | "
            f"yaw={'on' if self._publish_yaw else 'off'}")

    # ── Camera intrinsics ────────────────────────────────────────
    def _cb_info(self, msg: CameraInfo):
        if self._fx is None:
            self._fx, self._fy = msg.k[0], msg.k[4]
            self._cx, self._cy = msg.k[2], msg.k[5]
            self.get_logger().info(
                f"Intrinsics: fx={self._fx:.1f} fy={self._fy:.1f} "
                f"cx={self._cx:.1f} cy={self._cy:.1f}")

    # ── Frame processing ─────────────────────────────────────────
    def _cb_image(self, msg: Image):
        if self._fx is None:
            return
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge: {e}", throttle_duration_sec=5.0)
            return

        self._frames += 1
        det = self._detect(bgr)

        out = PoseArray()
        out.header = msg.header              # keep camera frame + stamp
        if det is not None:
            (u, v, r, yaw) = det
            self._hits += 1
            z = self._fx * self._D / (2.0 * r)
            x = (u - self._cx) * z / self._fx
            y = (v - self._cy) * z / self._fy
            p = Pose()
            p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
            if self._publish_yaw and yaw is not None:
                p.orientation.z = math.sin(yaw / 2.0)
                p.orientation.w = math.cos(yaw / 2.0)
            else:
                p.orientation.w = 1.0
            out.poses.append(p)
            if self._dbg_pub is not None:
                self._draw_debug(bgr, u, v, r, yaw)

        self._pub.publish(out)             # publish even when empty (= "lost")

        # periodic stats + headless frame dump
        dt = (self.get_clock().now() - self._t0).nanoseconds / 1e9
        if dt >= self._log_period:
            rate = self._hits / max(1, self._frames)
            self.get_logger().info(
                f"detect {100*rate:.0f}% ({self._hits}/{self._frames})  | {self._diag}")
            if self._save_path:
                self._save_frame(bgr, det)
            self._frames = self._hits = 0
            self._t0 = self.get_clock().now()

    def _save_frame(self, bgr, det):
        """Write the latest annotated frame to disk (headless tuning aid)."""
        try:
            img = bgr.copy()
            if det is not None:
                u, v, r, yaw = det
                cv2.circle(img, (int(u), int(v)), int(r), (0, 255, 0), 3)
                cv2.drawMarker(img, (int(u), int(v)), (0, 0, 255),
                               cv2.MARKER_CROSS, 30, 2)
                if yaw is not None:
                    dx, dy = math.cos(yaw) * r, math.sin(yaw) * r
                    cv2.line(img, (int(u - dx), int(v - dy)),
                             (int(u + dx), int(v + dy)), (255, 0, 0), 2)
            cv2.putText(img, self._diag, (8, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
            cv2.putText(img, self._diag, (8, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
            cv2.imwrite(self._save_path, img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        except Exception as e:
            self.get_logger().warn(f"debug save failed: {e}",
                                   throttle_duration_sec=5.0)

    def _detect(self, bgr):
        """Return (u, v, radius_px, yaw_rad|None) for the marker, or None.
        Records a human-readable reason in self._diag for headless tuning.

        Method: HoughCircles finds circular edges (robust to the marker's ring
        shape and to dark clutter merging), then keep the circle whose annulus
        is actually dark — that's the marker, and it rejects the bright inner
        circle and any spurious circles automatically."""
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        s = self._proc_scale
        small = cv2.resize(gray, None, fx=s, fy=s) if s != 1.0 else gray
        circles = cv2.HoughCircles(
            small, cv2.HOUGH_GRADIENT, dp=1.2,
            minDist=max(20, min(small.shape) // 4),
            param1=self._hp1, param2=self._hp2,
            minRadius=int(self._min_r * s), maxRadius=int(self._max_r * s))

        if circles is None:
            self._diag = (f"no circles found "
                          f"(lower hough_param2 below {self._hp2} to loosen)")
            return None

        best = None
        best_score = -1.0
        seen_dark = 0.0
        seen_con  = -999.0
        n = len(circles[0])
        for cx, cy, r in circles[0]:
            u, v, rr = cx / s, cy / s, r / s
            ring_dark, contrast = self._marker_metrics(gray, u, v, rr)
            seen_dark = max(seen_dark, ring_dark)
            seen_con  = max(seen_con, contrast)
            if ring_dark >= self._min_dark and contrast >= self._min_contrast:
                score = ring_dark + contrast / 255.0   # prefer dark ring + bright centre
                if score > best_score:
                    best_score = score
                    best = (u, v, rr, ring_dark, contrast)

        if best is None:
            self._diag = (f"{n} circle(s), none are the marker "
                          f"(best ring_dark={seen_dark:.2f}/{self._min_dark:.2f} "
                          f"centre_contrast={seen_con:+.0f}/{self._min_contrast:.0f} "
                          f"— need a DARK ring around a BRIGHT centre)")
            return None

        u, v, r, ring_dark, contrast = best
        yaw = self._cross_yaw(gray, u, v, r) if self._publish_yaw else None
        self._diag = (f"OK u={u:.0f} v={v:.0f} r={r:.0f}px "
                      f"ring={ring_dark:.2f} contrast={contrast:+.0f}"
                      + (f" yaw={math.degrees(yaw):+.0f}°" if yaw is not None
                         else " yaw=none(no cross)"))
        return u, v, r, yaw

    def _marker_metrics(self, gray, u, v, r):
        """Return (ring_dark_frac, centre_minus_ring_brightness).

        The WhyCon marker is a DARK ring around a BRIGHT centre, so a real
        marker has (a) a mostly-dark annulus and (b) a centre clearly brighter
        than that annulus. A solid dark blob (rock, shadow, shoe) fails (b),
        which is what rejects the false detections."""
        h, w = gray.shape
        ann = np.zeros((h, w), np.uint8)
        cv2.circle(ann, (int(u), int(v)), max(1, int(0.92 * r)), 255, -1)
        cv2.circle(ann, (int(u), int(v)), max(1, int(0.45 * r)), 0, -1)
        avals = gray[ann > 0]
        cen = np.zeros((h, w), np.uint8)
        cv2.circle(cen, (int(u), int(v)), max(1, int(0.32 * r)), 255, -1)
        cvals = gray[cen > 0]
        if avals.size == 0 or cvals.size == 0:
            return 0.0, -999.0
        ring_dark = float(np.count_nonzero(avals < self._dark_max)) / avals.size
        contrast  = float(cvals.mean() - avals.mean())   # bright centre → positive
        return ring_dark, contrast

    def _cross_yaw(self, gray, u, v, r):
        """Dominant cross-line angle inside the disc → yaw (rad), mod 90°.
        Returns None if no clear line is found."""
        x0 = max(0, int(u - r)); x1 = min(gray.shape[1], int(u + r))
        y0 = max(0, int(v - r)); y1 = min(gray.shape[0], int(v + r))
        roi = gray[y0:y1, x0:x1]
        if roi.size == 0:
            return None
        edges = cv2.Canny(roi, 60, 160)
        lines = cv2.HoughLines(edges, 1, np.pi / 180,
                               threshold=max(12, int(0.45 * r)))
        if lines is None:
            return None
        # HoughLines theta is the normal angle; line direction = theta - 90°.
        # Reduce to (-45°, 45°] (cross is 4-fold symmetric) and circular-mean.
        ang = []
        for l in lines[:40]:
            theta = l[0][1]
            d = (theta - math.pi / 2.0)        # line direction
            d = (d + math.pi / 4.0) % (math.pi / 2.0) - math.pi / 4.0
            ang.append(d)
        a = np.array(ang)
        mean = math.atan2(np.mean(np.sin(4 * a)), np.mean(np.cos(4 * a))) / 4.0
        return mean

    def _draw_debug(self, bgr, u, v, r, yaw):
        try:
            dbg = bgr.copy()
            cv2.circle(dbg, (int(u), int(v)), int(r), (0, 255, 0), 2)
            cv2.drawMarker(dbg, (int(u), int(v)), (0, 0, 255),
                           cv2.MARKER_CROSS, 24, 2)
            if yaw is not None:
                dx, dy = math.cos(yaw) * r, math.sin(yaw) * r
                cv2.line(dbg, (int(u - dx), int(v - dy)),
                         (int(u + dx), int(v + dy)), (255, 0, 0), 2)
            self._dbg_pub.publish(self._bridge.cv2_to_imgmsg(dbg, "bgr8"))
        except Exception:
            pass


def main():
    rclpy.init()
    node = WhyconDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
