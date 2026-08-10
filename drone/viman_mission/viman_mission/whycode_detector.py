#!/usr/bin/env python3
"""
whycode_detector — circular-marker detector that publishes the message the
missions consume.  Team Viman Rakshak / IRoC-U 2026.

Why this exists
───────────────
`survey_mission` (precision landing) and `whycode_mission` subscribe to
`whycode_interfaces/MarkerArray` on `/whycode_node/markers`.  The older
`whycon_detector` node publishes a *different* type (`geometry_msgs/PoseArray`)
on a *different* topic (`/whycon/poses`) with a *different* axis convention,
so it cannot drive those missions.  This node closes that gap: it runs the
same proven circle detection and publishes the marker pose in the WhyCode
message + frame the missions expect.

Detection (identical maths to whycon_detector)
──────────────────────────────────────────────
  HoughCircles finds circular edges; keep the circle whose annulus is dark
  around a bright centre (the WhyCon/WhyCode marker).  Metric pose by pinhole:
      Z = fx · D / d_px        (D = 0.41 m outer diameter, d_px = disc px)
      X = (u − cx) · Z / fx,   Y = (v − cy) · Z / fy
  Intrinsics from /camera/camera/color/camera_info.

Frame published (WhyCode convention — matches the mission consumers)
───────────────────────────────────────────────────────────────────
  Optical frame here is X=right, Y=down, Z=depth(+toward ground).  The
  mission reads each marker as  depth=position.x, right=−position.y,
  down=−position.z, i.e. it expects x=depth, y=left, z=up.  So we map:
      position.x =  Z      (depth)
      position.y = −X      (left)
      position.z = −Y      (up)

Run:
  ros2 run viman_mission whycode_detector \
      --ros-args --params-file <mission_params.yaml>
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from whycode_interfaces.msg import Marker, MarkerArray

from viman_mission.common import qos_best_effort


class WhycodeDetector(Node):

    def __init__(self):
        super().__init__("whycode_detector")

        self.declare_parameters("", [
            ("image_topic",         "/camera/camera/color/image_raw"),
            ("camera_info_topic",   "/camera/camera/color/camera_info"),
            ("output_topic",        "/whycode_node/markers"),
            ("marker_diameter_m",   0.41),    # 41 cm outer disc
            ("dark_max_v",          115),     # gray < this = candidate dark pixel
                                              # (raise if a grey — not black — disc is missed)
            ("min_radius_px",       18),
            ("max_radius_px",       600),
            ("hough_param1",        120),     # Canny high threshold
            ("hough_param2",        25),      # accumulator: LOWER = more circles
            ("min_dark_frac",       0.55),    # annulus must be >= this fraction dark
            ("min_circularity",     0.45),    # contour area / enclosing-circle area
            ("min_center_contrast", 4.0),     # centre brighter than ring by >= this
            ("clahe_clip",          3.0),     # CLAHE contrast boost for low light (0=off).
                                              # Normalises local contrast so a dim cog still
                                              # reads as a bright centre at night.
            ("open_kernel_px",      9),       # MORPH_OPEN size (at proc_scale) to erase
                                              # thin cage rails before contouring; 0=off
            ("center_bias",         0.6),     # prefer blobs near image centre (the marker
                                              # is ~below the drone); 0 disables
            ("proc_scale",          0.5),     # detect at this scale for speed
            ("process_below_alt_m", 2.5),     # only run detection when drone altitude is
                                              # below this (saves CPU/VIO fps during the
                                              # survey at cruise alt); <=0 = always run
            ("pose_topic",          "/mavros/local_position/pose"),
            ("publish_debug",       False),
            ("debug_save_path",     ""),       # headless: write latest annotated
                                               # JPEG here every log_period_s
                                               # (e.g. /tmp/whycode_debug.jpg)
            ("log_period_s",        2.0),
        ])

        def gp(n): return self.get_parameter(n).value
        self._D            = float(gp("marker_diameter_m"))
        self._dark_max     = int(gp("dark_max_v"))
        self._min_r        = int(gp("min_radius_px"))
        self._max_r        = int(gp("max_radius_px"))
        self._hp1          = int(gp("hough_param1"))
        self._hp2          = int(gp("hough_param2"))
        self._min_dark     = float(gp("min_dark_frac"))
        self._min_circ     = float(gp("min_circularity"))
        self._min_contrast = float(gp("min_center_contrast"))
        self._open_k       = int(gp("open_kernel_px"))
        self._center_bias  = float(gp("center_bias"))
        _clahe_clip        = float(gp("clahe_clip"))
        self._clahe        = (cv2.createCLAHE(clipLimit=_clahe_clip,
                                              tileGridSize=(8, 8))
                              if _clahe_clip > 0.0 else None)
        self._proc_scale   = float(gp("proc_scale"))
        self._below_alt    = float(gp("process_below_alt_m"))
        self._publish_dbg  = bool(gp("publish_debug"))
        self._save_path    = str(gp("debug_save_path"))
        self._log_period   = float(gp("log_period_s"))
        self._diag         = "startup"
        self._last_det     = None     # last (u,v,r) for headless debug dump
        self._alt          = 0.0      # latest drone altitude (m); 0 → process (safe)
        # Mission-phase gate: the director publishes "<command>|<rtabQ>" on
        # /viman/mission/status every tick. Detection only runs during the
        # MARKER_* landing phases, so the whole survey cruise (same 3 m altitude)
        # costs ZERO detection CPU — that headroom goes to RTAB/VIO. If no
        # mission status is ever seen (detector run standalone) we fall back to
        # the altitude gate below, so behaviour is unchanged there.
        self._mission_cmd  = ""
        self._status_seen  = False

        self._bridge = CvBridge()
        self._fx = self._fy = self._cx = self._cy = None

        self.create_subscription(
            CameraInfo, gp("camera_info_topic"), self._cb_info, 10)
        self.create_subscription(
            Image, gp("image_topic"), self._cb_image, 10)
        self.create_subscription(
            PoseStamped, gp("pose_topic"), self._cb_pose, qos_best_effort())
        self.create_subscription(
            String, "/viman/mission/status", self._cb_status, 10)
        self._pub = self.create_publisher(MarkerArray, gp("output_topic"), 10)
        self._dbg_pub = (self.create_publisher(Image, "/whycode_node/image_debug", 1)
                         if self._publish_dbg else None)

        self._frames = 0
        self._hits   = 0
        self._t0     = self.get_clock().now()

        alt_gate = (f"alt<{self._below_alt:.1f} m" if self._below_alt > 0 else "always")
        self.get_logger().info(
            f"whycode_detector up — marker {self._D:.2f} m | "
            f"in '{gp('image_topic')}' → out '{gp('output_topic')}' | "
            f"detects ONLY in MARKER_* landing phases (after RETURN); "
            f"standalone fallback: {alt_gate}")

    def _cb_pose(self, msg: PoseStamped):
        self._alt = msg.pose.position.z

    def _cb_status(self, msg: String):
        # "<command>|<rtabQ>" from the director; keep just the command.
        self._mission_cmd = (msg.data or "").rsplit("|", 1)[0].strip()
        self._status_seen = True

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
        # PHASE gate (primary): once the mission is publishing its status, only
        # detect during the MARKER_* landing phases. Survey cruise is the SAME
        # 3 m altitude as the landing search, so an altitude gate alone can't
        # tell them apart — this makes detection idle for the entire survey and
        # only wake up after RETURN, exactly when the marker search begins.
        if self._status_seen:
            if not self._mission_cmd.startswith("MARKER"):
                return
        # Altitude gate (fallback when run standalone, no mission status): skip
        # detection above the threshold so cruise keeps full CPU for VIO.
        elif self._below_alt > 0.0 and self._alt > self._below_alt:
            return
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge: {e}", throttle_duration_sec=5.0)
            return

        self._frames += 1
        det = self._detect(bgr)
        self._last_det = det

        out = MarkerArray()
        out.header = msg.header              # keep camera frame + stamp
        if det is not None:
            (u, v, r) = det
            self._hits += 1
            # pinhole metric pose in the OPTICAL frame (x=right, y=down, z=depth)
            z = self._fx * self._D / (2.0 * r)
            x = (u - self._cx) * z / self._fx
            y = (v - self._cy) * z / self._fy

            mk = Marker()
            p = Pose()
            # convert optical → WhyCode frame (x=depth, y=left, z=up)
            p.position.x = float(z)      # depth
            p.position.y = float(-x)     # left
            p.position.z = float(-y)     # up
            p.orientation.w = 1.0
            mk.position = p
            # NOTE: rotation (WhyCode yaw) is left at default — the survey
            # landing only uses position. If you later want whycode_mission's
            # YAW_ALIGN to work, decode the ring and fill mk.rotation here.
            out.markers.append(mk)
            if self._dbg_pub is not None:
                self._draw_debug(bgr, u, v, r)

        self._pub.publish(out)             # publish even when empty (= "lost")

        dt = (self.get_clock().now() - self._t0).nanoseconds / 1e9
        if dt >= self._log_period:
            rate = self._hits / max(1, self._frames)
            self.get_logger().info(
                f"detect {100*rate:.0f}% ({self._hits}/{self._frames})  | {self._diag}")
            if self._save_path:
                self._save_frame(bgr)
            self._frames = self._hits = 0
            self._t0 = self.get_clock().now()

    # ── Detection (same approach as whycon_detector) ─────────────
    def _detect(self, bgr):
        """Return (u, v, radius_px) for the marker, or None.

        Contour-based: the WhyCon/WhyCode marker is a SOLID DARK DISC with a
        bright centre.  We threshold dark pixels, find filled circular blobs,
        keep the one that is (a) round (area / enclosing-circle area), (b) dark
        in its annulus, and (c) bright in its centre.  This locks the disc and
        ignores box edges / frame rails that fooled HoughCircles.
        """
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        # Low-light contrast normalisation: lifts the dim cog above the dark ring
        # so contrast/ring metrics survive at night. Applied before blur so the
        # disc edge stays crisp for the contour fit.
        if self._clahe is not None:
            gray = self._clahe.apply(gray)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        s = self._proc_scale
        small = cv2.resize(gray, None, fx=s, fy=s) if s != 1.0 else gray

        # Dark blobs → white.
        mask = (small < self._dark_max).astype(np.uint8) * 255
        # OPEN first: erase thin dark cage rails / shadows that would otherwise
        # connect the disc to off-frame clutter and balloon the contour.
        if self._open_k >= 3:
            ko = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (self._open_k, self._open_k))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, ko)
        # CLOSE: smooth the disc edge and bridge small gaps.
        kc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kc, iterations=2)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self._diag = "no dark blobs (raise dark_max_v?)"
            return None

        img_cx = gray.shape[1] * 0.5
        img_cy = gray.shape[0] * 0.5
        half_diag = math.hypot(img_cx, img_cy)

        best = None
        best_score = -1.0
        cand = None            # strongest near-miss, for the diagnostic
        cand_score = -1e9
        n_eval = 0
        for c in contours:
            (cx, cy), rr = cv2.minEnclosingCircle(c)
            u, v, r = cx / s, cy / s, rr / s
            if r < self._min_r or r > self._max_r:
                continue
            area = cv2.contourArea(c)
            circ = area / (math.pi * rr * rr) if rr > 0 else 0.0
            n_eval += 1
            ring_dark, contrast = self._marker_metrics(gray, u, v, r)
            # prefer central blobs: subtract a penalty growing with distance
            # from image centre (the marker sits ~below the drone)
            dist = math.hypot(u - img_cx, v - img_cy) / half_diag
            score = circ + ring_dark + contrast / 255.0 - self._center_bias * dist
            if score > cand_score:
                cand_score = score
                cand = (u, v, r, circ, ring_dark, contrast)
            if (circ >= self._min_circ and ring_dark >= self._min_dark
                    and contrast >= self._min_contrast):
                if score > best_score:
                    best_score = score
                    best = (u, v, r)

        if best is None:
            if cand is None:
                self._diag = "blobs found, none in radius range"
                return None
            cu, cv, cr, cc, cd, ct = cand
            self._diag = (f"{n_eval} blob(s), none match — best @ "
                          f"u={cu:.0f} v={cv:.0f} r={cr:.0f}px  "
                          f"circ={cc:.2f}/{self._min_circ:.2f}  "
                          f"ring_dark={cd:.2f}/{self._min_dark:.2f}  "
                          f"contrast={ct:+.0f}/{self._min_contrast:.0f}")
            return None

        u, v, r = best
        self._diag = f"OK u={u:.0f} v={v:.0f} r={r:.0f}px"
        return u, v, r

    def _marker_metrics(self, gray, u, v, r):
        """Return (ring_dark_frac, centre_minus_ring_brightness)."""
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
        contrast  = float(cvals.mean() - avals.mean())
        return ring_dark, contrast

    def _save_frame(self, bgr):
        """Write the latest annotated frame to disk (headless tuning aid)."""
        try:
            img = bgr.copy()
            if self._last_det is not None:
                u, v, r = self._last_det
                cv2.circle(img, (int(u), int(v)), int(r), (0, 255, 0), 3)
                cv2.drawMarker(img, (int(u), int(v)), (0, 0, 255),
                               cv2.MARKER_CROSS, 30, 2)
            cv2.putText(img, self._diag, (8, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
            cv2.putText(img, self._diag, (8, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.imwrite(self._save_path, img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        except Exception as e:
            self.get_logger().warn(f"debug save failed: {e}",
                                   throttle_duration_sec=5.0)

    def _draw_debug(self, bgr, u, v, r):
        try:
            dbg = bgr.copy()
            cv2.circle(dbg, (int(u), int(v)), int(r), (0, 255, 0), 2)
            cv2.drawMarker(dbg, (int(u), int(v)), (0, 0, 255),
                           cv2.MARKER_CROSS, 24, 2)
            self._dbg_pub.publish(self._bridge.cv2_to_imgmsg(dbg, "bgr8"))
        except Exception:
            pass


def main():
    rclpy.init()
    node = WhycodeDetector()
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
