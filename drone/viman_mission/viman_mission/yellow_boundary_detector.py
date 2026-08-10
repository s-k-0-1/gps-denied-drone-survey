#!/usr/bin/env python3
"""
yellow_boundary_detector — potential-field yellow boundary detector.
Team Viman Rakshak / IRoC-U 2026.

Detects yellow boundary lines of ANY width or shape in the downward
camera stream. No Hough lines, no min-length, no min-width — just the
raw yellow mask. Every yellow pixel creates a small repulsion vector
pointing AWAY from that pixel (toward the drone's nadir); the closer
the pixel, the stronger its push. All the small vectors sum into ONE
resultant repulsion vector — the "red arrow" — telling the drone which
way to move to stay inside the boundary.

Published topics (every processed frame, even with no yellow —
subscribers use them as an aliveness signal):

  /viman/boundary/repulsion   geometry_msgs/Vector3Stamped
      Body-FLU frame (x = forward, y = left, z unused = 0).
      Direction = resultant push-away direction.
      Magnitude = strength ∈ [0, 1]:  (1 − nearest/influence_radius)².
      Zero vector when no yellow in view.

  /viman/boundary/nearest_m   std_msgs/Float32
      Metric ground distance (m) from the drone's nadir to the CLOSEST
      yellow pixel.  -1.0 = no yellow visible.

  /viman/boundary/coverage_pct  std_msgs/Float32
      Percent of the frame that is yellow (0–100) — "how much line
      do I see". Used by the mission/guard status tables.

  /viman/boundary/corner      geometry_msgs/Vector3Stamped
      L-CORNER detection: tape running in ≥2 distinct directions with
      an inner-corner point (convexity defect — same proven method as
      yellow_detector). vector.x/y = corner position in body-FLU
      metres, vector.z = 1.0 corner visible / 0.0 not.
      NOTE the potential field needs no special corner handling — the
      pixels of BOTH arms push simultaneously, so the resultant
      bisects the corner and the nearest-distance stop applies to
      whichever arm is closest. This topic is for awareness/logging.

Terminal STATUS TABLE (1 Hz):
    YELLOW%  DETECT%  NEAR(m)  PUSH-X  PUSH-Y  WEIGHT  PX  FPS
YELLOW% = frame coverage, DETECT% = frames with yellow this period,
PUSH-X/Y = repulsion vector (body FLU), WEIGHT = its magnitude [0-1].

Live viewing, three ways (same annotated feed: yellow mask tint,
BLUE arrow = toward nearest yellow, RED arrow = resultant repulsion):
  1. BROWSER (easiest, works over WiFi, nothing to install):
     mjpeg_port (default 8080) serves an MJPEG stream — open
     http://<jetson-ip>:8080 on your laptop/phone.
  2. rqt_image_view on /viman/boundary/image_debug (needs ROS 2 on
     the viewing machine).
  3. show_window:=true — OpenCV window; ONLY when a display exists
     (monitor on the Jetson, or `ssh -X`). Auto-disables when headless
     — a plain SSH session has no display, and OpenCV's Qt backend
     would otherwise abort the process.

Camera note: this node does NOT open the camera itself — it subscribes
to the image topic of whatever camera driver is already running.
NEVER run two RealSense drivers at once (the second one fails with
"failed to set power state" / "No device connected").

Geometry (downward nadir camera, "image top = drone nose"):
  image +u (right)  = body RIGHT  = body −Y
  image +v (down)   = body BACK   = body −X
  metres/pixel      = altitude / fx        (flat-ground pinhole)
  If the red arrow points the WRONG way in flight, fix it with
  cam_yaw_offset_deg (90 / 180 / 270). No code change needed.

FOV reality check: half-vertical-FOV ahead ≈ (img_h/2)/fx · alt
(≈1.1–1.2 m ahead at 3 m altitude on the D455) — approach speeds must
stay slow (≤0.3 m/s autonomous, ≤0.5 m/s manual). Physics, not a
tunable.

Run standalone:
  ros2 run viman_mission yellow_boundary_detector \
      --ros-args --params-file <mission_params.yaml>
"""

import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32, Float32MultiArray, String

from viman_mission.common import qos_best_effort

# Same field-proven yellow bounds as yellow_detector.py (OpenCV HSV).
DEFAULT_HSV_LOW  = [20, 65, 100]
DEFAULT_HSV_HIGH = [42, 255, 255]

WINDOW_NAME = "viman yellow boundary"


class YellowBoundaryDetector(Node):

    def __init__(self):
        super().__init__("yellow_boundary_detector")

        self.declare_parameters("", [
            ("image_topic",        "/camera/camera/color/image_raw"),
            ("camera_info_topic",  "/camera/camera/color/camera_info"),
            ("pose_topic",         "/mavros/local_position/pose"),
            ("repulsion_topic",    "/viman/boundary/repulsion"),
            ("nearest_topic",      "/viman/boundary/nearest_m"),
            ("coverage_topic",     "/viman/boundary/coverage_pct"),
            ("corner_topic",       "/viman/boundary/corner"),
            # --- yellow segmentation (ANY width — mask only) ---
            ("hsv_low",            DEFAULT_HSV_LOW),   # H,S,V lower bound
            ("hsv_high",           DEFAULT_HSV_HIGH),  # H,S,V upper bound
            ("use_clahe",          True),    # V-channel normalisation (shadows)
            ("min_pixels",         120),     # fewer yellow px than this (at
                                             # proc scale) = noise → "no yellow"
                                             # (raised from 40 — a real boundary
                                             # in view is ALWAYS several hundred px)
            ("open_kernel_px",     3),       # small MORPH_OPEN kills speckle
                                             # without erasing thin lines (0=off)
            # --- LINE-ONLY FILTER: keep long CONTINUOUS yellow boundary lines,
            #     reject small yellow PATCHES (clothing, tape scraps, stray
            #     objects). Runs on the proc-scale mask so EVERYTHING downstream
            #     (repulsion, nearest, coverage, lines, corner) sees only real
            #     boundary.  A component survives only if it is ALL of:
            #       BIG            — area >= min_component_area_px
            #       LONG           — length spans a big fraction of the frame
            #       THIN           — stroke width <= max_line_width_px
            #       ELONGATED      — length/width >= min_aspect_ratio  (NEW)
            #       RECTANGULAR    — fills its minAreaRect              (NEW)
            #     The last two kill the clothing / scrap false positives seen
            #     in flight footage: a shirt patch is BIG + LONG-ish + wide
            #     (fails THIN + ELONGATED); a random scrap is roundish
            #     (fails ELONGATED); real straight tape passes all five.
            ("line_filter_enabled",   True),
            ("close_kernel_px",       7),    # bridge small gaps ALONG the tape
                                             # first so a slightly broken line
                                             # stays ONE long component (0=off).
                                             # Kept small so it won't merge two
                                             # separate clothing patches.
            ("min_component_area_px", 150),  # drop components smaller than this
                                             # (speckle / tiny scraps), proc px.
                                             # Balanced: kills 80-100 px patches
                                             # while keeping thin partial-view
                                             # boundary lines that only occupy
                                             # a fraction of the frame edge.
            ("min_line_length_frac",  0.30), # a real boundary's LONG side must
                                             # span at least this fraction of the
                                             # larger image dimension. Lowered
                                             # from 0.55 — a boundary line just
                                             # entering the frame from the SIDE
                                             # during strafe is a partial line
                                             # and cannot span most of the frame
                                             # (flight log: filter rejected the
                                             # LEFT line during ACQ_LEFT because
                                             # it was only partially in view).
            ("min_line_length_px",    100),  # absolute floor for the above
                                             # (proc px), for small frames.
            ("max_line_width_px",     40),   # strokes THICKER than this are
                                             # blobs/objects, not tape (proc px).
                                             # Tape at 2-3 m altitude is
                                             # ~15-30 proc px wide; 40 leaves
                                             # margin without letting clothing
                                             # patches (typically 40-80 px thick)
                                             # through.
            ("min_aspect_ratio",      3.0),  # length/width of the component's
                                             # minAreaRect.  A real yellow tape
                                             # is extremely elongated (15:1 to
                                             # 30:1 fully in view; ~4-6:1 for a
                                             # partial view coming in from the
                                             # side).  Clothing / scrap patches
                                             # are 1:1 to 2.5:1, so 3.0 still
                                             # rejects them.  This is the single
                                             # strongest discriminator.
            ("min_rectangularity",    0.40), # contour area / minAreaRect area.
                                             # A straight tape run fills ~0.7-0.9
                                             # of its bounding rect; a partial
                                             # curved section can drop to ~0.45.
                                             # Irregular clothing blobs are
                                             # ~0.3, so 0.40 rejects them.
            # --- potential field ---
            ("influence_radius_m", 2.0),     # pixels farther than this push 0
            ("falloff_power",      2.0),     # weight = (1 - r/R)^power
            # --- corner detection (L of two tape arms) ---
            ("corner_enabled",     True),
            ("corner_min_diff_deg", 40.0),   # two line directions must differ
                                             # by at least this to be an L
            ("corner_min_defect_px", 10),    # min inner-corner depth at proc
                                             # scale (rejects wrinkles)
            # --- per-line detection (front + side lines at a corner) ---
            ("max_lines",          2),       # detect up to this many distinct
                                             # yellow lines (front + side).
                                             # 1 = old single-line behaviour.
            ("lines_topic",        "/viman/boundary/lines"),
            ("line_hough_thresh",  40),      # HoughLinesP accumulator votes
            ("line_min_len_px",    30),      # shortest segment kept (proc px)
            ("line_max_gap_px",    20),      # max gap to join a segment
            # --- processing ---
            ("proc_scale",         0.5),     # detect at half-res (Jetson)
            ("max_rate_hz",        15.0),    # cap processing rate (CPU budget;
                                             # VIO shares this machine)
            ("min_alt_m",          0.5),     # below this altitude publish
                                             # zero/-1 (ground clutter, prop
                                             # wash — measurements meaningless)
            ("default_alt_m",      3.0),     # used when no pose yet (bench)
            # --- mounting correction ---
            ("cam_yaw_offset_deg", 0.0),     # rotate body-frame output; set
                                             # 90/180/270 if arrow is wrong way
            # --- camera-tilt (pitch/roll) compensation ---
            # A downward camera reads a WRONG line distance when the drone
            # tilts: pitching nose-down to fly FORWARD makes a line ahead
            # look closer, so the mission/guard flees BACKWARD from it. This
            # shifts the effective nadir by the tilt so distances stay true.
            ("tilt_comp_enabled",  True),
            ("tilt_pitch_sign",   -1.0),     # BENCH-VERIFY: hold drone at fixed
                                             # height over the tape, tilt nose
                                             # DOWN — NEAR should stay ~constant.
                                             # If NEAR still drops, flip to +1.0.
            ("tilt_roll_sign",     1.0),     # same idea for roll (side line):
                                             # roll right, NEAR should hold.
            ("tilt_max_deg",       25.0),    # clamp — ignore extreme attitudes
            # --- live view / debug ---
            ("publish_debug",      True),    # /viman/boundary/image_debug
                                             # (view with rqt_image_view)
            ("show_window",        False),   # OpenCV window — needs a real
                                             # display; auto-off when headless
            ("mjpeg_port",         8080),    # live browser stream:
                                             # http://<jetson-ip>:8080
                                             # 0 = disabled
            ("debug_rate_hz",      10.0),    # annotated-frame rate cap
            ("debug_save_path",    ""),      # e.g. /tmp/boundary_debug.jpg
            ("log_period_s",       1.0),     # status-table row period
        ])

        gp = lambda n: self.get_parameter(n).value
        self._hsv_low   = np.array([int(v) for v in gp("hsv_low")],
                                   dtype=np.uint8)
        self._hsv_high  = np.array([int(v) for v in gp("hsv_high")],
                                   dtype=np.uint8)
        self._use_clahe = bool(gp("use_clahe"))
        self._min_px    = int(gp("min_pixels"))
        self._open_k    = int(gp("open_kernel_px"))
        self._line_filter       = bool(gp("line_filter_enabled"))
        self._close_k           = int(gp("close_kernel_px"))
        self._min_comp_area     = int(gp("min_component_area_px"))
        self._min_line_len_frac = float(gp("min_line_length_frac"))
        self._min_line_len_px   = int(gp("min_line_length_px"))
        self._max_line_width    = float(gp("max_line_width_px"))
        self._min_aspect        = float(gp("min_aspect_ratio"))
        self._min_rect          = float(gp("min_rectangularity"))
        self._influence = float(gp("influence_radius_m"))
        self._falloff   = float(gp("falloff_power"))
        self._corner_on = bool(gp("corner_enabled"))
        self._corner_diff_rad = math.radians(float(gp("corner_min_diff_deg")))
        self._corner_defect = int(gp("corner_min_defect_px"))
        self._max_lines = max(1, int(gp("max_lines")))
        self._line_hough_thresh = int(gp("line_hough_thresh"))
        self._line_min_len = int(gp("line_min_len_px"))
        self._line_max_gap = int(gp("line_max_gap_px"))
        self._scale     = float(gp("proc_scale"))
        self._interval_ns = int(1e9 / float(gp("max_rate_hz")))
        self._min_alt   = float(gp("min_alt_m"))
        self._default_alt = float(gp("default_alt_m"))
        yaw_off = math.radians(float(gp("cam_yaw_offset_deg")))
        self._cos_off, self._sin_off = math.cos(yaw_off), math.sin(yaw_off)
        self._tilt_comp = bool(gp("tilt_comp_enabled"))
        self._tilt_pitch_sign = float(gp("tilt_pitch_sign"))
        self._tilt_roll_sign = float(gp("tilt_roll_sign"))
        self._tilt_max = math.radians(float(gp("tilt_max_deg")))
        self._roll = 0.0
        self._pitch = 0.0
        self._publish_dbg = bool(gp("publish_debug"))
        self._show_window = bool(gp("show_window"))
        self._mjpeg_port  = int(gp("mjpeg_port"))
        self._dbg_interval_ns = int(1e9 / max(1.0, float(gp("debug_rate_hz"))))
        self._save_path   = str(gp("debug_save_path"))
        self._log_period  = float(gp("log_period_s"))

        self._clahe = (cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
                       if self._use_clahe else None)
        self._bridge = CvBridge()

        self._fx = None                     # from camera_info
        self._alt = 0.0                     # latest altitude
        self._have_pose = False
        self._last_proc_ns = 0
        self._last_dbg_ns = 0
        self._window_ok = None              # None=untried, False=headless

        # HEADLESS GUARD: cv2.imshow's Qt backend ABORTS the whole
        # process (SIGABRT, "could not connect to display") when there is
        # no X display — it cannot be caught from Python. Check the
        # environment BEFORE ever calling imshow.
        if self._show_window and not (os.environ.get("DISPLAY")
                                      or os.environ.get("WAYLAND_DISPLAY")):
            self._show_window = False
            self.get_logger().warn(
                "show_window requested but no display found (SSH session?)"
                " — window disabled. Watch the live feed in a browser "
                f"instead: http://<jetson-ip>:{self._mjpeg_port}  "
                "(or ssh -X for X-forwarding).")

        # MJPEG live stream — view from ANY browser on the same network.
        self._latest_jpeg = None
        self._mjpeg_clients = 0
        self._mjpeg_running = True
        self._mjpeg_srv = None
        if self._mjpeg_port > 0:
            try:
                det = self

                class _Handler(BaseHTTPRequestHandler):
                    def do_GET(self):
                        self.send_response(200)
                        self.send_header("Cache-Control",
                                         "no-cache, private")
                        self.send_header(
                            "Content-Type",
                            "multipart/x-mixed-replace; boundary=frame")
                        self.end_headers()
                        det._mjpeg_clients += 1
                        try:
                            while det._mjpeg_running:
                                jpg = det._latest_jpeg
                                if jpg is not None:
                                    self.wfile.write(
                                        b"--frame\r\n"
                                        b"Content-Type: image/jpeg\r\n"
                                        + f"Content-Length: {len(jpg)}"
                                          f"\r\n\r\n".encode()
                                        + jpg + b"\r\n")
                                time.sleep(0.08)
                        except (BrokenPipeError, ConnectionResetError,
                                OSError):
                            pass
                        finally:
                            det._mjpeg_clients -= 1

                    def log_message(self, *args):
                        pass                     # silence per-request spam

                self._mjpeg_srv = ThreadingHTTPServer(
                    ("0.0.0.0", self._mjpeg_port), _Handler)
                threading.Thread(target=self._mjpeg_srv.serve_forever,
                                 daemon=True).start()
                self.get_logger().info(
                    f"Live MJPEG stream: http://<jetson-ip>:"
                    f"{self._mjpeg_port}  (open in any browser)")
            except OSError as e:
                self.get_logger().warn(
                    f"MJPEG stream disabled (port {self._mjpeg_port}: {e})")
                self._mjpeg_srv = None

        # Latest results (for the status table)
        self._coverage = 0.0                # % of frame that is yellow
        self._nearest = -1.0
        self._push = (0.0, 0.0)             # body-FLU repulsion (weighted)
        self._strength = 0.0                # = weight of the opposite vector
        self._n_px = 0
        self._corner_body = None            # (bx, by) body-FLU, or None
        self._corner_uv = None              # full-res pixel, for debug draw

        # Per-line results (front + side). Each entry:
        #   {'dist': m, 'nx': body-FLU push-away X, 'ny': ..Y,
        #    'strength': 0-1, 'uv': (u,v) full-res foot-point for the blue arrow}
        self._lines = []

        # Period counters
        self._frames = 0
        self._yellow_frames = 0
        self._t_period = time.monotonic()
        self._rows = 0

        self._image_topic = str(gp("image_topic"))
        self._info_topic = str(gp("camera_info_topic"))
        self._last_img_ns = 0               # ANY image arrival (pre-filter)
        self._start_ns = self.get_clock().now().nanoseconds  # for startup grace

        self.create_subscription(
            CameraInfo, self._info_topic, self._cb_info, 10)
        self.create_subscription(
            Image, self._image_topic, self._cb_image, qos_best_effort())
        self.create_subscription(
            PoseStamped, gp("pose_topic"), self._cb_pose, qos_best_effort())
        # Mission status from the director ("<command>|<rtabQ>") — shown as the
        # CMD and RTAB columns in the table below.
        self._cmd_txt = "—"
        self._rtab_q  = "—"
        self.create_subscription(
            String, "/viman/mission/status", self._cb_status, 10)

        self._rep_pub = self.create_publisher(
            Vector3Stamped, gp("repulsion_topic"), 10)
        self._near_pub = self.create_publisher(
            Float32, gp("nearest_topic"), 10)
        self._cov_pub = self.create_publisher(
            Float32, gp("coverage_topic"), 10)
        self._corner_pub = self.create_publisher(
            Vector3Stamped, gp("corner_topic"), 10)
        self._lines_pub = self.create_publisher(
            Float32MultiArray, gp("lines_topic"), 10)
        self._dbg_pub = (self.create_publisher(
            Image, "/viman/boundary/image_debug", 1)
            if self._publish_dbg else None)

        self.create_timer(self._log_period, self._print_table_row)
        self.create_timer(5.0, self._camera_watchdog)

        self.get_logger().info(
            "LINE-ONLY filter " + ("ON" if self._line_filter else "OFF")
            + f": keep area>={self._min_comp_area}px, "
            f"len>={self._min_line_len_frac*100:.0f}% frame "
            f"(floor {self._min_line_len_px}px), width<={self._max_line_width:.0f}px, "
            f"aspect>={self._min_aspect:.1f}:1, rect>={self._min_rect:.2f} "
            "— small yellow patches (clothing/scraps/objects) are ignored.")
        self.get_logger().info(
            f"yellow_boundary_detector up — influence {self._influence:.1f} m,"
            f" falloff^{self._falloff:.0f}, ≤{1e9/self._interval_ns:.0f} Hz, "
            f"scale {self._scale} | analysis via terminal table"
            + (" + window" if self._show_window else "")
            + (" + debug topic" if self._publish_dbg else "")
            + (f" + {self._save_path}" if self._save_path else ""))

    # ── Callbacks ────────────────────────────────────────────────

    def _cb_info(self, msg: CameraInfo):
        if self._fx is None:
            self._fx = float(msg.k[0])
            self.get_logger().info(f"Intrinsics: fx={self._fx:.1f}")

    def _cb_pose(self, msg: PoseStamped):
        self._alt = msg.pose.position.z
        self._have_pose = True
        # roll (about body +X / forward) and pitch (about body +Y / left),
        # ENU-FLU — used to keep line distances honest when the drone tilts.
        q = msg.pose.orientation
        self._roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                                1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        sp = 2.0 * (q.w * q.y - q.z * q.x)
        self._pitch = math.asin(max(-1.0, min(1.0, sp)))

    def _cb_image(self, msg: Image):
        now_ns = self.get_clock().now().nanoseconds
        self._last_img_ns = now_ns          # camera-alive watchdog feed
        # Rate cap — drop frames beyond max_rate_hz
        if now_ns - self._last_proc_ns < self._interval_ns:
            return
        self._last_proc_ns = now_ns

        if self._fx is None:
            return

        alt = self._alt if self._have_pose else self._default_alt
        if self._have_pose and alt < self._min_alt:
            self._lines = []
            self._set_result(0.0, 0.0, 0.0, -1.0, 0.0, 0)
            self._publish(msg.header.stamp)
            return

        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge: {e}", throttle_duration_sec=5.0)
            return

        self._frames += 1
        mask = self._yellow_mask(bgr)
        ys, xs = np.nonzero(mask)
        coverage = 100.0 * len(xs) / mask.size
        nearest_uv = None                    # full-res (u, v) of closest px

        if len(xs) < self._min_px:
            self._corner_body = self._corner_uv = None
            self._lines = []
            self._set_result(0.0, 0.0, 0.0, -1.0, coverage, len(xs))
            self._publish(msg.header.stamp)
            self._maybe_debug(bgr, mask, alt, now_ns, None)
            return

        self._yellow_frames += 1

        # ── Pixel offsets → metric body-FLU ground offsets ────────
        h, w = mask.shape
        cx, cy = w * 0.5, h * 0.5
        # Camera-tilt compensation: shift the effective nadir (cx, cy) by the
        # drone's pitch/roll so a line's ground distance is measured from where
        # the camera ACTUALLY looks, not from image centre. Without this, a
        # forward pitch makes the FRONT line read too close and the drone
        # retreats backward until the line leaves the frame. All downstream
        # distances (nearest, per-line, corner) use this cx/cy, so one shift
        # fixes them all.
        if self._tilt_comp and self._have_pose:
            f = self._fx * self._scale
            pitch = max(-self._tilt_max, min(self._tilt_max, self._pitch))
            roll = max(-self._tilt_max, min(self._tilt_max, self._roll))
            cx += self._tilt_roll_sign * f * math.tan(roll)
            cy += self._tilt_pitch_sign * f * math.tan(pitch)
        m_per_px = max(alt, 0.3) / (self._fx * self._scale)

        du = (xs - cx) * m_per_px        # +right of nadir  → body −Y
        dv = (ys - cy) * m_per_px        # +below in image  → body −X
        bx = -dv                         # body forward
        by = -du                         # body left

        # Optional mounting yaw correction
        if self._sin_off != 0.0:
            bx, by = (bx * self._cos_off - by * self._sin_off,
                      bx * self._sin_off + by * self._cos_off)

        r = np.hypot(bx, by)
        r = np.maximum(r, 1e-6)
        idx = int(np.argmin(r))
        nearest = float(r[idx])
        nearest_uv = (float(xs[idx]) / self._scale,
                      float(ys[idx]) / self._scale)

        # ── Potential field: weighted sum of unit push-away vectors ──
        # weight(r) = (1 − r/R)^p inside the influence radius, else 0.
        wgt = np.clip(1.0 - r / self._influence, 0.0, None) ** self._falloff
        wsum = float(wgt.sum())
        if wsum > 1e-9:
            # push-away = −(pixel direction), weighted mean over pixels
            mx = float((wgt * (-bx / r)).sum()) / wsum
            my = float((wgt * (-by / r)).sum()) / wsum
            norm = math.hypot(mx, my)
            if norm > 1e-9:
                dir_x, dir_y = mx / norm, my / norm
            else:
                # Perfectly surrounded / symmetric — no usable direction.
                dir_x = dir_y = 0.0
        else:
            dir_x = dir_y = 0.0

        # Weight (strength) from the NEAREST pixel — monotonic:
        # 0 at the influence radius, → 1 as the line reaches the nadir.
        # "The closer the yellow, the stronger the push."
        strength = max(0.0, min(1.0, 1.0 - nearest / self._influence)) ** 2

        # ── Corner detection (L of two tape arms) ────────────────
        self._detect_corner(mask, cx, cy, m_per_px)

        # ── Per-line split (front + side) — each gets its own vector ─
        self._detect_lines(mask, cx, cy, m_per_px, bx, by, r)

        self._set_result(dir_x * strength, dir_y * strength, strength,
                         nearest, coverage, len(xs))
        self._publish(msg.header.stamp)
        self._maybe_debug(bgr, mask, alt, now_ns, nearest_uv)

    # ── Segmentation: mask only, no shape constraints ────────────

    def _yellow_mask(self, bgr):
        s = self._scale
        small = cv2.resize(bgr, None, fx=s, fy=s) if s != 1.0 else bgr
        small = cv2.GaussianBlur(small, (5, 5), 0)
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        if self._clahe is not None:
            h_ch, s_ch, v_ch = cv2.split(hsv)
            hsv = cv2.merge([h_ch, s_ch, self._clahe.apply(v_ch)])
        mask = cv2.inRange(hsv, self._hsv_low, self._hsv_high)
        if self._open_k >= 3:
            k = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (self._open_k, self._open_k))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        # Keep ONLY long continuous boundary lines; erase small yellow patches
        # (clothing, scraps, objects) so they never enter the potential field.
        if self._line_filter:
            if self._close_k >= 3:
                ck = cv2.getStructuringElement(
                    cv2.MORPH_ELLIPSE, (self._close_k, self._close_k))
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, ck)
            mask = self._keep_line_components(mask)
        return mask

    def _keep_line_components(self, mask):
        """Keep only LONG, CONTINUOUS, THIN, ELONGATED, RECTANGULAR yellow
        components (the real arena boundary tape); erase everything else
        (yellow patches on clothing, tape scraps, coloured objects).

        WHY: the potential field sums a push over EVERY yellow pixel, so a
        scrap on a shoe directly under the drone (near≈0.18 m, weight≈0.83)
        pushed as hard as — and masked — the real boundary metres away. That
        is the erratic motion seen in testing. Filtering the mask HERE fixes
        nearest, repulsion, per-line and corner all at once.

        A connected component is kept only if it passes ALL five tests:
          • BIG         — area >= min_component_area_px           (not a speckle)
          • LONG        — minAreaRect longer side spans a big fraction of the
                          frame                                    (a real run)
          • THIN        — stroke width (area / half-perimeter) <= max_line_width_px
                          (a stroke, not a filled blob/object)
          • ELONGATED   — minAreaRect length/width >= min_aspect_ratio
                          (real tape is ≥15:1; patches are 1:1 to 3:1) — this
                          is the strongest single discriminator against
                          clothing patches, which triggered the erratic motion
                          in the flight footage.
          • RECTANGULAR — contour area / minAreaRect area >= min_rectangularity
                          (real tape fills its bounding rect ~0.7-0.9;
                          irregular blobs fill much less)

        A straight tape AND an L-corner (two thin arms) both pass; a fat
        clothing patch of similar length fails ELONGATED; a roundish scrap
        fails ELONGATED; irregular shapes fail RECTANGULAR. If nothing
        qualifies the returned mask is empty and the detector correctly
        reports 'no boundary' (a scrap is not a wall)."""
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return mask
        h, w = mask.shape
        min_len = max(float(self._min_line_len_px),
                      self._min_line_len_frac * max(h, w))
        keep = np.zeros_like(mask)
        for c in cnts:
            area = cv2.contourArea(c)
            if area < self._min_comp_area:            # too small → speckle/scrap
                continue
            (_, _), (rw, rh), _ = cv2.minAreaRect(c)
            length = max(rw, rh)
            width = min(rw, rh)
            if length < min_len:                      # too short → not a boundary
                continue
            # ELONGATED: length/width. Guard against div-by-zero on
            # degenerate 1-pixel-wide rects; a rect that thin is definitely
            # a real line, so pass the aspect-ratio test.
            aspect = length / width if width > 1.0 else float("inf")
            if aspect < self._min_aspect:             # too square → patch/blob
                continue
            # RECTANGULAR: how well the contour fills its minAreaRect. A
            # straight tape run fills most of it (~0.7-0.9); an irregular
            # clothing patch of similar bounding box fills far less.
            rect_area = length * width
            if rect_area > 1e-6 and (area / rect_area) < self._min_rect:
                continue
            perim = cv2.arcLength(c, True)
            stroke = area / max(0.5 * perim, 1.0)     # ≈ average tape thickness
            if stroke > self._max_line_width:         # too fat → blob/object
                continue
            cv2.drawContours(keep, [c], -1, 255, thickness=cv2.FILLED)
        return keep

    # ── Corner detection ─────────────────────────────────────────
    # Same proven recipe as yellow_detector.py: Hough segments on the
    # mask EDGES (Canny first — a filled band would spew interior
    # lines), require two direction clusters ≥ corner_min_diff apart,
    # then locate the inner corner as the deepest convexity defect of
    # the tape blob. Runs on the already-computed half-res mask —
    # a few ms on the Jetson.

    def _detect_corner(self, mask, cx, cy, m_per_px):
        self._corner_body = self._corner_uv = None
        if not self._corner_on:
            return
        try:
            edges = cv2.Canny(mask, 50, 150)
            lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 40,
                                    minLineLength=30, maxLineGap=20)
            if lines is None or len(lines) < 2:
                return
            a = np.arctan2(lines[:, 0, 3] - lines[:, 0, 1],
                           lines[:, 0, 2] - lines[:, 0, 0]) % np.pi
            # two direction clusters ≥ min_diff apart? (wrap at π)
            d = np.abs(a[:, None] - a[None, :])
            d = np.minimum(d, np.pi - d)
            if float(d.max()) < self._corner_diff_rad:
                return                     # single straight line only

            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                return
            c = max(cnts, key=cv2.contourArea)
            if len(c) < 10:
                return
            eps = 0.005 * cv2.arcLength(c, True)
            c = cv2.approxPolyDP(c, eps, True)
            if len(c) < 5:
                return
            hull = cv2.convexHull(c, returnPoints=False)
            defects = cv2.convexityDefects(c, hull)
            if defects is None or len(defects) == 0:
                return
            best_depth, best_pt = 0.0, None
            for de in defects:
                s, e, f, depth_fp = de[0]
                depth = depth_fp / 256.0
                if depth > best_depth:
                    best_depth = depth
                    best_pt = tuple(c[f][0])
            if best_pt is None or best_depth < self._corner_defect:
                return

            u, v = float(best_pt[0]), float(best_pt[1])
            bx = -(v - cy) * m_per_px       # body forward
            by = -(u - cx) * m_per_px       # body left
            if self._sin_off != 0.0:
                bx, by = (bx * self._cos_off - by * self._sin_off,
                          bx * self._sin_off + by * self._cos_off)
            self._corner_body = (bx, by)
            self._corner_uv = (u / self._scale, v / self._scale)
        except Exception:
            self._corner_body = self._corner_uv = None

    # ── Per-line split (front + side) ────────────────────────────
    # The summed potential field gives ONE blended repulsion, which is
    # fine for a single line but ambiguous at a corner. Here we split
    # the yellow into up to _max_lines straight lines by orientation,
    # and for EACH line report its own perpendicular distance + a
    # push-away unit vector — the "own blue vector" per line. A corner
    # then has two independent standoffs, so a consumer can hold clear
    # of BOTH lines at once instead of chasing a blended average.

    def _cluster_angles(self, ang, k):
        """Group segment angles (rad, mod π) into ≤ k clusters by the
        largest angular gaps (≥ corner_min_diff). Returns per-segment
        labels. Rotates the circular order to start after the biggest
        gap so wrap-around clustering is handled correctly."""
        n = len(ang)
        if n == 0:
            return np.zeros(0, dtype=int)
        if k <= 1:
            return np.zeros(n, dtype=int)
        order = np.argsort(ang)
        a = ang[order]
        gaps = np.append(np.diff(a), (a[0] + np.pi) - a[-1])  # last=wrap gap
        start = (int(np.argmax(gaps)) + 1) % n
        rot = np.concatenate([np.arange(start, n), np.arange(0, start)])
        a_rot = a[rot]
        gaps_rot = np.diff(a_rot) if n > 1 else np.zeros(0)
        cand = [(float(gaps_rot[i]), i) for i in range(len(gaps_rot))
                if gaps_rot[i] >= self._corner_diff_rad]
        labels_rot = np.zeros(n, dtype=int)
        if cand:
            cand.sort(reverse=True)
            splits = sorted(i for _, i in cand[:k - 1])
            lab = 0
            for i in range(n):
                labels_rot[i] = lab
                if i in splits:
                    lab += 1
        labels_sorted = np.zeros(n, dtype=int)
        labels_sorted[rot] = labels_rot
        labels = np.zeros(n, dtype=int)
        labels[order] = labels_sorted
        return labels

    def _detect_lines(self, mask, cx, cy, m_per_px, bx, by, r):
        """Populate self._lines: up to _max_lines dicts, each
        {dist(m), nx, ny (body-FLU push-away unit), strength, uv}.
        Falls back to the single nearest-pixel line if Hough finds no
        segments (thin/curved tape), so there is ALWAYS ≥1 line while
        yellow is in view."""
        def _fallback():
            idx = int(np.argmin(r))
            d = float(r[idx])
            pbx, pby = float(-bx[idx]), float(-by[idx])   # pixel → nadir
            nrm = math.hypot(pbx, pby)
            if nrm < 1e-9:
                return []
            strength = max(0.0, min(1.0,
                                    1.0 - d / self._influence)) ** self._falloff
            return [{'dist': d, 'nx': pbx / nrm, 'ny': pby / nrm,
                     'strength': strength, 'uv': None}]

        if self._max_lines <= 1:
            self._lines = _fallback()
            return
        try:
            edges = cv2.Canny(mask, 50, 150)
            hlines = cv2.HoughLinesP(
                edges, 1, np.pi / 180, self._line_hough_thresh,
                minLineLength=self._line_min_len,
                maxLineGap=self._line_max_gap)
        except Exception:
            hlines = None
        if hlines is None or len(hlines) == 0:
            self._lines = _fallback()
            return

        segs = hlines[:, 0, :].astype(np.float64)          # x1,y1,x2,y2
        ang = np.arctan2(segs[:, 3] - segs[:, 1],
                         segs[:, 2] - segs[:, 0]) % np.pi
        length = np.hypot(segs[:, 2] - segs[:, 0], segs[:, 3] - segs[:, 1])
        labels = self._cluster_angles(ang, self._max_lines)

        out = []
        for lab in sorted(set(labels.tolist())):
            sel = labels == lab
            pts = np.vstack([segs[sel][:, :2], segs[sel][:, 2:]])
            if len(pts) < 2:
                continue
            vx, vy, x0, y0 = cv2.fitLine(
                pts.astype(np.float32), cv2.DIST_L2, 0, 0.01, 0.01).flatten()
            nu = (-float(vy), float(vx))                   # unit normal (image)
            s = (cx - float(x0)) * nu[0] + (cy - float(y0)) * nu[1]
            d_m = abs(s) * m_per_px
            sgn = 1.0 if s >= 0 else -1.0
            pu = (sgn * nu[0], sgn * nu[1])                # image unit → nadir
            foot_u = (cx - s * nu[0]) / self._scale        # nearest point (full-res)
            foot_v = (cy - s * nu[1]) / self._scale
            nbx, nby = -pu[1], -pu[0]                      # image → body-FLU
            if self._sin_off != 0.0:
                nbx, nby = (nbx * self._cos_off - nby * self._sin_off,
                            nbx * self._sin_off + nby * self._cos_off)
            nn = math.hypot(nbx, nby)
            if nn < 1e-9:
                continue
            strength = max(0.0, min(1.0,
                                    1.0 - d_m / self._influence)) ** self._falloff
            out.append({'dist': d_m, 'nx': nbx / nn, 'ny': nby / nn,
                        'strength': strength, 'uv': (foot_u, foot_v),
                        '_len': float(length[sel].sum())})

        if not out:
            self._lines = _fallback()
            return
        # keep the strongest few (by total tape length), report nearest first
        out.sort(key=lambda L: -L['_len'])
        out = out[:self._max_lines]
        out.sort(key=lambda L: L['dist'])
        self._lines = out

    # ── Results / publishing ─────────────────────────────────────

    def _set_result(self, px, py, strength, nearest, coverage, n_px):
        self._push = (px, py)
        self._strength = strength
        self._nearest = nearest
        self._coverage = coverage
        self._n_px = n_px

    def _publish(self, stamp):
        v = Vector3Stamped()
        v.header.stamp = stamp
        v.header.frame_id = "body_flu"      # x fwd, y left
        v.vector.x = float(self._push[0])
        v.vector.y = float(self._push[1])
        v.vector.z = 0.0
        self._rep_pub.publish(v)

        n = Float32(); n.data = float(self._nearest)
        self._near_pub.publish(n)
        c = Float32(); c.data = float(self._coverage)
        self._cov_pub.publish(c)

        k = Vector3Stamped()
        k.header.stamp = stamp
        k.header.frame_id = "body_flu"
        if self._corner_body is not None:
            k.vector.x = float(self._corner_body[0])
            k.vector.y = float(self._corner_body[1])
            k.vector.z = 1.0                # corner visible
        self._corner_pub.publish(k)

        # Per-line array: [n, (dist, nx, ny, strength) × n] — body-FLU,
        # normals point AWAY from each line (toward the nadir). Consumers
        # clamp motion toward each line independently (corner-safe).
        arr = Float32MultiArray()
        data = [float(len(self._lines))]
        for L in self._lines:
            data += [float(L['dist']), float(L['nx']),
                     float(L['ny']), float(L['strength'])]
        arr.data = data
        self._lines_pub.publish(arr)

    # ── Camera-alive watchdog (every 5 s) ────────────────────────
    # The one failure that MUST be unmissable: no camera stream means
    # no detection means the guard is blind. Shout until frames arrive.

    def _camera_watchdog(self):
        now_ns = self.get_clock().now().nanoseconds
        if self._last_img_ns == 0:
            # STARTUP GRACE: rs_pipeline does a USB hardware-reset + re-
            # enumerate on boot (~5-12 s) before the first frame. During that
            # window "no images yet" is EXPECTED, not a fault — so log a calm
            # "waiting" note, and only escalate to the scary DETECTION-IS-DEAD
            # error if frames still haven't arrived after the grace period.
            if now_ns - self._start_ns < 15_000_000_000:
                self.get_logger().info(
                    f"Waiting for camera images on '{self._image_topic}' "
                    "(RealSense still booting/hardware-resetting)…",
                    throttle_duration_sec=5.0)
                return
            self.get_logger().error(
                f"NO CAMERA IMAGES on '{self._image_topic}' — is a camera "
                "driver running? Start your RealSense driver, or relaunch "
                "with start_camera:=true. DETECTION IS DEAD — DO NOT FLY "
                "NEAR THE LINE.")
        elif now_ns - self._last_img_ns > 3_000_000_000:
            self.get_logger().error(
                f"Camera images STOPPED on '{self._image_topic}' "
                f"({(now_ns - self._last_img_ns)/1e9:.0f} s ago) — "
                "DETECTION IS DEAD.")
        elif self._fx is None:
            self.get_logger().error(
                f"Images arriving but NO camera_info on "
                f"'{self._info_topic}' — cannot compute metric distances. "
                "DETECTION IS DEAD.")

    # ── Status table (1 Hz) — the primary analysis output ────────

    def _cb_status(self, msg: String):
        """Latest mission status from the director: "<command>|<rtabQ>"."""
        txt = msg.data or ""
        if "|" in txt:
            cmd, q = txt.rsplit("|", 1)
            self._cmd_txt = cmd.strip() or "—"
            self._rtab_q  = q.strip() or "—"
        else:
            self._cmd_txt = txt.strip() or "—"

    def _print_table_row(self):
        now = time.monotonic()
        dt = max(1e-3, now - self._t_period)
        fps = self._frames / dt
        det_pct = (100.0 * self._yellow_frames / self._frames
                   if self._frames else 0.0)
        self._frames = self._yellow_frames = 0
        self._t_period = now

        if self._rows % 15 == 0:
            print(f"\n  {'YELLOW%':>8}  {'DETECT%':>8}  {'NEAR(m)':>8}  "
                  f"{'PUSH-X':>7}  {'PUSH-Y':>7}  {'WEIGHT':>7}  "
                  f"{'PX':>6}  {'FPS':>5}  {'RTAB':>6}  {'CMD':<26}  CORNER")
            print("  " + "─" * 120)
        self._rows += 1

        near_txt = f"{self._nearest:8.2f}" if self._nearest >= 0 else \
            f"{'—':>8}"
        if self._corner_body is not None:
            corner_txt = (f"YES ({self._corner_body[0]:+.2f},"
                          f"{self._corner_body[1]:+.2f})m")
        else:
            corner_txt = "—"
        if len(self._lines) >= 2:
            dists = ",".join(f"{L['dist']:.2f}" for L in self._lines)
            corner_txt += f" [{len(self._lines)}L:{dists}]"
        cmd_txt = (self._cmd_txt[:26]) if self._cmd_txt else "—"
        print(f"  {self._coverage:>7.2f}%  {det_pct:>7.0f}%  {near_txt}  "
              f"{self._push[0]:>+7.2f}  {self._push[1]:>+7.2f}  "
              f"{self._strength:>7.2f}  {self._n_px:>6d}  {fps:>5.1f}  "
              f"{self._rtab_q:>6}  {cmd_txt:<26}  {corner_txt}")

    # ── Optional annotated frame (window / topic / JPEG on disk) ─

    def _maybe_debug(self, bgr, mask, alt, now_ns, nearest_uv):
        mjpeg_live = self._mjpeg_srv is not None and self._mjpeg_clients > 0
        want = (self._dbg_pub is not None or self._show_window
                or self._save_path or mjpeg_live)
        if not want:
            return
        if now_ns - self._last_dbg_ns < self._dbg_interval_ns:
            return
        self._last_dbg_ns = now_ns

        try:
            dbg = bgr.copy()
            big = cv2.resize(mask, (bgr.shape[1], bgr.shape[0]),
                             interpolation=cv2.INTER_NEAREST)
            dbg[big > 0] = (0, 255, 255)                 # yellow tint
            h, w = dbg.shape[:2]
            c = (w // 2, h // 2)

            # Influence radius circle (metres → pixels at this altitude)
            r_px = int(self._influence * self._fx / max(alt, 0.3))
            cv2.circle(dbg, c, r_px, (200, 200, 200), 1, cv2.LINE_AA)
            cv2.circle(dbg, c, 6, (255, 0, 0), -1)       # nadir dot (blue)

            # BLUE arrow — toward the NEAREST yellow pixel (the threat)
            if nearest_uv is not None:
                cv2.arrowedLine(dbg, c,
                                (int(nearest_uv[0]), int(nearest_uv[1])),
                                (255, 0, 0), 3, tipLength=0.15)

            # CYAN arrow + label per detected LINE — front and side each
            # get their own "blue vector" pointing to that line's nearest
            # point, so a corner shows both standoffs at once.
            for li, L in enumerate(self._lines):
                if L.get('uv') is None:
                    continue
                fu, fv = int(L['uv'][0]), int(L['uv'][1])
                cv2.arrowedLine(dbg, c, (fu, fv), (255, 200, 0), 2,
                                tipLength=0.2)
                cv2.putText(dbg, f"L{li+1} {L['dist']:.2f}m", (fu + 6, fv),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 2)

            # RED arrow — resultant repulsion (push-away), body→screen:
            # body fwd = image up (−v), body left = image left (−u)
            px, py = self._push
            mag = math.hypot(px, py)
            if mag > 1e-6:
                ln = 60 + 140 * min(1.0, self._strength)
                tip = (int(c[0] - (py / mag) * ln),
                       int(c[1] - (px / mag) * ln))
                cv2.arrowedLine(dbg, c, tip, (0, 0, 255), 5, tipLength=0.3)

            # Corner marker (magenta)
            if self._corner_uv is not None:
                cu, cv_ = int(self._corner_uv[0]), int(self._corner_uv[1])
                cv2.circle(dbg, (cu, cv_), 18, (255, 0, 255), 3)
                cv2.putText(dbg, "CORNER", (cu + 22, cv_ + 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

            # HUD
            near_txt = (f"{self._nearest:.2f}m" if self._nearest >= 0
                        else "none")
            lines = [
                f"near={near_txt}  yellow={self._coverage:.1f}%  "
                f"weight={self._strength:.2f}"
                + ("  CORNER!" if self._corner_body is not None else ""),
                "RED=push away   BLUE=nearest yellow",
            ]
            for i, txt in enumerate(lines):
                y = 30 + i * 26
                cv2.putText(dbg, txt, (8, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 0, 0), 4)
                cv2.putText(dbg, txt, (8, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (255, 255, 255), 1)

            if self._dbg_pub is not None:
                self._dbg_pub.publish(self._bridge.cv2_to_imgmsg(dbg, "bgr8"))
            if mjpeg_live:
                ok, buf = cv2.imencode(
                    ".jpg", dbg, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    self._latest_jpeg = buf.tobytes()
            if self._save_path:
                cv2.imwrite(self._save_path, dbg,
                            [cv2.IMWRITE_JPEG_QUALITY, 85])
            if self._show_window and self._window_ok is not False:
                try:
                    cv2.imshow(WINDOW_NAME, dbg)
                    cv2.waitKey(1)
                    self._window_ok = True
                except cv2.error:
                    self._window_ok = False
                    self.get_logger().warn(
                        "OpenCV window failed — disabled (headless?).")
        except Exception as e:
            self.get_logger().warn(f"debug frame: {e}",
                                   throttle_duration_sec=5.0)

    def destroy_node(self):
        self._mjpeg_running = False
        if self._mjpeg_srv is not None:
            try:
                self._mjpeg_srv.shutdown()
            except Exception:
                pass
        if self._window_ok:
            try:
                cv2.destroyWindow(WINDOW_NAME)
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = YellowBoundaryDetector()
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