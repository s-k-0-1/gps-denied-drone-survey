#!/usr/bin/env python3
"""
survey_mission — Lawnmower photographic survey.
Team Viman Rakshak / IRoC-U 2026.

Flies a boustrophedon (lawnmower) grid over a configurable area.
At every checkpoint the drone settles, captures a colour frame, and
logs ENU coordinates + heading to a CSV — everything lands on the SSD.

Mission profile
───────────────
  IDLE → ARM → TAKEOFF → STABLE_OF → HOVER_HOME → SEED → VALIDATE
  → HANDOVER → GOTO_HOME → SURVEY (loop over checkpoints) → RETURN
  → FLOW_SETTLE → DESCEND → LAND → DISARM → DONE

  Yaw note: the survey-grid reference is the drone's OWN heading captured
  automatically at ARM — no hard-coded compass value. Heading still floats
  (tracks the live EKF estimate) through the whole climb so an EKF2
  magnetometer yaw reset cannot snap the airframe; the captured arm-time
  heading is then APPLIED at altitude in HOVER_HOME by gently slewing onto
  it (yaw_slew_dps), and held for the whole survey.

Output (written while flying)
──────────────────────────────
  /media/jetson/ROS2_SSD/survey/survey_<YYYYMMDD_HHMMSS>/
      coordinates.csv   — checkpoint, row, col, timestamp_s,
                          x_enu, y_enu, z_enu, yaw_deg, image_file
      cp0000_r00c00.jpg — JPEG images at each checkpoint

Run
───
  ros2 launch viman_mission bringup.launch.py mission_node:=survey_mission

Override params at launch
──────────────────────────
  ros2 launch viman_mission bringup.launch.py mission_node:=survey_mission \
    survey_width_m:=8.0 survey_height_m:=8.0 \
    stripe_spacing_m:=1.5 col_spacing_m:=1.5 \
    target_alt:=4.0
"""

import csv
import math
import os
import signal
import threading
import time
from datetime import datetime
from enum import Enum, auto

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from mavros_msgs.msg import RCIn, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String, UInt8
from std_srvs.srv import SetBool, Trigger
from whycode_interfaces.msg import MarkerArray

from viman_mission.common import (qos_best_effort, qos_reliable,
                                  yaw_deg_from_quaternion)

# vio_gate states
GS_UNSEEDED    = 0
GS_SEEDING     = 1
GS_VALIDATING  = 2
GS_OPEN        = 3
GS_FAULT_MIN   = 4   # 4, 5, 6 are all fault states


class Phase(Enum):
    IDLE        = auto()
    ARM         = auto()
    TAKEOFF     = auto()
    STABLE_OF   = auto()
    HOVER_HOME  = auto()   # fly back to arm point after takeoff drift, lock yaw
    SEED        = auto()
    VALIDATE    = auto()
    HANDOVER    = auto()
    GOTO_HOME   = auto()   # fly to arming point on VIO before survey starts
    ACQ_BACK    = auto()   # centre-start: fly BACK to the back line
    ACQ_LEFT    = auto()   # centre-start: slide LEFT until back+left in-band = corner
    ACQ_REYAW   = auto()   # centre-start: re-yaw to the L-corner, then survey
    SURVEY      = auto()   # lawnmower loop
    RETURN      = auto()   # crawl home
    FLOW_SETTLE = auto()   # gate closed, hold before descent
    FLOW_HOLD   = auto()   # zero-velocity hold on flow after VIO fault
    MARKER_DESCEND = auto()  # sink from home alt to search alt over home
    MARKER_SEARCH  = auto()  # hold at search alt, confirm the marker
    MARKER_CENTER  = auto()  # centre over the WhyCode marker
    MARKER_LAND    = auto()  # descend on the centred marker, hand off to AUTO.LAND
    DESCEND     = auto()   # controlled descent on flow (marker-less fallback)
    LAND        = auto()
    DISARM      = auto()
    SAFE_MANUAL = auto()
    DONE        = auto()


class SurveyMission(Node):

    def __init__(self):
        super().__init__("survey_mission")

        # ── Parameter declarations ────────────────────────────────
        p = self.declare_parameters("", [
            # --- flight profile ---
            ("target_alt",             3.0),
            ("alt_tolerance",          0.12),
            ("at_alt_confirm_s",       1.5),
            ("stable_of_secs",         4.0),
            ("seed_timeout_s",         10.0),
            ("validate_if_min",        0.7),
            ("validate_hold_s",        5.0),
            ("validate_dip_grace_s",   1.0),
            ("validate_timeout_s",     60.0),
            ("motion_test",            True),
            ("motion_amp_m",           0.2),
            ("motion_leg_s",           4.0),
            ("handover_settle_s",      2.0),
            ("goto_radius_m",          0.20),
            ("goto_timeout_s",         20.0),
            ("descend_speed_ms",       0.25),
            ("descend_handoff_alt_m",  0.3),
            ("descend_timeout_s",      15.0),
            ("flow_settle_s",          2.5),
            ("max_revalidations",      2),
            ("sp_rate_hz",             20.0),
            ("rc_ch5_index",           4),
            ("rc_start_low",           1200),
            ("rc_interrupt_high",      1700),
            ("preflight_pose_hz_min",  15.0),
            ("rtab_odom_topic",        "/rtabmap/rtabmap/odom"),
            # --- survey-specific ---
            ("survey_width_m",         6.0),   # East-West extent [m]
            ("survey_height_m",        6.0),   # North-South extent [m]
            ("stripe_spacing_m",       1.0),   # N-S pitch between stripes [m]
            ("col_spacing_m",          1.0),   # E-W pitch between checkpoints [m]
            ("waypoint_radius_m",      0.25),  # arrival acceptance circle [m]
            ("waypoint_settle_s",      1.0),   # hover time before photo [s]
            ("survey_speed_ms",        0.25),  # crawl speed [m/s]
            ("survey_dir",             "/media/jetson/ROS2_SSD/survey"),
            ("yaw_slew_dps",           15.0),  # max yaw rotation rate [°/s]
            # --- yaw reference: hold the ARM-time heading (default) or a fixed
            #     compass angle, exactly like boundary_test_auto ---
            ("yaw_use_arm_heading",    True),   # True = hold whatever heading the
                                                # drone had at ARM (survey default;
                                                # set yaw by facing the drone before
                                                # arming). False = use mission_yaw_deg.
            ("mission_yaw_deg",        0.0),    # fixed TARGET heading [deg] in the
                                                # ENU pose-yaw frame; used ONLY when
                                                # yaw_use_arm_heading is False.
            # --- HOVER_HOME yaw alignment (ACTIVE): after slewing onto the
            #     target heading, wait until the reported yaw is actually there,
            #     then RE-LOCK the grid frame to the drone's ACTUAL settled yaw
            #     (same align-and-relock as boundary_test_auto) ---
            ("yaw_align_tol_deg",      3.0),    # lock once |err| ≤ this [deg]
            ("yaw_align_hold_s",       1.0),    # hold aligned this long first [s]
            ("yaw_align_timeout_s",    25.0),   # give up aligning, proceed anyway [s]
        ])

        (self._target_alt, self._alt_tol, self._at_alt_confirm_s,
         self._stable_of_secs, self._seed_timeout_s,
         self._if_min, self._validate_hold_s, self._dip_grace_s,
         self._validate_timeout_s, self._motion_test,
         self._motion_amp, self._motion_leg_s, self._handover_settle_s,
         self._goto_radius, self._goto_timeout,
         self._descend_speed, self._descend_handoff_alt, self._descend_timeout,
         self._flow_settle_s, self._max_revalidations,
         self._sp_rate_hz, self._rc_ch5_idx, self._rc_start_low,
         self._rc_interrupt_high, self._pose_hz_min, rtab_topic,
         self._survey_w, self._survey_h, self._stripe_spacing,
         self._col_spacing, self._wpt_radius, self._wpt_settle_s,
         self._survey_speed, self._base_survey_dir,
         self._yaw_slew_dps,
         self._yaw_use_arm_heading, self._mission_yaw_deg,
         self._yaw_align_tol_deg, self._yaw_align_hold_s,
         self._yaw_align_timeout_s) = (x.value for x in p)

        # --- WhyCode precision landing (marker at the home/return point). If a
        #     marker is found the drone centres on it and lands on it; if not,
        #     it falls back to the flow-only descent at home. ---
        mp = self.declare_parameters("", [
            ("marker_land_enabled",      True),
            ("markers_topic",            "/whycode_node/markers"),
            ("marker_search_alt_m",      2.0),   # descend to here, then search
            ("marker_detect_frames",     4),     # consecutive good frames to lock
            ("marker_timeout_s",         2.0),   # pose older than this = "lost"
            ("marker_alt_gate_m",        1.5),   # reject |depth-alt| > this
            ("marker_outlier_jump_m",    1.0),   # reject a lone frame-to-frame jump
            ("marker_cam_x_sign",        1.0),   # flip if centring goes wrong in X
            ("marker_cam_y_sign",       -1.0),   # flip if centring goes wrong in Y
            ("marker_cam_yaw_offset_deg", 0.0),  # camera mounting rotation about the optical
                                                 # axis (0/90/180/270). If lat_err never
                                                 # shrinks in MARKER_CENTER (drone orbits),
                                                 # try 90 then 270.
            ("marker_center_thr_m",      0.15),  # within this = "centred"
            ("marker_center_hold_s",     1.0),   # hold centred this long
            ("marker_center_timeout_s",  20.0),  # can't centre → descend anyway
            ("marker_search_timeout_s",  15.0),  # no marker → flow-land at home
            ("marker_descend_speed_ms",  0.15),  # sink rate on the marker
        ])
        (self._marker_land_enabled, self._marker_topic, self._marker_search_alt,
         self._marker_detect_frames, self._marker_timeout_s, self._marker_alt_gate,
         self._marker_outlier, self._marker_cam_x_sign, self._marker_cam_y_sign,
         _marker_cam_yaw_off_deg, self._marker_center_thr, self._marker_center_hold_s,
         self._marker_center_timeout_s, self._marker_search_timeout_s,
         self._marker_descend_speed) = (x.value for x in mp)
        self._marker_cam_yaw_off = math.radians(_marker_cam_yaw_off_deg)

        # Runtime marker state (updated by _cb_markers / used by the MARKER_* phases)
        self._marker_mx = self._marker_my = self._marker_depth = 0.0
        self._marker_stamp_ns     = 0
        self._marker_last_xy      = None
        self._marker_count        = 0
        self._marker_ex = self._marker_ey = 0.0
        self._marker_desc_z       = 0.0
        self._marker_center_since = None
        self._marker_center_start = None
        self._marker_search_start = None
        # Multi-altitude search: look at marker_search_alt for search_step_s, then
        # climb by alt_inc and try again, up to search_max_alt; if still nothing,
        # flow-land. (Higher = wider camera view to find an off-centre marker.)
        self._marker_cur_alt        = 0.0    # current search altitude [m]
        self._marker_search_step_s  = 10.0   # search this long at each altitude [s]
        self._marker_search_alt_inc = 0.5    # climb this much on a timeout [m]
        self._marker_search_max_alt = 3.0    # highest search altitude [m]

        # ── Mission state ─────────────────────────────────────────
        self._phase      = Phase.IDLE
        self._last_phase = None

        # ENU origin locked at arm time
        self._home_x = 0.0
        self._home_y = 0.0
        self._arm_heading_q   = None   # heading captured at ARM (the survey
                                       # reference — applied at altitude)
        self._hold_heading_q  = None   # active heading target (grid reference)
        self._cmd_yaw_rad     = None   # slewed commanded yaw [rad]
        self._hover_home_since = None  # timer for HOVER_HOME arrival hold
        self._yaw_align_start   = None # HOVER_HOME yaw-alignment start time
        self._yaw_aligned_since = None # time the reported yaw first read in-tolerance
        # HOVER_HOME yaw: 'arm' (slew to arm heading) → 'refine' (watch yellow
        # L-corner) → 'yellow' (slew to the L-corner) → SEED. Auto-arm always
        # runs first; the yellow refine only perfects it if a corner is seen.
        self._yaw_stage         = None
        self._yaw_refine_start  = None
        self._yaw_refining      = False   # True only during the refine window
        self._yaw_refine_window_s = 6.0   # watch for the yellow L-corner this long [s]

        # Validation anchor (flow setpoint during VALIDATE)
        self._validate_anchor = (0.0, 0.0)

        # ── Live sensor data ──────────────────────────────────────
        self._pose          = PoseStamped()
        self._pose.pose.orientation.w = 1.0
        self._state         = State()
        self._rc            = ()
        self._vio_state     = 255      # 255 = vio_gate not yet seen
        self._init_factor   = 0.0
        self._rtab_cov      = 0.0      # RTAB odom x-x covariance (0=silent, <100 good, >=100 lost)
        self._pose_stamps   = []       # ring for hz check
        self._last_rtab_ns  = 0
        self._ch5_latched   = False    # safety: CH5 must go HIGH once before LOW arms

        # Image buffer
        self._bridge        = CvBridge()
        self._latest_image  = None    # most recent colour frame (numpy, RGB)

        # ── Survey data ───────────────────────────────────────────
        self._waypoints          = []      # [(x_enu, y_enu), ...]
        self._wpt_idx            = 0       # current target (banked across fault recovery)
        self._capture_lock       = threading.Lock()  # guards checkpoint save + index bump
        self._last_captured_xy   = None   # (x, y) of last successfully captured checkpoint;
                                          # used by GOTO_HOME to return nearby instead of
                                          # all the way to survey origin (which empties the
                                          # F2M local map and causes divergence on the way back)
        self._survey_run_dir     = ""
        self._log_path           = ""
        self._survey_initialized = False   # set once GOTO_HOME completes

        # ── Navigation helpers ────────────────────────────────────
        self._sp             = None      # crawl setpoint [x, y], reset per phase
        self._arrived_since  = None
        self._settle_start   = None
        self._goto_start     = None
        self._desc_z         = 0.0
        self._desc_start     = None

        # ── Phase timers / flags ──────────────────────────────────
        self._at_alt_since    = None
        self._stable_since    = None
        self._seed_sent       = False
        self._seed_start      = None
        self._validate_start  = None
        self._if_good_since   = None
        self._if_low_since    = None
        self._handover_sent   = False
        self._handover_start  = None
        self._flow_hold_start = None
        self._flow_settle_ts  = None
        self._revalidations   = 0
        self._offboard_req    = False
        self._arm_req         = False
        self._gate_close_sent = False
        self._land_req        = False
        self._ctrl_c          = False
        self._last_print      = 0.0

        # ── WhyCode marker-landing state ──────────────────────────
        self._mk_x = self._mk_y = self._mk_z = 0.0  # camera right/down/depth [m]
        self._mk_stamp_ns     = 0
        self._mk_last_xy      = None
        self._mk_detect       = 0     # consecutive accepted detections
        self._mk_ex = self._mk_ey = 0.0   # latest marker position in ENU
        self._land_sp         = None  # ramped XY setpoint during marker landing
        self._mk_search_start = None
        self._mk_center_since = None
        self._mk_center_start = None

        signal.signal(signal.SIGINT, self._sigint)

        # ── ROS wiring ────────────────────────────────────────────
        cb = ReentrantCallbackGroup()
        qos_be  = qos_best_effort()
        qos_rel = qos_reliable()

        self.create_subscription(State,      "/mavros/state",
                                 self._state_cb,     qos_rel, callback_group=cb)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._pose_cb,      qos_be,  callback_group=cb)
        self.create_subscription(RCIn,        "/mavros/rc/in",
                                 self._rc_cb,        qos_be,  callback_group=cb)
        self.create_subscription(Odometry,    rtab_topic,
                                 self._rtab_alive_cb, qos_rel, callback_group=cb)
        self.create_subscription(UInt8,       "/viman/vio_state",
                                 self._vio_state_cb, 10,       callback_group=cb)
        self.create_subscription(Float32,     "/viman/init_factor",
                                 self._if_cb,        10,       callback_group=cb)
        self.create_subscription(Image, "/camera/camera/color/image_raw",
                                 self._image_cb,     qos_be,  callback_group=cb)
        if self._marker_land_enabled:
            self.create_subscription(MarkerArray, self._marker_topic,
                                     self._cb_markers, qos_be, callback_group=cb)

        self._sp_pub  = self.create_publisher(
            PoseStamped,  "/mavros/setpoint_position/local", 10)
        self._vel_pub = self.create_publisher(
            TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10)
        # Compact mission status for the detector's terminal table: "<cmd>|<rtabQ>".
        # Lets the yellow_boundary_detector show the current command + RTAB quality
        # as two extra columns, so the director itself prints nothing per-tick.
        self._status_pub = self.create_publisher(
            String, "/viman/mission/status", 10)

        self._arm_cli  = self.create_client(CommandBool, "/mavros/cmd/arming",
                                            callback_group=cb)
        self._mode_cli = self.create_client(SetMode,     "/mavros/set_mode",
                                            callback_group=cb)
        self._seed_cli = self.create_client(Trigger,     "/viman/seed",
                                            callback_group=cb)
        self._gate_cli = self.create_client(SetBool,     "/viman/gate",
                                            callback_group=cb)

        self._handlers = {
            Phase.IDLE:        self._do_idle,
            Phase.ARM:         self._do_arm,
            Phase.TAKEOFF:     self._do_takeoff,
            Phase.STABLE_OF:   self._do_stable_of,
            Phase.HOVER_HOME:  self._do_hover_home,
            Phase.SEED:        self._do_seed,
            Phase.VALIDATE:    self._do_validate,
            Phase.HANDOVER:    self._do_handover,
            Phase.GOTO_HOME:   self._do_goto_home,
            Phase.SURVEY:      self._do_survey,
            Phase.RETURN:      self._do_return,
            Phase.FLOW_SETTLE: self._do_flow_settle,
            Phase.MARKER_DESCEND: self._do_marker_descend,
            Phase.MARKER_SEARCH:  self._do_marker_search,
            Phase.MARKER_CENTER:  self._do_marker_center,
            Phase.MARKER_LAND:    self._do_marker_land,
            Phase.DESCEND:     self._do_descend,
            Phase.LAND:        self._do_land,
            Phase.DISARM:      self._do_disarm,
            Phase.FLOW_HOLD:   self._do_flow_hold,
            Phase.SAFE_MANUAL: self._do_safe_manual,
            Phase.DONE:        lambda: None,
        }

        self.create_timer(1.0 / self._sp_rate_hz, self._loop, callback_group=cb)

        n_rows = self._n_rows
        n_cols = self._n_cols
        self.get_logger().info(
            f"SurveyMission ready — "
            f"{self._survey_w:.0f}×{self._survey_h:.0f} m survey @ {self._target_alt:.1f} m | "
            f"{n_rows} stripes × {n_cols} cols = {n_rows * n_cols} checkpoints "
            f"(spacing {self._col_spacing:.1f}×{self._stripe_spacing:.1f} m) | "
            f"output → {self._base_survey_dir}/survey_<ts>/\n"
            "SAFETY LATCH: flip CH5 HIGH once, then LOW to start mission.")

    # ─────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────

    def _state_cb(self, m):
        self._state = m

    def _vio_state_cb(self, m):
        self._vio_state = m.data

    def _if_cb(self, m):
        self._init_factor = m.data

    def _rtab_alive_cb(self, msg):
        self._last_rtab_ns = self.get_clock().now().nanoseconds
        # RTAB publishes the odom quality in the pose x-x covariance:
        #   0 = not yet tracking, 0<cov<100 = tracking good, >=100 (99999) = LOST.
        try:
            self._rtab_cov = float(msg.pose.covariance[0])
        except Exception:
            pass

    def _pose_cb(self, m: PoseStamped):
        self._pose = m
        t = self.get_clock().now().nanoseconds
        self._pose_stamps.append(t)
        # keep 2 s window for Hz estimate
        while self._pose_stamps and self._pose_stamps[0] < t - 2_000_000_000:
            self._pose_stamps.pop(0)

    def _image_cb(self, msg: Image):
        """Buffer latest colour frame as RGB numpy array."""
        try:
            self._latest_image = self._bridge.imgmsg_to_cv2(msg, "rgb8")
        except Exception as e:
            self.get_logger().warn(
                f"Image convert failed: {e}", throttle_duration_sec=5.0)

    def _rc_cb(self, m: RCIn):
        self._rc = m.channels
        # latch: CH5 must be seen HIGH at least once before LOW can start
        if self._ch5() >= 1300:
            self._ch5_latched = True
        # CH5 emergency interrupt at any point during flight
        if self._phase in (Phase.SAFE_MANUAL, Phase.DONE, Phase.IDLE):
            return
        if self._ch5() >= self._rc_interrupt_high:
            self.get_logger().warn(
                f"⚠ RC INTERRUPT CH5={self._ch5()} → STABILIZED")
            self._mode("STABILIZED")
            self._phase = Phase.SAFE_MANUAL

    def _sigint(self, *_):
        if self._ctrl_c or self._phase in (
                Phase.IDLE, Phase.DONE, Phase.SAFE_MANUAL, Phase.DISARM):
            raise KeyboardInterrupt
        self.get_logger().warn("Ctrl+C — AUTO.LAND now")
        self._ctrl_c = True

    # ─────────────────────────────────────────────────────────────
    # Main loop
    # ─────────────────────────────────────────────────────────────

    def _loop(self):
        if self._phase != self._last_phase:
            self.get_logger().info(f"══ Phase: {self._phase.name} ══")
            self._last_phase = self._phase
            # reset per-phase navigation state on every transition
            self._sp = None

        if self._ctrl_c and self._phase not in (
                Phase.LAND, Phase.DISARM, Phase.DONE, Phase.SAFE_MANUAL):
            self._phase = Phase.LAND
            return

        self._handlers[self._phase]()

    # ─────────────────────────────────────────────────────────────
    # Lawnmower geometry
    # ─────────────────────────────────────────────────────────────

    def _build_waypoints(self):
        """Return boustrophedon checkpoint list aligned to the drone's arm-time heading.

        Grid axes (body frame, rotated into ENU by _cmd_yaw_rad):
          Rows  → drone FORWARD direction  (the direction the camera faces at arm time)
          Cols  → drone LEFT direction     (90° CCW from forward)
          Odd rows reversed for lawnmower (boustrophedon) pattern.

        _cmd_yaw_rad tracks _hold_heading_q, which is the drone's ARM-time
        heading (captured automatically at arming, applied at altitude). So the
        grid is always relative to where the drone was pointing when armed —
        not a hard-coded compass value, not magnetic north.
        """
        yaw = self._cmd_yaw_rad          # VIO-frame yaw set at HANDOVER

        # Forward unit vector (direction drone is facing)
        fwd_x =  math.cos(yaw)
        fwd_y =  math.sin(yaw)
        # Right unit vector (90° CW from forward — cols sweep right)
        left_x =  math.sin(yaw)
        left_y = -math.cos(yaw)

        n_rows = self._n_rows
        n_cols = self._n_cols
        wps = []
        # Outer loop = cols (stripes offset to the left)
        # Inner loop = rows (forward sweeps) — so FIRST movement is always forward
        for c in range(n_cols):
            row_range = list(range(n_rows))
            if c % 2 == 1:
                row_range = row_range[::-1]          # reverse odd stripes for lawnmower
            for row in row_range:
                x_enu = self._home_x + row * self._stripe_spacing * fwd_x \
                                     + c   * self._col_spacing    * left_x
                y_enu = self._home_y + row * self._stripe_spacing * fwd_y \
                                     + c   * self._col_spacing    * left_y
                wps.append((x_enu, y_enu))
        self.get_logger().info(
            f"Lawnmower: {n_rows} rows × {n_cols} cols = {len(wps)} checkpoints "
            f"| area {self._survey_w:.1f}×{self._survey_h:.1f} m "
            f"| origin ({self._home_x:.2f}, {self._home_y:.2f}) "
            f"| grid aligned to arm-time heading {math.degrees(yaw):.1f}° "
            f"(forward=({fwd_x:.2f},{fwd_y:.2f}), left=({left_x:.2f},{left_y:.2f}))")
        return wps

    @property
    def _n_cols(self):
        return max(1, round(self._survey_w / self._col_spacing)) + 1

    @property
    def _n_rows(self):
        return max(1, round(self._survey_h / self._stripe_spacing)) + 1

    def _wpt_to_rowcol(self, idx: int):
        """Map flat waypoint index → (row, col).

        Must mirror _build_waypoints' ordering EXACTLY: the outer loop is
        columns (stripes) and the inner loop is rows (n_rows per column), with
        odd columns sweeping their rows in reverse (boustrophedon). The flat
        index therefore groups by n_rows, not n_cols. The original code divided
        by n_cols, so every row/col label in coordinates.csv and the JPEG
        filenames was wrong whenever the grid was not square.
        """
        n_rows = self._n_rows
        col = idx // n_rows
        pos = idx % n_rows
        row = (n_rows - 1 - pos) if (col % 2 == 1) else pos
        return row, col

    # ─────────────────────────────────────────────────────────────
    # Survey storage
    # ─────────────────────────────────────────────────────────────

    def _init_survey_storage(self):
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._survey_run_dir = os.path.join(self._base_survey_dir, f"survey_{ts}")
        os.makedirs(self._survey_run_dir, exist_ok=True)
        self._log_path = os.path.join(self._survey_run_dir, "coordinates.csv")
        with open(self._log_path, 'w', newline='') as f:
            csv.writer(f).writerow([
                "checkpoint", "row", "col",
                "timestamp_s",
                "x_enu", "y_enu", "z_enu",
                "yaw_deg",
                "image_file",
            ])
        self.get_logger().info(
            f"Survey output dir: {self._survey_run_dir}  "
            f"({len(self._waypoints)} checkpoints)")

    def _capture_checkpoint(self, idx: int) -> bool:
        """Save JPEG + CSV row for checkpoint idx.  Returns True if image saved."""
        row, col = self._wpt_to_rowcol(idx)
        img_name = f"cp{idx:04d}_r{row:02d}c{col:02d}.jpg"
        img_path = os.path.join(self._survey_run_dir, img_name)

        saved = False
        if self._latest_image is not None:
            try:
                bgr = cv2.cvtColor(self._latest_image, cv2.COLOR_RGB2BGR)
                cv2.imwrite(img_path, bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
                saved = True
            except Exception as e:
                self.get_logger().error(f"imwrite cp{idx}: {e}")
                img_name = "WRITE_FAILED"
        else:
            self.get_logger().warn(f"No image frame available for cp{idx} — coords only")
            img_name = "NO_FRAME"

        p   = self._pose.pose.position
        yaw = yaw_deg_from_quaternion(self._pose.pose.orientation)
        ts  = self.get_clock().now().nanoseconds / 1e9

        with open(self._log_path, 'a', newline='') as f:
            csv.writer(f).writerow([
                idx, row, col,
                f"{ts:.3f}",
                f"{p.x:.4f}", f"{p.y:.4f}", f"{p.z:.4f}",
                f"{yaw:.1f}",
                img_name,
            ])

        status = "✓ saved" if saved else "✗ no image"
        self.get_logger().info(
            f"[cp {idx:04d}] row={row} col={col}  "
            f"pos=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})  yaw={yaw:.1f}°  {status}")
        return saved

    # ─────────────────────────────────────────────────────────────
    # Preflight
    # ─────────────────────────────────────────────────────────────

    def _preflight_failures(self):
        fails = []
        if not self._state.connected:
            fails.append("FCU")
        # pose rate check (need ~15 Hz minimum)
        hz = len(self._pose_stamps) / 2.0
        if hz < self._pose_hz_min:
            fails.append(f"pose {hz:.0f}<{self._pose_hz_min:.0f} Hz")
        if len(self._rc) <= self._rc_ch5_idx:
            fails.append("no RC")
        if self.get_clock().now().nanoseconds - self._last_rtab_ns > 2_000_000_000:
            fails.append("RTAB silent")
        if self._vio_state == 255:
            fails.append("vio_gate silent")
        if not self._seed_cli.service_is_ready():
            fails.append("/viman/seed")
        if not self._arm_cli.service_is_ready() or not self._mode_cli.service_is_ready():
            fails.append("MAVROS svc")
        return fails

    # ─────────────────────────────────────────────────────────────
    # Phase handlers — pre-survey (identical flow to square_mission)
    # ─────────────────────────────────────────────────────────────

    def _do_idle(self):
        self._pub_sp(0.0, 0.0, 0.3)
        fails = self._preflight_failures()
        if fails:
            self.get_logger().warn(
                "PREFLIGHT BLOCKED: " + ", ".join(fails),
                throttle_duration_sec=5.0)
            return
        if not self._ch5_latched:
            self.get_logger().info(
                "Preflight OK — flip CH5 HIGH once then LOW to arm",
                throttle_duration_sec=5.0)
            return
        if self._ch5() <= self._rc_start_low:
            self._offboard_req = False
            self._arm_req      = False
            self._phase        = Phase.ARM

    def _do_arm(self):
        self._pub_sp(0.0, 0.0, 0.3)
        if not self._offboard_req:
            self._mode("OFFBOARD")
            self._offboard_req = True
            return
        if self._state.mode != "OFFBOARD":
            self._mode("OFFBOARD")
            return
        if not self._arm_req:
            self._arm(True)
            self._arm_req = True
            return
        if self._state.armed:
            self._home_x = self._pose.pose.position.x
            self._home_y = self._pose.pose.position.y
            # Capture the ARM-time heading NOW — this is the automatic survey
            # reference (no hard-coded compass value). We do NOT start holding
            # it yet: the heading refs stay None so _pub_sp commands the live
            # estimate (zero yaw error → no rotation) and the drone climbs
            # straight. This matters because on the ground the EKF heading is
            # magnetometer-only and the high motor current during the climb
            # triggers an EKF2 mag yaw reset of ~10-15°; holding a rigid yaw
            # setpoint through that would make the airframe chase the reset
            # (the sudden yaw snap on the way up). The captured arm heading is
            # applied later, at altitude, by slewing gently onto it.
            self._arm_heading_q  = self._pose.pose.orientation
            self._hold_heading_q = None
            self._cmd_yaw_rad    = None
            self.get_logger().info(
                f"Armed. HOME = ({self._home_x:.3f}, {self._home_y:.3f})  "
                f"arm-time heading = "
                f"{yaw_deg_from_quaternion(self._arm_heading_q):.1f}° "
                f"(survey reference; yaw floats until altitude — no snap)")
            self._phase = Phase.TAKEOFF

    def _do_takeoff(self):
        self._pub_sp(self._home_x, self._home_y, self._target_alt)
        alt = self._pose.pose.position.z
        self._tele(f"TAKEOFF  {alt:.2f} / {self._target_alt:.1f} m")
        if abs(alt - self._target_alt) <= self._alt_tol:
            if self._at_alt_since is None:
                self._at_alt_since = self.get_clock().now()
            elif self._secs(self._at_alt_since) >= self._at_alt_confirm_s:
                self._stable_since = self.get_clock().now()
                self._phase = Phase.STABLE_OF
        else:
            self._at_alt_since = None

    def _do_stable_of(self):
        """Zero-velocity hold on optical flow to let EKF settle."""
        self._pub_vel_hold()
        if self._secs(self._stable_since) >= self._stable_of_secs:
            self.get_logger().info(
                f"Stable. Flying to arm point "
                f"({self._home_x:.2f}, {self._home_y:.2f}) to correct takeoff drift.")
            self._hover_home_since = None
            self._phase = Phase.HOVER_HOME

    @staticmethod
    def _yaw_quat(yaw_rad):
        """Quaternion (about +Z) for a target yaw in the MAVROS/ENU frame."""
        q = Quaternion()
        q.w = math.cos(yaw_rad / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw_rad / 2.0)
        return q

    def _yaw_align_override(self):
        """Hook: return a yaw [rad] to lock at HOVER_HOME, or None to use the
        arm-time/fixed heading. Base returns None (plain survey is unchanged);
        the boundary-aware subclass overrides this to align to a yellow
        L-corner when two perpendicular lines are visible."""
        return None

    def _yaw_refine_enabled(self):
        """Hook: True if the yellow-line refine window should run after the
        arm-time yaw lock. Base = False (plain survey seeds straight after arm);
        the boundary-aware subclass returns True."""
        return False

    def _do_hover_home(self):
        """Fly to arm point at survey altitude, settle, slew onto the heading,
        HOLD until the reported yaw is actually aligned, then RE-LOCK the grid
        frame to the drone's ACTUAL settled yaw before seeding.

        Corrects lateral drift accumulated during takeoff and gives the cleanest
        possible grid reference — the same two-stage align-and-relock scheme as
        boundary_test_auto. Stage 1 gets home + settles + applies the heading;
        stage 2 waits for the slew to actually reach it, then anchors the grid
        to the real settled yaw so every survey stripe flies straight along the
        drone's own body axes instead of skewing by the residual PX4 yaw error.
        """
        self._pub_sp(self._home_x, self._home_y, self._target_alt)

        px   = self._pose.pose.position.x
        py   = self._pose.pose.position.y
        dist = math.hypot(px - self._home_x, py - self._home_y)

        # ── Stage 1: get home + settle, then apply the ARM-time heading ──
        # Auto-arm yaw ALWAYS runs first (reliable baseline); the yellow L-corner
        # only refines it later, in the stage-3 window.
        if self._hold_heading_q is None:
            self._tele(f"HOVER_HOME  dist={dist:.2f} m → "
                       f"({self._home_x:.2f},{self._home_y:.2f})")
            if dist <= self._goto_radius:
                if self._hover_home_since is None:
                    self._hover_home_since = self.get_clock().now()
                elif self._secs(self._hover_home_since) >= 2.0:
                    if self._yaw_use_arm_heading and self._arm_heading_q is not None:
                        self._hold_heading_q = self._arm_heading_q
                        src = "ARM-time"
                    else:
                        self._hold_heading_q = self._yaw_quat(
                            math.radians(self._mission_yaw_deg))
                        src = f"fixed {self._mission_yaw_deg:.1f}°"
                    self._cmd_yaw_rad       = math.radians(
                        yaw_deg_from_quaternion(self._pose.pose.orientation))
                    self._yaw_align_start   = self.get_clock().now()
                    self._yaw_aligned_since = None
                    self._yaw_stage         = "arm"
                    self.get_logger().info(
                        f"Arm point reached, settled. Slewing onto {src} heading "
                        f"{yaw_deg_from_quaternion(self._hold_heading_q):.1f}° "
                        "(auto-arm correction), holding until aligned.")
            else:
                self._hover_home_since = None
            return

        # ── Stage 3: yellow L-corner REFINE window (after the arm re-lock) ──
        if self._yaw_stage == "refine":
            ovr = self._yaw_align_override()
            if ovr is not None:
                # A stable yellow L-corner appeared → re-slew onto it, re-lock.
                self._hold_heading_q    = self._yaw_quat(ovr)
                self._cmd_yaw_rad       = math.radians(
                    yaw_deg_from_quaternion(self._pose.pose.orientation))
                self._yaw_align_start   = self.get_clock().now()
                self._yaw_aligned_since = None
                self._yaw_stage         = "yellow"
                self.get_logger().info(
                    f"Yellow L-corner found ({math.degrees(ovr):.1f}°) — refining "
                    "yaw onto it (arm baseline already locked).")
            elif self._secs(self._yaw_refine_start) > self._yaw_refine_window_s:
                self.get_logger().info(
                    f"No yellow L-corner in {self._yaw_refine_window_s:.0f} s — "
                    "keeping the arm-time yaw. Seeding.")
                self._yaw_refining = False
                self._seed_sent    = False
                self._phase        = Phase.SEED
            else:
                self._tele(
                    "HOVER_HOME  refine — watching for yellow L-corner "
                    f"({self._secs(self._yaw_refine_start):.1f}/"
                    f"{self._yaw_refine_window_s:.0f} s)")
            return

        # ── Stage 2 / 4: slew onto _hold_heading_q, wait aligned, RE-LOCK the
        #    grid to the ACTUAL settled yaw. Used for BOTH the arm pass and the
        #    yellow pass; the branch after the lock decides what comes next. ──
        cur = yaw_deg_from_quaternion(self._pose.pose.orientation)
        tgt = yaw_deg_from_quaternion(self._hold_heading_q)
        err = abs((cur - tgt + 180.0) % 360.0 - 180.0)
        self._tele(f"HOVER_HOME  [{self._yaw_stage}] yaw={cur:.1f} → {tgt:.1f}° "
                   f"(err={err:.1f}°)")

        if err <= self._yaw_align_tol_deg:
            if self._yaw_aligned_since is None:
                self._yaw_aligned_since = self.get_clock().now()
        else:
            self._yaw_aligned_since = None
        aligned = (self._yaw_aligned_since is not None and
                   self._secs(self._yaw_aligned_since) >= self._yaw_align_hold_s)
        timeout = self._secs(self._yaw_align_start) > self._yaw_align_timeout_s
        if aligned or timeout:
            actual_yaw_rad       = math.radians(cur)
            self._cmd_yaw_rad    = actual_yaw_rad
            self._hold_heading_q = self._yaw_quat(actual_yaw_rad)
            done = 'aligned' if aligned else 'timeout — proceeding anyway'
            if self._yaw_stage == "arm" and self._yaw_refine_enabled():
                # Arm baseline locked → OPEN the yellow-refine window.
                self.get_logger().info(
                    f"Yaw locked at ACTUAL {cur:.1f}° (arm baseline, {done}). "
                    f"Watching for a yellow L-corner {self._yaw_refine_window_s:.0f} s "
                    "to refine.")
                self._yaw_stage        = "refine"
                self._yaw_refine_start = self.get_clock().now()
                self._yaw_refining     = True
                self._yaw_est_buf      = []   # fresh samples for the refine window
            else:
                what = ("yellow L-corner" if self._yaw_stage == "yellow"
                        else "arm baseline")
                self.get_logger().info(
                    f"Yaw locked at ACTUAL {cur:.1f}° ({what}, {done}). Grid frame "
                    "anchored here — stripes fly straight along the real body "
                    "axes. Seeding.")
                self._yaw_refining = False
                self._seed_sent    = False
                self._phase        = Phase.SEED

    def _do_seed(self):
        """Trigger RTAB odometry reset + seed vio_gate."""
        self._pub_vel_hold()
        if not self._seed_sent:
            if not self._seed_cli.service_is_ready():
                return
            self._seed_cli.call_async(Trigger.Request())
            self._seed_sent = True
            self._seed_start = self.get_clock().now()
            return
        if self._vio_state == GS_VALIDATING:
            self._validate_start  = self.get_clock().now()
            self._if_good_since   = None
            self._if_low_since    = None
            self._validate_anchor = (self._pose.pose.position.x,
                                     self._pose.pose.position.y)
            self._phase = Phase.VALIDATE
        elif self._secs(self._seed_start) > self._seed_timeout_s:
            self.get_logger().warn("Seed timeout — flow hold, then retry")
            self._flow_hold_start = None
            self._phase           = Phase.FLOW_HOLD

    def _motion_offset(self):
        """Tiny square pattern during VALIDATE to force non-trivial motion."""
        if not self._motion_test:
            return 0.0, 0.0
        corners = (
            (0.0,               0.0),
            (self._motion_amp,  0.0),
            (self._motion_amp,  self._motion_amp),
            (0.0,               self._motion_amp),
        )
        leg = int(self._secs(self._validate_start) / self._motion_leg_s)
        return corners[leg % 4]

    def _do_validate(self):
        ax, ay = self._validate_anchor
        ox, oy = self._motion_offset()
        self._pub_sp(ax + ox, ay + oy, self._target_alt)

        if self._vio_state >= GS_FAULT_MIN:
            self.get_logger().warn("VIO fault during VALIDATE — flow hold")
            self._flow_hold_start = None
            self._phase           = Phase.FLOW_HOLD
            return

        el = self._secs(self._validate_start)
        if self._init_factor >= self._if_min:
            self._if_low_since = None
            if self._if_good_since is None:
                self._if_good_since = self.get_clock().now()
            if self._secs(self._if_good_since) >= self._validate_hold_s:
                self._handover_sent  = False
                self._handover_start = None
                self._phase          = Phase.HANDOVER
                return
        elif self._if_good_since is not None:
            if self._if_low_since is None:
                self._if_low_since = self.get_clock().now()
            elif self._secs(self._if_low_since) > self._dip_grace_s:
                self._if_good_since = self._if_low_since = None

        self._tele(
            f"VALIDATE  IF={self._init_factor:.2f} (need ≥{self._if_min:.2f} "
            f"for {self._validate_hold_s:.0f}s)  t={el:.0f}s")

        if el > self._validate_timeout_s:
            self.get_logger().error("Validation timeout — landing on flow")
            self._phase = Phase.LAND

    def _do_handover(self):
        ax, ay = self._validate_anchor
        self._pub_sp(ax, ay, self._target_alt)

        if not self._handover_sent:
            if not self._gate_cli.service_is_ready():
                return
            req = SetBool.Request()
            req.data = True
            self._gate_cli.call_async(req)
            self._handover_sent  = True
            self._handover_start = self.get_clock().now()
            return

        if self._vio_state == GS_OPEN:
            if self._secs(self._handover_start) >= self._handover_settle_s:
                # Keep the ARM-time heading as the survey reference — it is
                # already the active target and the drone has slewed onto it
                # since HOVER_HOME. We deliberately do NOT re-capture from the
                # VIO pose here: the whole point is that the grid aligns to the
                # heading the drone had at arming. (Grid-building and heading-
                # hold both use _hold_heading_q, so they stay mutually
                # consistent even if the VIO frame differs by a few degrees.)
                if not self._survey_initialized:
                    self.get_logger().info(
                        f"Gate OPEN — holding ARM-time survey heading "
                        f"{yaw_deg_from_quaternion(self._hold_heading_q):.1f}°.")

                if self._survey_initialized:
                    # Survey already in progress — skip GOTO_HOME entirely.
                    # After each reseed the F2M local map starts fresh, so
                    # crawling back to the last-checkpoint zone gains nothing
                    # and risks quality collapse over low-texture ground
                    # (seen in flight: quality 300→25 during GOTO_HOME approach,
                    # causing a second cov_spike fault before resuming).
                    # _do_survey() crawls to the next waypoint from current pos.
                    self.get_logger().info(
                        f"Gate OPEN — VIO active. Resuming survey at "
                        f"cp{self._wpt_idx} from current position "
                        f"(GOTO_HOME skipped — survey already initialized).")
                    self._begin_survey()
                else:
                    self.get_logger().info(
                        f"Gate OPEN — VIO active. Flying to survey origin "
                        f"({self._home_x:.2f}, {self._home_y:.2f})")
                    self._goto_start    = None
                    self._arrived_since = None
                    self._phase         = Phase.GOTO_HOME
        elif self._secs(self._handover_start) > 5.0:
            self.get_logger().warn("Gate did not open — flow hold")
            self._flow_hold_start = None
            self._phase           = Phase.FLOW_HOLD

    # ─────────────────────────────────────────────────────────────
    # Phase handlers — survey
    # ─────────────────────────────────────────────────────────────

    def _do_goto_home(self):
        """Fly to resume position before (re)starting the survey grid.

        First entry (no checkpoints captured yet):
            → fly to survey origin (home_x, home_y) to anchor the grid.

        Fault recovery (at least one checkpoint already captured):
            → fly to the LAST CAPTURED checkpoint position instead.
            Rationale: the F2M local map covers recent survey frames. Flying
            all the way back to origin empties the map → feature matches fail
            → quality drops → divergence. Going to the previous checkpoint
            keeps the drone inside its own local map → stable quality.
        """
        # pick resume target
        if self._last_captured_xy is not None:
            tx, ty = self._last_captured_xy
        else:
            tx, ty = self._home_x, self._home_y

        if self._vio_state >= GS_FAULT_MIN:
            self.get_logger().error("VIO fault during GOTO_HOME — flow hold")
            self._flow_hold_start = None
            self._phase           = Phase.FLOW_HOLD
            return
        if self._goto_start is None:
            self._goto_start = self.get_clock().now()

        if self._secs(self._goto_start) > self._goto_timeout:
            self.get_logger().warn("GOTO_HOME timeout — starting survey from current position")
            if not self._survey_initialized:
                # only update origin on first entry; preserve home for RETURN
                px = self._pose.pose.position.x
                py = self._pose.pose.position.y
                self._home_x = px
                self._home_y = py
            self._begin_survey()
            return

        dist = self._crawl_to(tx, ty)
        self._tele(f"GOTO_HOME  dist={dist:.2f} m")

        if dist <= self._goto_radius:
            if self._arrived_since is None:
                self._arrived_since = self.get_clock().now()
            elif self._secs(self._arrived_since) >= 1.0:
                self._arrived_since = None
                self._goto_start    = None
                self._begin_survey()

    def _begin_survey(self):
        """One-time survey init (idempotent — safe to call on re-entry after fault)."""
        if not self._survey_initialized:
            self._waypoints          = self._build_waypoints()
            self._wpt_idx            = 0
            self._init_survey_storage()
            self._survey_initialized = True
        else:
            self.get_logger().info(
                f"Survey resuming from cp{self._wpt_idx} / {len(self._waypoints)}")

        self._sp            = None
        self._arrived_since = None
        self._settle_start  = None
        self._phase         = Phase.SURVEY

    def _do_survey(self):
        """Lawnmower loop.

        Each checkpoint goes through two sub-stages:
          1. navigate  — crawl setpoint toward (tx, ty) until within radius
          2. settle    — hold position for waypoint_settle_s, then capture
        """
        if self._vio_state >= GS_FAULT_MIN:
            self.get_logger().error(
                f"VIO fault at cp{self._wpt_idx} — flow hold. "
                "Survey will RESUME from this checkpoint after revalidation.")
            self._flow_hold_start = None
            self._phase           = Phase.FLOW_HOLD
            return

        # All checkpoints done?
        if self._wpt_idx >= len(self._waypoints):
            self.get_logger().info(
                f"Survey COMPLETE — {len(self._waypoints)} checkpoints captured. Returning.")
            self._arrived_since = None
            self._goto_start    = None
            self._phase         = Phase.RETURN
            return

        tx, ty = self._waypoints[self._wpt_idx]
        total  = len(self._waypoints)

        # ── Sub-stage 1: navigate ─────────────────────────────
        if self._settle_start is None:
            dist = self._crawl_to(tx, ty)
            row, col = self._wpt_to_rowcol(self._wpt_idx)
            self._tele(
                f"SURVEY  cp{self._wpt_idx:04d}/{total-1}  "
                f"row={row} col={col}  "
                f"→ ({tx:.1f},{ty:.1f})  dist={dist:.2f} m")

            if dist <= self._wpt_radius:
                if self._arrived_since is None:
                    self._arrived_since = self.get_clock().now()
                elif self._secs(self._arrived_since) >= 0.3:
                    # arrived — start settle timer
                    self._settle_start  = self.get_clock().now()
                    self._arrived_since = None
            else:
                self._arrived_since = None
            return

        # ── Sub-stage 2: settle + capture ─────────────────────
        self._pub_sp(tx, ty, self._target_alt)   # hard hold during settle
        elapsed = self._secs(self._settle_start)
        settle_dur = self._wpt_settle_s
        self._tele(
            f"SURVEY  cp{self._wpt_idx:04d}  SETTLING  "
            f"{elapsed:.1f}/{settle_dur:.1f} s")

        if elapsed >= settle_dur:
            # Lock prevents a concurrent timer callback (MultiThreadedExecutor +
            # ReentrantCallbackGroup) from double-saving and double-incrementing.
            with self._capture_lock:
                if self._settle_start is None:
                    return  # sibling callback already handled this checkpoint
                snap_idx           = self._wpt_idx
                self._settle_start = None   # mark done — blocks re-entry
                # Anchor the next segment's ramp to the IDEAL waypoint position,
                # not the drone's actual (drifted) position.  This ensures each
                # inter-waypoint path is a true straight line on the grid axis
                # rather than a slightly-angled line from wherever the drone settled.
                self._sp           = list(self._waypoints[snap_idx])
                self._wpt_idx      = snap_idx + 1
            # Heavy I/O (imwrite) runs outside the lock so we don't block pose callbacks
            self._capture_checkpoint(snap_idx)
            self._last_captured_xy = self._waypoints[snap_idx]

    # ─────────────────────────────────────────────────────────────
    # Phase handlers — landing sequence (mirrors mission_director)
    # ─────────────────────────────────────────────────────────────

    def _do_return(self):
        """Crawl home on VIO, then hand off to flow-only landing."""
        if self._vio_state >= GS_FAULT_MIN:
            self.get_logger().warn("VIO fault during return — landing here on flow")
            self._phase = Phase.LAND
            return
        if self._goto_start is None:
            self._goto_start = self.get_clock().now()
        if self._secs(self._goto_start) > self._goto_timeout:
            self.get_logger().warn("Return timeout — landing on flow")
            self._phase = Phase.LAND
            return

        dist = self._crawl_to(self._home_x, self._home_y)
        self._tele(f"RETURN  dist={dist:.2f} m → home")

        if dist <= self._goto_radius:
            if self._arrived_since is None:
                self._arrived_since = self.get_clock().now()
            elif self._secs(self._arrived_since) >= 1.0:
                # close gate before descent — camera should not feed landing
                if self._gate_cli.service_is_ready():
                    req = SetBool.Request()
                    req.data = False
                    self._gate_cli.call_async(req)
                self._gate_close_sent = True
                self._flow_settle_ts  = self.get_clock().now()
                self.get_logger().info("Home ✓ — gate closed, entering flow settle")
                self._arrived_since = None
                self._goto_start    = None
                self._phase         = Phase.FLOW_SETTLE
        else:
            self._arrived_since = None

    def _do_flow_settle(self):
        """Hold at home altitude on flow for flow_settle_s, then either start the
        WhyCode marker landing (if enabled) or the plain flow-only descent."""
        self._pub_sp(self._home_x, self._home_y, self._target_alt)
        if self._secs(self._flow_settle_ts) >= self._flow_settle_s:
            self._desc_z     = self._pose.pose.position.z
            self._desc_start = self.get_clock().now()
            if self._marker_land_enabled:
                self._marker_count = 0
                # TOP-DOWN marker search: ALWAYS start at the TOP of the ladder
                # (max_alt, widest FOV) — climbing there first if the survey/return
                # altitude is lower — then step DOWN on each 10 s miss
                # (3.0 → 2.5 → 2.0 m). The wide FOV up high catches a laterally-
                # offset marker that falls outside the frame lower down; searching
                # only at the (lower) return altitude used to miss it entirely.
                # max_alt is kept below the detector's active altitude so it runs.
                self._marker_cur_alt      = self._marker_search_max_alt
                self._marker_search_start = self.get_clock().now()
                self.get_logger().info(
                    f"Flow settled — holding at {self._marker_cur_alt:.1f} m and "
                    "searching for the WhyCode marker, stepping DOWN "
                    f"({self._marker_search_max_alt:.1f} → "
                    f"{self._marker_search_alt:.1f} m) if not seen.")
                self._phase = Phase.MARKER_SEARCH
            else:
                self._phase = Phase.DESCEND

    def _do_descend(self):
        """Controlled descend at descend_speed_ms on flow; hand off at handoff alt."""
        self._desc_z = max(0.0, self._desc_z - self._descend_speed / self._sp_rate_hz)
        self._pub_sp(self._home_x, self._home_y, self._desc_z)
        alt  = self._pose.pose.position.z
        ex   = self._pose.pose.position.x - self._home_x
        ey   = self._pose.pose.position.y - self._home_y
        self._tele(
            f"DESCEND  alt={alt:.2f} → {self._descend_handoff_alt:.2f} m  "
            f"horiz off=({ex:+.2f},{ey:+.2f})")
        if alt <= self._descend_handoff_alt:
            self._phase = Phase.LAND
        elif self._secs(self._desc_start) > self._descend_timeout:
            self.get_logger().warn("Descend timeout — AUTO.LAND")
            self._phase = Phase.LAND

    # ── WhyCode precision landing ─────────────────────────────────────

    def _cb_markers(self, m: MarkerArray):
        """Track the marker nearest the image centre. WhyCode camera frame:
        position.x=depth, y=left, z=up → store as camera right/down/depth with
        an altitude gate and a lone-outlier reject."""
        if not m.markers:
            return
        best = min(m.markers,
                   key=lambda mk: abs(mk.position.position.y)
                                + abs(mk.position.position.z))
        depth =  best.position.position.x
        mx    = -best.position.position.y     # camera right
        my    = -best.position.position.z     # camera down
        alt = self._pose.pose.position.z
        # Gate 1: camera depth must match the drone's height (downward cam).
        if self._marker_alt_gate > 0.0 and alt > 0.3 \
                and abs(depth - alt) > self._marker_alt_gate:
            return
        # Gate 2: reject a lone frame-to-frame jump while still fresh.
        if self._marker_last_xy is not None and self._marker_stamp_ns != 0:
            age = (self.get_clock().now().nanoseconds
                   - self._marker_stamp_ns) / 1e9
            if age <= self._marker_timeout_s and math.hypot(
                    mx - self._marker_last_xy[0],
                    my - self._marker_last_xy[1]) > self._marker_outlier:
                return
        self._marker_mx, self._marker_my, self._marker_depth = mx, my, depth
        self._marker_last_xy = (mx, my)
        st = m.header.stamp
        self._marker_stamp_ns = (st.sec * 1_000_000_000 + st.nanosec) \
            if (st.sec or st.nanosec) else self.get_clock().now().nanoseconds

    def _marker_enu_offset(self):
        """ENU offset (dx, dy) from the drone to the marker, or None if the
        latest detection is stale (> marker_timeout_s)."""
        if self._marker_stamp_ns == 0:
            return None
        age = (self.get_clock().now().nanoseconds - self._marker_stamp_ns) / 1e9
        if age > self._marker_timeout_s:
            return None
        tx = self._marker_mx * self._marker_cam_x_sign
        ty = self._marker_my * self._marker_cam_y_sign
        # Camera MOUNTING rotation about the optical axis (marker_cam_yaw_offset_deg):
        # rotate the camera axes onto the drone body axes. Without this, centring
        # drives the wrong way / orbits the marker. Matches the working whycode
        # landing (your camera needs 270°).
        co = math.cos(self._marker_cam_yaw_off)
        so = math.sin(self._marker_cam_yaw_off)
        bx = co * tx - so * ty
        by = so * tx + co * ty
        # Rotate body axes into ENU using the live drone yaw.
        yaw = math.radians(yaw_deg_from_quaternion(self._pose.pose.orientation))
        dx = math.cos(yaw) * bx - math.sin(yaw) * by
        dy = math.sin(yaw) * bx + math.cos(yaw) * by
        return (dx, dy)

    def _do_marker_descend(self):
        """Sink from home altitude down to marker_search_alt over the home point.

        Keeps an eye out for the marker on the way down — if it already shows up
        before reaching search altitude, jump straight to centring.
        """
        self._desc_z = max(self._marker_search_alt,
                           self._desc_z - self._descend_speed / self._sp_rate_hz)
        self._pub_sp(self._home_x, self._home_y, self._desc_z)
        alt = self._pose.pose.position.z
        self._tele(
            f"MARKER_DESCEND  alt={alt:.2f} → {self._marker_search_alt:.2f} m")

        # Descend ALL the way to the search altitude BEFORE looking — no early
        # commit mid-descent. A marginal high-altitude detection (marker small
        # and off to the side at ~3 m) used to lock a bad fix before the drone
        # had settled. Reaching 2 m first lets the altitude ladder
        # (search 2 m → climb 2.5 m → climb 3 m) start cleanly from the lowest,
        # most reliable search height.
        if alt <= self._marker_search_alt + self._alt_tol:
            self._marker_count        = 0
            self._marker_cur_alt      = self._marker_search_alt   # start search at 2 m
            self._marker_search_start = self.get_clock().now()
            self._phase               = Phase.MARKER_SEARCH

    def _do_marker_search(self):
        """Hold at search altitude over home, wait for stable marker detections.

        Confirmed → lock the marker ENU position and start MARKER_LAND.
        Timeout  → give up on the marker, fall back to flow-only DESCEND.
        """
        self._pub_sp(self._home_x, self._home_y, self._marker_cur_alt)
        alt = self._pose.pose.position.z
        # Reach the target search altitude FIRST. While still climbing/settling,
        # don't count detections and hold the 10 s window at zero, so each rung of
        # the ladder gets a full 10 s search AT that altitude (not mid-climb).
        if abs(alt - self._marker_cur_alt) > self._alt_tol:
            self._marker_count        = 0
            self._marker_search_start = self.get_clock().now()
            self._tele(f"MARKER_SEARCH → settling at {self._marker_cur_alt:.1f} m "
                       f"(alt={alt:.2f})")
            return
        off = self._marker_enu_offset()
        if off is None:
            self._marker_count = 0
            self._tele(f"MARKER_SEARCH @ {self._marker_cur_alt:.1f} m — no marker "
                       f"({self._secs(self._marker_search_start):.0f}/"
                       f"{self._marker_search_step_s:.0f}s)")
            if self._secs(self._marker_search_start) > self._marker_search_step_s:
                nxt = self._marker_cur_alt - self._marker_search_alt_inc
                if nxt >= self._marker_search_alt - 1e-3:
                    self.get_logger().info(
                        f"No marker at {self._marker_cur_alt:.1f} m after "
                        f"{self._marker_search_step_s:.0f} s — descending to "
                        f"{nxt:.1f} m to search (wider→closer).")
                    self._marker_cur_alt      = nxt
                    self._marker_search_start = self.get_clock().now()
                else:
                    self.get_logger().warn(
                        f"No WhyCode marker down to {self._marker_search_alt:.1f} m "
                        "— normal flow-only landing at home.")
                    self._desc_z     = self._pose.pose.position.z
                    self._desc_start = self.get_clock().now()
                    self._phase      = Phase.DESCEND
            return

        self._marker_count += 1
        px, py = self._pose.pose.position.x, self._pose.pose.position.y
        self._marker_ex = px + off[0]
        self._marker_ey = py + off[1]
        self._tele(
            f"MARKER_SEARCH — marker offset=({off[0]:+.2f},{off[1]:+.2f}) m "
            f"frame {self._marker_count}/{self._marker_detect_frames}")

        if self._marker_count >= self._marker_detect_frames:
            self.get_logger().info(
                f"Marker confirmed → ENU ({self._marker_ex:.2f},{self._marker_ey:.2f}). "
                "Centring at search altitude before descent.")
            self._marker_center_since = None
            self._marker_center_start = self.get_clock().now()
            self._phase               = Phase.MARKER_CENTER

    def _do_marker_center(self):
        """Hold at search altitude and slide laterally onto the marker.

        Stays at marker_search_alt (no descent) and crawls toward the marker
        until the lateral error is within marker_center_thr for
        marker_center_hold_s — only THEN does it start descending (MARKER_LAND).
        If it can't centre within marker_center_timeout_s it descends anyway so
        the drone never gets stuck hovering.
        """
        off = self._marker_enu_offset()
        if off is not None:
            px, py = self._pose.pose.position.x, self._pose.pose.position.y
            self._marker_ex = px + off[0]
            self._marker_ey = py + off[1]

        # Ramp an INDEPENDENT setpoint accumulator toward the marker (same proven
        # pattern as the survey _crawl_to). Anchoring the setpoint to the live
        # pose instead would only ever lead the drone by one step → no traction.
        if self._sp is None:
            self._sp = [self._pose.pose.position.x, self._pose.pose.position.y]
        step = self._survey_speed / self._sp_rate_hz
        sdx, sdy = self._marker_ex - self._sp[0], self._marker_ey - self._sp[1]
        sd = math.hypot(sdx, sdy)
        if sd > 1e-6:
            f = min(1.0, step / sd)
            self._sp[0] += sdx * f
            self._sp[1] += sdy * f
        self._pub_sp(self._sp[0], self._sp[1], self._marker_cur_alt)

        # Convergence is judged on the ACTUAL drone-to-marker distance.
        px, py = self._pose.pose.position.x, self._pose.pose.position.y
        lat = math.hypot(self._marker_ex - px, self._marker_ey - py)
        self._tele(
            f"MARKER_CENTER  lat_err={lat:.2f} m  "
            f"marker_ENU=({self._marker_ex:+.2f},{self._marker_ey:+.2f})  "
            f"(need ≤{self._marker_center_thr:.2f} for {self._marker_center_hold_s:.1f}s)"
            + ("" if off is not None else "  (marker lost — holding last fix)"))

        # Require a FRESH detection to confirm centred — never lock "centred" on
        # a stale fix (which previously let a diverged run-away land off-pad).
        if off is not None and lat <= self._marker_center_thr:
            if self._marker_center_since is None:
                self._marker_center_since = self.get_clock().now()
            elif self._secs(self._marker_center_since) >= self._marker_center_hold_s:
                self.get_logger().info(
                    f"Centred over marker (lat_err={lat:.2f} m) — descending to land.")
                self._marker_desc_z = self._pose.pose.position.z
                self._phase         = Phase.MARKER_LAND
        else:
            self._marker_center_since = None
            if self._secs(self._marker_center_start) > self._marker_center_timeout_s:
                self.get_logger().warn(
                    f"Centre timeout ({self._marker_center_timeout_s:.0f}s, "
                    f"lat_err={lat:.2f} m) — descending anyway.")
                self._marker_desc_z = self._pose.pose.position.z
                self._phase         = Phase.MARKER_LAND

    def _do_marker_land(self):
        """Descend on the (already-centred) marker; hand off to AUTO.LAND near ground.

        Keeps re-centring live during the sink. If the marker is lost for longer
        than marker_timeout_s the last locked ENU position is held, so a brief
        dropout doesn't abort the landing.
        """
        # Refresh the locked marker position from live detections.
        off = self._marker_enu_offset()
        if off is not None:
            px, py = self._pose.pose.position.x, self._pose.pose.position.y
            self._marker_ex = px + off[0]
            self._marker_ey = py + off[1]

        # Sink
        self._marker_desc_z = max(
            0.0, self._marker_desc_z - self._marker_descend_speed / self._sp_rate_hz)

        # Ramp an INDEPENDENT lateral setpoint toward the marker while sinking
        # (same accumulator pattern as the survey crawl, so the drone actually
        # tracks the marker instead of creeping one step behind it).
        if self._sp is None:
            self._sp = [self._pose.pose.position.x, self._pose.pose.position.y]
        step = self._survey_speed / self._sp_rate_hz
        sdx, sdy = self._marker_ex - self._sp[0], self._marker_ey - self._sp[1]
        sd = math.hypot(sdx, sdy)
        if sd > 1e-6:
            f = min(1.0, step / sd)
            self._sp[0] += sdx * f
            self._sp[1] += sdy * f
        self._pub_sp(self._sp[0], self._sp[1], self._marker_desc_z)

        px, py = self._pose.pose.position.x, self._pose.pose.position.y
        lat = math.hypot(self._marker_ex - px, self._marker_ey - py)
        alt = self._pose.pose.position.z
        self._tele(
            f"MARKER_LAND  alt={alt:.2f} m  lat_err={lat:.2f} m"
            + ("" if off is not None else "  (marker lost — holding last fix)"))

        if alt <= self._descend_handoff_alt:
            self.get_logger().info(
                f"Over marker at {alt:.2f} m (lat_err={lat:.2f} m) — AUTO.LAND.")
            self._phase = Phase.LAND

    def _do_land(self):
        if not self._gate_close_sent:
            if self._gate_cli.service_is_ready():
                req = SetBool.Request()
                req.data = False
                self._gate_cli.call_async(req)
            self._gate_close_sent = True
        if not self._land_req:
            self._mode("AUTO.LAND")
            self._land_req = True
            self._phase    = Phase.DISARM

    def _do_disarm(self):
        if not self._state.armed:
            captured = self._wpt_idx
            total    = len(self._waypoints)
            self.get_logger().info(
                f"Disarmed — mission complete ✓  "
                f"({captured}/{total} checkpoints captured)\n"
                f"Data: {self._survey_run_dir}")
            self._phase = Phase.DONE

    def _do_flow_hold(self):
        """Stable zero-velocity hold on flow after a VIO fault.
        After stable_of_secs, attempt re-validation (up to max_revalidations).
        Survey wpt_idx is banked — survey resumes from where it left off."""
        self._pub_vel_hold()
        if self._flow_hold_start is None:
            self._flow_hold_start = self.get_clock().now()
        if self._secs(self._flow_hold_start) < self._stable_of_secs:
            return
        # ready to retry
        self._flow_hold_start = None
        self._revalidations  += 1
        if self._revalidations > int(self._max_revalidations):
            self.get_logger().error(
                f"Max revalidations ({int(self._max_revalidations)}) exceeded — AUTO.LAND")
            self._phase = Phase.LAND
            return
        self.get_logger().warn(
            f"Re-validation attempt {self._revalidations}/{int(self._max_revalidations)} "
            f"— survey will resume from cp{self._wpt_idx}")
        self._seed_sent = False
        self._phase     = Phase.SEED

    def _do_safe_manual(self):
        self.get_logger().info(
            "SAFE MANUAL — pilot has control.", throttle_duration_sec=5.0)

    # ─────────────────────────────────────────────────────────────
    # Motion / setpoint helpers
    # ─────────────────────────────────────────────────────────────

    def _crawl_to(self, tx: float, ty: float) -> float:
        """Move internal setpoint toward (tx, ty) at survey_speed_ms.
        Returns Euclidean distance from CURRENT DRONE POSITION to target."""
        if self._sp is None:
            self._sp = [self._pose.pose.position.x,
                        self._pose.pose.position.y]
        step = self._survey_speed / self._sp_rate_hz
        dx, dy = tx - self._sp[0], ty - self._sp[1]
        d = math.hypot(dx, dy)
        if d > 1e-6:
            f = min(1.0, step / d)
            self._sp[0] += dx * f
            self._sp[1] += dy * f
        self._pub_sp(self._sp[0], self._sp[1], self._target_alt)
        return math.hypot(
            self._pose.pose.position.x - tx,
            self._pose.pose.position.y - ty)

    def _pub_sp(self, x: float, y: float, z: float):
        m = PoseStamped()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = "local_origin"
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)

        if self._hold_heading_q is not None and self._cmd_yaw_rad is not None:
            # Slew _cmd_yaw_rad toward arm-time target yaw at yaw_slew_dps.
            # This prevents PX4 from snapping aggressively if the drone drifted.
            target_yaw = math.radians(
                yaw_deg_from_quaternion(self._hold_heading_q))
            max_step = math.radians(self._yaw_slew_dps) / self._sp_rate_hz
            diff = (target_yaw - self._cmd_yaw_rad + math.pi) % (2 * math.pi) - math.pi
            self._cmd_yaw_rad += max(min(diff, max_step), -max_step)
            yaw = self._cmd_yaw_rad
            # Pure-yaw quaternion (roll=pitch=0)
            m.pose.orientation.w = math.cos(yaw / 2.0)
            m.pose.orientation.x = 0.0
            m.pose.orientation.y = 0.0
            m.pose.orientation.z = math.sin(yaw / 2.0)
        else:
            m.pose.orientation = self._pose.pose.orientation

        self._sp_pub.publish(m)

    def _pub_vel_hold(self):
        """Publish zero velocity — lets flow hold position without fighting EKF drift."""
        m = TwistStamped()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = "local_origin"
        self._vel_pub.publish(m)

    def _mode(self, mode: str):
        if self._mode_cli.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = mode
            self._mode_cli.call_async(req)

    def _arm(self, value: bool):
        if self._arm_cli.service_is_ready():
            req = CommandBool.Request()
            req.value = value
            self._arm_cli.call_async(req)

    def _secs(self, t) -> float:
        if t is None:
            return 0.0
        return (self.get_clock().now() - t).nanoseconds * 1e-9

    def _ch5(self) -> int:
        return int(self._rc[self._rc_ch5_idx]) if len(self._rc) > self._rc_ch5_idx else 1500

    def _tele(self, line: str):
        # Terminal stays quiet — the current command + RTAB quality are shown in
        # the yellow_boundary_detector's status table instead. We just publish a
        # compact "<command>|<rtabQ>" string here every tick; important state
        # CHANGES are still logged separately via get_logger().info elsewhere.
        cov = self._rtab_cov
        if cov <= 0.0:
            q = "SILENT"
        elif cov >= 100.0:
            q = "LOST"
        elif cov >= 10.0:
            q = "WEAK"
        else:
            q = "GOOD"
        try:
            self._status_pub.publish(String(data=f"{line}|{q}"))
        except Exception:
            pass
        now = time.monotonic()
        if now - self._last_print >= 1.0:
            self._last_print = now
            self.get_logger().debug(
                f"{line}  | RTAB cov={cov:.3f} [{q}]  IF={self._init_factor:.2f}")


# ─────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = SurveyMission()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
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
