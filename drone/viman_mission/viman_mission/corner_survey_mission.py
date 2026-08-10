#!/usr/bin/env python3
"""
corner_survey_mission.py  ·  Viman Rakshak  ·  IRoC-U 2026
============================================================
Yellow-tape L-corner guided survey.

Phase sequence:
  IDLE → ARM → TAKEOFF → STABLE_OF → HOVER_HOME → SEED → VALIDATE → HANDOVER
  → FIND_CORNER → ALIGN_C1 → SURVEY → RETURN
  → FLOW_SETTLE → DESCEND → LAND → DISARM → DONE

FIND_CORNER
  Flies body -X (backward) at find_corner_speed_ms searching for the L-corner.
  If not found within find_corner_max_m, swings body +Y (left) for one more leg.
  Aborts to LAND if both legs exhaust without finding the corner.

ALIGN_C1
  Visual servo in body XY: drives corner_px toward image centre (±align_px_tol).
  Holds for align_hold_s once centred.  Records corner ENU position then
  transitions to SURVEY.

SURVEY
  Standard boustrophedon from the corner ENU origin (same grid as survey_mission).
  Before each waypoint step, checks dist_to_line in the travel direction and skips
  any waypoint where tape is within tape_stop_dist_m (boundary reached).
  Captures JPEG + CSV at each checkpoint.

Body-frame sign conventions (MAVROS ENU / FLU):
  body +X = forward   body +Y = LEFT   body +Z = up
  Image pixel origin top-left; +u = right, +v = down.
  Camera mounted nadir with "top" facing drone nose:
    image up    = drone forward
    image right = drone right = body -Y

Run:
  ros2 launch viman_mission bringup.launch.py mission_node:=corner_survey_mission
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

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import RCIn, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, UInt8
from std_srvs.srv import SetBool, Trigger

from viman_mission.common import (qos_best_effort, qos_reliable,
                                  yaw_deg_from_quaternion)
from viman_mission.yellow_detector import YellowDetector

# vio_gate states
GS_UNSEEDED   = 0
GS_SEEDING    = 1
GS_VALIDATING = 2
GS_OPEN       = 3
GS_FAULT_MIN  = 4   # 4,5,6 are all fault states


class Phase(Enum):
    IDLE         = auto()
    ARM          = auto()
    TAKEOFF      = auto()
    STABLE_OF    = auto()
    HOVER_HOME   = auto()
    SEED         = auto()
    VALIDATE     = auto()
    HANDOVER     = auto()
    FIND_CORNER  = auto()   # fly body +X (forward) until forward tape wall detected
    ALIGN_YAW    = auto()   # collect tape-angle samples; rotate to face the wall
    SURVEY       = auto()   # boustrophedon grid
    RETURN       = auto()
    FLOW_SETTLE  = auto()
    FLOW_HOLD    = auto()
    DESCEND      = auto()
    LAND         = auto()
    DISARM       = auto()
    SAFE_MANUAL  = auto()
    DONE         = auto()


class CornerSurveyMission(Node):

    # D455 @ 1280×720 intrinsics
    _CAM_CX: int   = 640
    _CAM_CY: int   = 360
    _CAM_FX: float = 906.0
    _CAM_FY: float = 906.0

    def __init__(self):
        super().__init__("corner_survey_mission")

        # ── Parameter declarations ────────────────────────────────
        p = self.declare_parameters("", [
            # flight profile (mirrors survey_mission)
            ("target_alt",             3.0),
            ("alt_tolerance",          0.12),
            ("at_alt_confirm_s",       1.5),
            ("stable_of_secs",         4.0),
            ("seed_timeout_s",         10.0),
            ("validate_if_min",        0.51),
            ("validate_hold_s",        5.0),
            ("validate_dip_grace_s",   1.0),
            ("validate_timeout_s",     60.0),
            ("motion_test",            True),
            ("motion_amp_m",           0.2),
            ("motion_leg_s",           4.0),
            ("handover_settle_s",      2.0),
            ("goto_radius_m",          0.20),
            ("goto_timeout_s",         30.0),
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
            ("yaw_slew_dps",           15.0),
            # survey grid
            ("survey_width_m",         6.0),
            ("survey_height_m",        6.0),
            ("stripe_spacing_m",       1.5),
            ("col_spacing_m",          1.5),
            ("waypoint_radius_m",      0.25),
            ("waypoint_settle_s",      1.0),
            ("survey_speed_ms",        0.2),
            ("survey_dir",             "/media/jetson/ROS2_SSD/survey"),
            # corner finding
            ("find_corner_speed_ms",   0.15),  # forward crawl speed to the wall
            ("find_corner_max_m",      5.0),   # max forward travel before abort
            # tape boundary enforcement
            ("tape_stop_dist_m",       0.45),  # skip wpt if tape this close ahead
            # yaw alignment
            ("yaw_align_tol_deg",      3.0),   # error below this = aligned
            ("yaw_align_hold_s",       2.0),   # hold within tol this long before locking
            ("yaw_align_timeout_s",    25.0),  # give up and use current yaw if exceeded
            ("yaw_align_dir",          1.0),   # +1.0 or -1.0 — flip if drone rotates AWAY from tape
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
         self._yaw_slew_dps,
         self._survey_w, self._survey_h, self._stripe_spacing,
         self._col_spacing, self._wpt_radius, self._wpt_settle_s,
         self._survey_speed, self._base_survey_dir,
         self._find_speed, self._find_max_m,
         self._tape_stop_dist,
         self._yaw_align_tol, self._yaw_align_hold_s,
         self._yaw_align_timeout,
         self._yaw_align_dir) = (x.value for x in p)

        # ── Detector ──────────────────────────────────────────────
        self._detector = YellowDetector(debug=False)

        # ── Mission state ─────────────────────────────────────────
        self._phase      = Phase.IDLE
        self._last_phase = None

        # Arm-time ENU position (returned to for landing)
        self._arm_x = 0.0
        self._arm_y = 0.0
        # Survey origin (overwritten with corner ENU after ALIGN_C1)
        self._home_x = 0.0
        self._home_y = 0.0

        self._hold_heading_q  = None   # arm-time orientation quaternion
        self._cmd_yaw_rad     = None   # slewed yaw setpoint [rad]
        self._hover_home_since = None

        self._validate_anchor = (0.0, 0.0)

        # ── Live sensor data ──────────────────────────────────────
        self._pose         = PoseStamped()
        self._pose.pose.orientation.w = 1.0
        self._state        = State()
        self._rc           = ()
        self._vio_state    = 255
        self._init_factor  = 0.0
        self._pose_stamps  = []
        self._last_rtab_ns = 0
        self._ch5_latched  = False

        # Camera — stored as BGR for direct use in detector
        self._bridge      = CvBridge()
        self._latest_bgr  = None
        self._frame_lock  = threading.Lock()

        # ── Survey data ───────────────────────────────────────────
        self._waypoints          = []
        self._wpt_idx            = 0
        self._capture_lock       = threading.Lock()
        self._last_captured_xy   = None
        self._survey_run_dir     = ""
        self._log_path           = ""
        self._survey_initialized = False
        self._corner_x           = 0.0
        self._corner_y           = 0.0

        # ── Navigation helpers ────────────────────────────────────
        self._sp            = None
        self._arrived_since = None
        self._settle_start  = None
        self._goto_start    = None
        self._desc_z        = 0.0
        self._desc_start    = None

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

        # FIND_CORNER (single forward approach to the wall) sub-state
        self._fc_origin_x    = 0.0   # ENU position when FIND_CORNER first entered (HANDOVER)
        self._fc_origin_y    = 0.0
        self._fc_braking     = False  # True while publishing vel=0 after wall first spotted
        self._fc_brake_count = 0      # frames elapsed in braking period
        self._fc_vio_fault_count = 0  # consecutive VIO-fault ticks in FIND_CORNER

        # ALIGN_YAW state
        self._align_yaw_start          = None   # phase entry time (for timeout)
        self._yaw_samples: list        = []     # tape-angle samples (deg)
        self._align_phase              = 'measure'  # 'measure' | 'rotate' | 'verify'
        self._align_target_yaw_rad     = None   # computed target heading after measure
        self._align_rotate_start       = None   # time rotation/verify phase began

        signal.signal(signal.SIGINT, self._sigint)

        # ── ROS wiring ────────────────────────────────────────────
        cb      = ReentrantCallbackGroup()
        qos_be  = qos_best_effort()
        qos_rel = qos_reliable()

        self.create_subscription(State,       "/mavros/state",
                                 self._state_cb,      qos_rel, callback_group=cb)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._pose_cb,       qos_be,  callback_group=cb)
        self.create_subscription(RCIn,        "/mavros/rc/in",
                                 self._rc_cb,         qos_be,  callback_group=cb)
        self.create_subscription(Odometry,    rtab_topic,
                                 self._rtab_alive_cb, qos_rel, callback_group=cb)
        self.create_subscription(UInt8,       "/viman/vio_state",
                                 self._vio_state_cb,  10, callback_group=cb)
        self.create_subscription(Float32,     "/viman/init_factor",
                                 self._if_cb,         10, callback_group=cb)
        self.create_subscription(Image, "/camera/camera/color/image_raw",
                                 self._image_cb,      qos_be,  callback_group=cb)

        self._sp_pub  = self.create_publisher(
            PoseStamped,  "/mavros/setpoint_position/local", 10)
        self._vel_pub = self.create_publisher(
            TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10)

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
            Phase.FIND_CORNER: self._do_find_wall,
            Phase.ALIGN_YAW:   self._do_align_yaw,
            Phase.SURVEY:      self._do_survey,
            Phase.RETURN:      self._do_return,
            Phase.FLOW_SETTLE: self._do_flow_settle,
            Phase.FLOW_HOLD:   self._do_flow_hold,
            Phase.DESCEND:     self._do_descend,
            Phase.LAND:        self._do_land,
            Phase.DISARM:      self._do_disarm,
            Phase.SAFE_MANUAL: self._do_safe_manual,
            Phase.DONE:        lambda: None,
        }

        self.create_timer(1.0 / self._sp_rate_hz, self._loop, callback_group=cb)

        self.get_logger().info(
            "CornerSurveyMission ready — "
            f"{self._survey_w:.0f}×{self._survey_h:.0f} m @ {self._target_alt:.1f} m | "
            "SAFETY LATCH: flip CH5 HIGH once, then LOW to start.")

    # ─────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────

    def _state_cb(self, m):       self._state = m
    def _vio_state_cb(self, m):   self._vio_state = m.data
    def _if_cb(self, m):          self._init_factor = m.data
    def _rtab_alive_cb(self, _):  self._last_rtab_ns = self.get_clock().now().nanoseconds

    def _pose_cb(self, m: PoseStamped):
        self._pose = m
        t = self.get_clock().now().nanoseconds
        self._pose_stamps.append(t)
        while self._pose_stamps and self._pose_stamps[0] < t - 2_000_000_000:
            self._pose_stamps.pop(0)

    def _image_cb(self, msg: Image):
        """Store latest frame as BGR numpy (ready for YellowDetector)."""
        try:
            rgb = self._bridge.imgmsg_to_cv2(msg, "rgb8")
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            with self._frame_lock:
                self._latest_bgr = bgr
        except Exception as e:
            self.get_logger().warn(f"Image convert: {e}", throttle_duration_sec=5.0)

    def _rc_cb(self, m: RCIn):
        self._rc = m.channels
        if self._ch5() >= 1300:
            self._ch5_latched = True
        if self._phase in (Phase.SAFE_MANUAL, Phase.DONE, Phase.IDLE):
            return
        if self._ch5() >= self._rc_interrupt_high:
            self.get_logger().warn(f"⚠ RC INTERRUPT → STABILIZED")
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
            self._sp = None

        if self._ctrl_c and self._phase not in (
                Phase.LAND, Phase.DISARM, Phase.DONE, Phase.SAFE_MANUAL):
            self._phase = Phase.LAND
            return

        self._handlers[self._phase]()

    # ─────────────────────────────────────────────────────────────
    # Pre-survey phases  (identical to survey_mission)
    # ─────────────────────────────────────────────────────────────

    def _preflight_failures(self):
        fails = []
        if not self._state.connected:
            fails.append("FCU")
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

    def _do_idle(self):
        self._pub_sp(0.0, 0.0, 0.3)
        fails = self._preflight_failures()
        if fails:
            self.get_logger().warn("PREFLIGHT BLOCKED: " + ", ".join(fails),
                                   throttle_duration_sec=5.0)
            return
        if not self._ch5_latched:
            self.get_logger().info(
                "Preflight OK — flip CH5 HIGH once then LOW to arm",
                throttle_duration_sec=5.0)
            return
        if self._ch5() <= self._rc_start_low:
            self._offboard_req = self._arm_req = False
            self._phase = Phase.ARM

    def _do_arm(self):
        self._pub_sp(0.0, 0.0, 0.3)
        if not self._offboard_req:
            self._mode("OFFBOARD"); self._offboard_req = True; return
        if self._state.mode != "OFFBOARD":
            self._mode("OFFBOARD"); return
        if not self._arm_req:
            self._arm(True); self._arm_req = True; return
        if self._state.armed:
            self._arm_x = self._pose.pose.position.x
            self._arm_y = self._pose.pose.position.y
            self._home_x = self._arm_x
            self._home_y = self._arm_y
            self.get_logger().info(
                f"Armed. ARM = ({self._arm_x:.3f}, {self._arm_y:.3f})")
            self._phase = Phase.TAKEOFF

    def _do_takeoff(self):
        self._pub_sp(self._arm_x, self._arm_y, self._target_alt)
        alt = self._pose.pose.position.z
        self._tele(f"TAKEOFF  {alt:.2f}/{self._target_alt:.1f} m")
        if abs(alt - self._target_alt) <= self._alt_tol:
            if self._at_alt_since is None:
                self._at_alt_since = self.get_clock().now()
            elif self._secs(self._at_alt_since) >= self._at_alt_confirm_s:
                self._stable_since = self.get_clock().now()
                self._phase = Phase.STABLE_OF
        else:
            self._at_alt_since = None

    def _do_stable_of(self):
        self._pub_vel_hold()
        if self._secs(self._stable_since) >= self._stable_of_secs:
            self._hover_home_since = None
            self._phase = Phase.HOVER_HOME

    def _do_hover_home(self):
        self._pub_sp(self._arm_x, self._arm_y, self._target_alt)
        dist = math.hypot(self._pose.pose.position.x - self._arm_x,
                          self._pose.pose.position.y - self._arm_y)
        self._tele(f"HOVER_HOME  dist={dist:.2f} m")
        if dist <= self._goto_radius:
            if self._hover_home_since is None:
                self._hover_home_since = self.get_clock().now()
            elif self._secs(self._hover_home_since) >= 2.0:
                self._seed_sent = False
                self._phase = Phase.SEED
        else:
            self._hover_home_since = None

    def _do_seed(self):
        self._pub_vel_hold()
        if not self._seed_sent:
            if not self._seed_cli.service_is_ready():
                return
            self._seed_cli.call_async(Trigger.Request())
            self._seed_sent  = True
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
            self.get_logger().warn("Seed timeout — flow hold, retry")
            self._flow_hold_start = None
            self._phase = Phase.FLOW_HOLD

    def _motion_offset(self):
        if not self._motion_test:
            return 0.0, 0.0
        corners = ((0.0, 0.0),
                   (self._motion_amp, 0.0),
                   (self._motion_amp, self._motion_amp),
                   (0.0, self._motion_amp))
        leg = int(self._secs(self._validate_start) / self._motion_leg_s)
        return corners[leg % 4]

    def _do_validate(self):
        ax, ay = self._validate_anchor
        ox, oy = self._motion_offset()
        self._pub_sp(ax + ox, ay + oy, self._target_alt)
        if self._vio_state >= GS_FAULT_MIN:
            self.get_logger().warn("VIO fault during VALIDATE — flow hold")
            self._flow_hold_start = None
            self._phase = Phase.FLOW_HOLD
            return
        el = self._secs(self._validate_start)
        if self._init_factor >= self._if_min:
            self._if_low_since = None
            if self._if_good_since is None:
                self._if_good_since = self.get_clock().now()
            if self._secs(self._if_good_since) >= self._validate_hold_s:
                self._handover_sent  = False
                self._handover_start = None
                self._phase = Phase.HANDOVER
                return
        elif self._if_good_since is not None:
            if self._if_low_since is None:
                self._if_low_since = self.get_clock().now()
            elif self._secs(self._if_low_since) > self._dip_grace_s:
                self._if_good_since = self._if_low_since = None
        self._tele(f"VALIDATE  IF={self._init_factor:.2f}≥{self._if_min:.2f}  t={el:.0f}s")
        if el > self._validate_timeout_s:
            self.get_logger().error("Validation timeout — landing")
            self._phase = Phase.LAND

    def _do_handover(self):
        ax, ay = self._validate_anchor
        self._pub_sp(ax, ay, self._target_alt)
        if not self._handover_sent:
            if not self._gate_cli.service_is_ready():
                return
            req = SetBool.Request(); req.data = True
            self._gate_cli.call_async(req)
            self._handover_sent  = True
            self._handover_start = self.get_clock().now()
            return
        if self._vio_state == GS_OPEN:
            if self._secs(self._handover_start) >= self._handover_settle_s:
                if self._survey_initialized:
                    # Fault recovery: skip FIND_CORNER, resume survey directly
                    self.get_logger().info(
                        f"Gate OPEN (recovery) — resuming survey at cp{self._wpt_idx}")
                    self._sp = self._arrived_since = self._settle_start = None
                    self._phase = Phase.SURVEY
                else:
                    self.get_logger().info("Gate OPEN → FIND_WALL")
                    self._fc_braking        = False
                    self._fc_brake_count    = 0
                    self._fc_vio_fault_count = 0
                    self._fc_origin_x = self._pose.pose.position.x
                    self._fc_origin_y = self._pose.pose.position.y
                    self._phase = Phase.FIND_CORNER
        elif self._secs(self._handover_start) > 5.0:
            self.get_logger().warn("Gate did not open — flow hold")
            self._flow_hold_start = None
            self._phase = Phase.FLOW_HOLD

    # ─────────────────────────────────────────────────────────────
    # FIND_CORNER
    # ─────────────────────────────────────────────────────────────

    def _do_find_wall(self):
        """
        Fly body +X (forward) at find_corner_speed_ms until the forward tape
        boundary is detected within tape_stop_dist_m.

        When tape is seen close enough:
          1. Brake for ~0.5 s so the drone decelerates.
          2. Lock corner ENU to current position.
          3. Transition to ALIGN_YAW, which collects tape-angle samples while
             stationary and rotates the survey grid accordingly.

        This replaces the two-leg corner search + pixel-centring servo with a
        single straight approach to the wall — faster, simpler, and more
        consistent because the tape line is always in front.

        VIO fault: hold vel=0 and wait (do NOT land immediately).
        Max distance guard: if find_corner_max_m is reached without seeing the
        tape, AUTO.LAND (operator placed the drone too far from the survey area).
        """
        # ── VIO fault: hold, don't move ───────────────────────────
        if self._vio_state >= GS_FAULT_MIN:
            self._fc_vio_fault_count += 1
            self.get_logger().warn(
                f"VIO fault in FIND_WALL ({self._fc_vio_fault_count}) — holding",
                throttle_duration_sec=2.0)
            self._pub_vel_hold()
            if self._fc_vio_fault_count > 60:   # ~3 s sustained → LAND
                self.get_logger().error("Sustained VIO fault in FIND_WALL — LAND")
                self._phase = Phase.LAND
            return
        self._fc_vio_fault_count = 0

        # ── Braking period after wall first seen ─────────────────
        if self._fc_braking:
            self._fc_brake_count += 1
            self._pub_vel_hold()
            if self._fc_brake_count >= 10:   # 0.5 s at 20 Hz — enough to stop
                self._fc_braking = False
                # Lock the survey origin here at the wall
                self._corner_x = self._pose.pose.position.x
                self._corner_y = self._pose.pose.position.y
                self.get_logger().info(
                    f"Wall reached — origin locked ENU "
                    f"({self._corner_x:.3f}, {self._corner_y:.3f}) → ALIGN_YAW")
                self._align_yaw_start      = None
                self._yaw_samples          = []
                self._align_phase          = 'measure'
                self._align_target_yaw_rad = None
                self._align_rotate_start   = None
                self._phase = Phase.ALIGN_YAW
            return

        # ── Max distance safety guard ─────────────────────────────
        fwd_dist = math.hypot(
            self._pose.pose.position.x - self._fc_origin_x,
            self._pose.pose.position.y - self._fc_origin_y)
        if fwd_dist >= self._find_max_m:
            self.get_logger().error(
                f"FIND_WALL: {fwd_dist:.1f}m forward with no tape — LAND")
            self._phase = Phase.LAND
            return

        # ── Tape check ────────────────────────────────────────────
        frame = self._get_bgr_frame()
        if frame is not None:
            alt = max(self._pose.pose.position.z, 1.0)
            d = self._detector.dist_to_line(
                frame, 'fwd',
                altitude_m=alt, fx=self._CAM_FX, fy=self._CAM_FY)
            if d is not None and d < self._tape_stop_dist:
                self.get_logger().info(
                    f"Wall tape detected at {d:.2f}m — braking")
                self._fc_braking     = True
                self._fc_brake_count = 0
                self._pub_vel_hold()
                return

        # ── Fly forward ───────────────────────────────────────────
        self._tele(f"FIND_WALL  fwd={fwd_dist:.2f}/{self._find_max_m:.1f}m  "
                   f"speed={self._find_speed:.2f} m/s")
        self._pub_body_vel(self._find_speed, 0.0)

    # ─────────────────────────────────────────────────────────────
    # ALIGN_YAW
    # ─────────────────────────────────────────────────────────────

    def _do_align_yaw(self):
        """
        Physically align drone heading to the back wall tape — 3 phases:

          MEASURE  Hold position, collect yaw_error_deg() for yaw_align_hold_s.
                   Compute median of all samples.  If |median| < tol → already
                   aligned, skip rotation.

          ROTATE   Command position + target yaw via _pub_sp_with_yaw so PX4
                   slews smoothly to the corrected heading.  Wait until actual
                   heading is within yaw_align_tol_deg of target (max 15 s).
                   Slow controlled rotation → VIO stays healthy.

          VERIFY   Collect a handful of fresh tape-angle samples at the new
                   heading.  Log residual.  Transition to SURVEY regardless
                   (we've done our best; survey grid uses current heading).

        After physical rotation _launch_survey() uses yaw_correction_deg=0
        because the drone itself is now aligned — no further grid math needed.
        """
        if self._align_yaw_start is None:
            self._align_yaw_start = self.get_clock().now()

        elapsed = self._secs(self._align_yaw_start)

        # ── Global timeout ────────────────────────────────────────────────────
        if elapsed > self._yaw_align_timeout:
            self.get_logger().warn(
                "ALIGN_YAW timeout — launching survey with current heading")
            self._launch_survey()
            return

        # ═══════════════════════════════════════════════════════════════════
        # PHASE 1 — MEASURE: hold stationary, collect tape-angle samples
        # ═══════════════════════════════════════════════════════════════════
        if self._align_phase == 'measure':
            self._pub_sp(self._corner_x, self._corner_y, self._target_alt)

            frame = self._get_bgr_frame()
            if frame is not None:
                err = self._detector.yaw_error_deg(frame)
                if err is not None:
                    self._yaw_samples.append(err)

            n    = len(self._yaw_samples)
            last = f"{self._yaw_samples[-1]:+.1f}°" if n else "none"
            self._tele(
                f"ALIGN_YAW [measure]  samples={n}  last={last}  "
                f"t={elapsed:.1f}/{self._yaw_align_hold_s:.1f}s")

            if elapsed >= self._yaw_align_hold_s and n >= 5:
                med = float(sorted(self._yaw_samples)[n // 2])
                self.get_logger().info(
                    f"ALIGN_YAW measure: {n} samples  median={med:+.1f}°")

                if abs(med) < self._yaw_align_tol:
                    self.get_logger().info(
                        "ALIGN_YAW: tape already aligned — no rotation needed → SURVEY")
                    self._launch_survey()
                    return

                # Compute target heading and enter ROTATE phase.
                # yaw_align_dir is the field-settable SIGN of the correction.
                # The detector's own usage note (yaw_rate = -K·err) implies the
                # correction may need to be negative; if the drone rotates AWAY
                # from the tape on the first flight, set yaw_align_dir:=-1.0 in
                # mission_params.yaml — no code change needed.
                current_hdg = yaw_deg_from_quaternion(self._pose.pose.orientation)
                correction  = self._yaw_align_dir * med
                self._align_target_yaw_rad = math.radians(current_hdg + correction)
                self._align_phase          = 'rotate'
                self._align_rotate_start   = self.get_clock().now()
                self.get_logger().info(
                    f"ALIGN_YAW → ROTATE: current={current_hdg:.1f}°  "
                    f"tape_err={med:+.1f}°  correction={correction:+.1f}°  "
                    f"target={math.degrees(self._align_target_yaw_rad):.1f}°")
            return

        # ═══════════════════════════════════════════════════════════════════
        # PHASE 2 — ROTATE: command target yaw, wait for convergence
        # ═══════════════════════════════════════════════════════════════════
        if self._align_phase == 'rotate':
            self._pub_sp_with_yaw(
                self._corner_x, self._corner_y,
                self._target_alt, self._align_target_yaw_rad)

            rotate_elapsed = self._secs(self._align_rotate_start)
            current_hdg    = yaw_deg_from_quaternion(self._pose.pose.orientation)
            target_hdg     = math.degrees(self._align_target_yaw_rad)
            hdg_err        = ((target_hdg - current_hdg + 180.0) % 360.0) - 180.0

            self._tele(
                f"ALIGN_YAW [rotate]  hdg={current_hdg:.1f}°  "
                f"tgt={target_hdg:.1f}°  err={hdg_err:+.1f}°  "
                f"t={rotate_elapsed:.1f}s")

            if abs(hdg_err) < self._yaw_align_tol:
                self.get_logger().info(
                    f"ALIGN_YAW: heading converged  err={hdg_err:+.1f}° → VERIFY")
                self._align_phase        = 'verify'
                self._yaw_samples        = []
                self._align_rotate_start = self.get_clock().now()
                return

            if rotate_elapsed > 15.0:
                self.get_logger().warn(
                    "ALIGN_YAW rotate timeout — launching with best heading")
                self._launch_survey()
            return

        # ═══════════════════════════════════════════════════════════════════
        # PHASE 3 — VERIFY: confirm tape is now horizontal, then SURVEY
        # ═══════════════════════════════════════════════════════════════════
        if self._align_phase == 'verify':
            self._pub_sp_with_yaw(
                self._corner_x, self._corner_y,
                self._target_alt, self._align_target_yaw_rad)

            frame = self._get_bgr_frame()
            if frame is not None:
                err = self._detector.yaw_error_deg(frame)
                if err is not None:
                    self._yaw_samples.append(err)

            n              = len(self._yaw_samples)
            verify_elapsed = self._secs(self._align_rotate_start)
            last           = f"{self._yaw_samples[-1]:+.1f}°" if n else "none"
            self._tele(
                f"ALIGN_YAW [verify]  samples={n}  last={last}  "
                f"t={verify_elapsed:.1f}s")

            if n >= 5 or verify_elapsed > 3.0:
                residual = (float(sorted(self._yaw_samples)[n // 2])
                            if n > 0 else 0.0)
                if abs(residual) < self._yaw_align_tol * 2:
                    self.get_logger().info(
                        f"ALIGN_YAW: verified  residual={residual:+.1f}° → SURVEY")
                else:
                    self.get_logger().warn(
                        f"ALIGN_YAW: residual {residual:+.1f}° exceeds 2×tol — "
                        f"launching anyway")
                # Drone is now physically aligned — no additional grid correction.
                self._launch_survey()
            return

    # ─────────────────────────────────────────────────────────────
    # Survey initialisation
    # ─────────────────────────────────────────────────────────────

    def _launch_survey(self, yaw_correction_deg: float = 0.0):
        """Set survey origin to detected corner; build grid; go to SURVEY.

        yaw_correction_deg: offset measured by ALIGN_YAW (tape angle error).
        Added to the drone's current heading so the grid aligns with the tape
        without physically rotating the drone.
        """
        self._home_x = self._corner_x
        self._home_y = self._corner_y
        # Base heading: drone's actual orientation at survey-launch time.
        # Apply tape-angle correction so grid rows run perpendicular to the tape.
        current_hdg_deg = yaw_deg_from_quaternion(self._pose.pose.orientation)
        self._cmd_yaw_rad = math.radians(current_hdg_deg + yaw_correction_deg)
        self._waypoints = self._build_waypoints()
        self._wpt_idx   = 0
        self._init_survey_storage()
        self._survey_initialized = True
        self._sp = self._arrived_since = self._settle_start = None
        self._phase = Phase.SURVEY
        self.get_logger().info(
            f"Survey launched from corner ({self._corner_x:.2f}, {self._corner_y:.2f}) "
            f"— {len(self._waypoints)} checkpoints.")

    # ─────────────────────────────────────────────────────────────
    # Lawnmower geometry  (identical to survey_mission)
    # ─────────────────────────────────────────────────────────────

    def _build_waypoints(self):
        yaw   = self._cmd_yaw_rad
        fwd_x =  math.cos(yaw);  fwd_y =  math.sin(yaw)
        rgt_x =  math.sin(yaw);  rgt_y = -math.cos(yaw)  # 90° CW from fwd
        n_rows = self._n_rows
        n_cols = self._n_cols
        wps = []
        for c in range(n_cols):
            row_range = list(range(n_rows))
            if c % 2 == 1:
                row_range = row_range[::-1]
            for row in row_range:
                x = self._home_x + row * self._stripe_spacing * fwd_x \
                                 + c   * self._col_spacing    * rgt_x
                y = self._home_y + row * self._stripe_spacing * fwd_y \
                                 + c   * self._col_spacing    * rgt_y
                wps.append((x, y))
        self.get_logger().info(
            f"Grid {n_rows}×{n_cols}={len(wps)} wpts  "
            f"origin ({self._home_x:.2f},{self._home_y:.2f})  "
            f"yaw={math.degrees(yaw):.1f}°")
        return wps

    @property
    def _n_cols(self):
        return max(1, round(self._survey_w / self._col_spacing)) + 1

    @property
    def _n_rows(self):
        return max(1, round(self._survey_h / self._stripe_spacing)) + 1

    def _wpt_to_rowcol(self, idx: int):
        """Flat waypoint index → (row, col).

        Mirrors _build_waypoints' ordering EXACTLY: outer loop = columns
        (stripes), inner loop = rows (n_rows per column), odd columns reversed
        (boustrophedon). The flat index groups by n_rows, NOT n_cols — dividing
        by n_cols was a bug that mislabeled almost every checkpoint on a
        non-square grid (7.5×9 → 6 cols × 7 rows: 41/42 labels wrong).
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
        self._survey_run_dir = os.path.join(
            self._base_survey_dir, f"corner_survey_{ts}")
        os.makedirs(self._survey_run_dir, exist_ok=True)
        self._log_path = os.path.join(self._survey_run_dir, "coordinates.csv")
        with open(self._log_path, 'w', newline='') as f:
            csv.writer(f).writerow([
                "checkpoint", "row", "col", "timestamp_s",
                "x_enu", "y_enu", "z_enu", "yaw_deg", "image_file"])
        self.get_logger().info(f"Survey dir: {self._survey_run_dir}")

    def _capture_checkpoint(self, idx: int) -> bool:
        row, col = self._wpt_to_rowcol(idx)
        img_name = f"cp{idx:04d}_r{row:02d}c{col:02d}.jpg"
        img_path = os.path.join(self._survey_run_dir, img_name)
        saved    = False
        frame    = self._get_bgr_frame()
        if frame is not None:
            try:
                cv2.imwrite(img_path, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                saved = True
            except Exception as e:
                self.get_logger().error(f"imwrite cp{idx}: {e}")
                img_name = "WRITE_FAILED"
        else:
            img_name = "NO_FRAME"
        p   = self._pose.pose.position
        yaw = yaw_deg_from_quaternion(self._pose.pose.orientation)
        ts  = self.get_clock().now().nanoseconds / 1e9
        with open(self._log_path, 'a', newline='') as f:
            csv.writer(f).writerow([
                idx, row, col, f"{ts:.3f}",
                f"{p.x:.4f}", f"{p.y:.4f}", f"{p.z:.4f}",
                f"{yaw:.1f}", img_name])
        self.get_logger().info(
            f"[cp{idx:04d}] r{row}c{col}  "
            f"({p.x:.2f},{p.y:.2f},{p.z:.2f})  {'✓' if saved else '✗'}")
        return saved

    # ─────────────────────────────────────────────────────────────
    # SURVEY
    # ─────────────────────────────────────────────────────────────

    def _do_survey(self):
        if self._vio_state >= GS_FAULT_MIN:
            self.get_logger().error(
                f"VIO fault at cp{self._wpt_idx} — flow hold (will resume)")
            self._flow_hold_start = None
            self._phase = Phase.FLOW_HOLD
            return

        if self._wpt_idx >= len(self._waypoints):
            self.get_logger().info(
                f"Survey COMPLETE — {len(self._waypoints)} checkpoints. Returning.")
            self._arrived_since = self._goto_start = None
            self._phase = Phase.RETURN
            return

        tx, ty = self._waypoints[self._wpt_idx]

        # ── Navigate ──────────────────────────────────────────────
        if self._settle_start is None:
            dist = self._crawl_to(tx, ty)
            row, col = self._wpt_to_rowcol(self._wpt_idx)
            self._tele(
                f"SURVEY  cp{self._wpt_idx:04d}/{len(self._waypoints)-1}  "
                f"r{row}c{col}  dist={dist:.2f} m")

            # Tape boundary check before advancing toward this waypoint
            frame = self._get_bgr_frame()
            if frame is not None:
                alt = max(self._pose.pose.position.z, 1.0)
                # Only check 'fwd' — the forward boundary is the real safety stop.
                # Left/right checks were triggering on the perpendicular L-corner tape
                # arms when the drone was near the corner, causing false holds.
                d = self._detector.dist_to_line(
                    frame, 'fwd',
                    altitude_m=alt, fx=self._CAM_FX, fy=self._CAM_FY)
                if d is not None and d >= 0.15 and d < self._tape_stop_dist:
                    self.get_logger().warn(
                        f"Tape fwd={d:.2f}m < {self._tape_stop_dist:.2f}m "
                        f"at cp{self._wpt_idx} — boundary reached, holding here")
                    # Pin the waypoint to current position so the settle phase
                    # holds in place rather than commanding into the tape.
                    self._waypoints[self._wpt_idx] = (
                        self._pose.pose.position.x,
                        self._pose.pose.position.y)
                    self._settle_start  = self.get_clock().now()
                    self._arrived_since = None
                    return

            if dist <= self._wpt_radius:
                if self._arrived_since is None:
                    self._arrived_since = self.get_clock().now()
                elif self._secs(self._arrived_since) >= 0.3:
                    self._settle_start  = self.get_clock().now()
                    self._arrived_since = None
            else:
                self._arrived_since = None
            return

        # ── Settle + capture ──────────────────────────────────────
        self._pub_sp(tx, ty, self._target_alt)
        elapsed = self._secs(self._settle_start)
        self._tele(
            f"SURVEY  cp{self._wpt_idx:04d}  SETTLING  "
            f"{elapsed:.1f}/{self._wpt_settle_s:.1f} s")
        if elapsed >= self._wpt_settle_s:
            with self._capture_lock:
                if self._settle_start is None:
                    return
                snap_idx           = self._wpt_idx
                self._settle_start = None
                self._sp           = list(self._waypoints[snap_idx])
                self._wpt_idx      = snap_idx + 1
            self._capture_checkpoint(snap_idx)
            self._last_captured_xy = self._waypoints[snap_idx]

    # ─────────────────────────────────────────────────────────────
    # Landing sequence  (mirrors survey_mission)
    # ─────────────────────────────────────────────────────────────

    def _do_return(self):
        if self._vio_state >= GS_FAULT_MIN:
            self.get_logger().warn("VIO fault during RETURN — landing")
            self._phase = Phase.LAND
            return
        if self._goto_start is None:
            self._goto_start = self.get_clock().now()
        if self._secs(self._goto_start) > self._goto_timeout:
            self.get_logger().warn("Return timeout — landing")
            self._phase = Phase.LAND
            return
        dist = self._crawl_to(self._arm_x, self._arm_y)  # return to ARM point
        self._tele(f"RETURN  dist={dist:.2f} m → arm point")
        if dist <= self._goto_radius:
            if self._arrived_since is None:
                self._arrived_since = self.get_clock().now()
            elif self._secs(self._arrived_since) >= 1.0:
                if self._gate_cli.service_is_ready():
                    req = SetBool.Request(); req.data = False
                    self._gate_cli.call_async(req)
                self._gate_close_sent = True
                self._flow_settle_ts  = self.get_clock().now()
                self._arrived_since   = self._goto_start = None
                self._phase = Phase.FLOW_SETTLE
        else:
            self._arrived_since = None

    def _do_flow_settle(self):
        self._pub_sp(self._arm_x, self._arm_y, self._target_alt)
        if self._secs(self._flow_settle_ts) >= self._flow_settle_s:
            self._desc_z    = self._pose.pose.position.z
            self._desc_start = self.get_clock().now()
            self._phase      = Phase.DESCEND

    def _do_descend(self):
        self._desc_z = max(0.0, self._desc_z - self._descend_speed / self._sp_rate_hz)
        self._pub_sp(self._arm_x, self._arm_y, self._desc_z)
        alt = self._pose.pose.position.z
        self._tele(f"DESCEND  alt={alt:.2f}/{self._descend_handoff_alt:.2f} m")
        if alt <= self._descend_handoff_alt:
            self._phase = Phase.LAND
        elif self._secs(self._desc_start) > self._descend_timeout:
            self.get_logger().warn("Descend timeout — AUTO.LAND")
            self._phase = Phase.LAND

    def _do_land(self):
        if not self._gate_close_sent:
            if self._gate_cli.service_is_ready():
                req = SetBool.Request(); req.data = False
                self._gate_cli.call_async(req)
            self._gate_close_sent = True
        if not self._land_req:
            self._mode("AUTO.LAND")
            self._land_req = True
            self._phase    = Phase.DISARM

    def _do_disarm(self):
        if not self._state.armed:
            self.get_logger().info(
                f"Disarmed ✓  ({self._wpt_idx}/{len(self._waypoints)} checkpoints)\n"
                f"Data: {self._survey_run_dir}")
            self._phase = Phase.DONE

    def _do_flow_hold(self):
        """Hold on flow after VIO fault, then re-seed (up to max_revalidations)."""
        self._pub_vel_hold()
        if self._flow_hold_start is None:
            self._flow_hold_start = self.get_clock().now()
        if self._secs(self._flow_hold_start) < self._stable_of_secs:
            return
        self._flow_hold_start = None
        self._revalidations  += 1
        if self._revalidations > int(self._max_revalidations):
            self.get_logger().error(
                f"Max revalidations ({int(self._max_revalidations)}) exceeded — AUTO.LAND")
            self._phase = Phase.LAND
            return
        self.get_logger().warn(
            f"Re-validation {self._revalidations}/{int(self._max_revalidations)} "
            f"— resume cp{self._wpt_idx}")
        self._seed_sent = False
        self._phase     = Phase.SEED

    def _do_safe_manual(self):
        self.get_logger().info("SAFE MANUAL — pilot has control.",
                               throttle_duration_sec=5.0)

    # ─────────────────────────────────────────────────────────────
    # Motion helpers
    # ─────────────────────────────────────────────────────────────

    def _pub_body_vel(self, vx_body: float, vy_body: float, vz: float = 0.0):
        """
        Publish velocity in ENU by rotating body-frame (FLU) velocity by drone yaw.

        Body convention: +X = forward, +Y = left, +Z = up.
        Rotation to ENU (yaw θ measured CCW from East):
            vx_enu = vx_body·cos(θ) − vy_body·sin(θ)
            vy_enu = vx_body·sin(θ) + vy_body·cos(θ)

        Examples at θ=0 (facing East):
            body (−1, 0) → ENU (−1, 0) = West = backward ✓
            body ( 0, 1) → ENU (0, 1)  = North = drone-left ✓
            body ( 0,−1) → ENU (0,−1) = South = drone-right ✓
        """
        yaw    = math.radians(yaw_deg_from_quaternion(self._pose.pose.orientation))
        vx_enu = vx_body * math.cos(yaw) - vy_body * math.sin(yaw)
        vy_enu = vx_body * math.sin(yaw) + vy_body * math.cos(yaw)
        m = TwistStamped()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = "local_origin"
        m.twist.linear.x  = float(vx_enu)
        m.twist.linear.y  = float(vy_enu)
        m.twist.linear.z  = float(vz)
        self._vel_pub.publish(m)

    def _crawl_to(self, tx: float, ty: float) -> float:
        """Ramp position setpoint toward (tx,ty) at survey_speed. Returns dist to target."""
        if self._sp is None:
            self._sp = [self._pose.pose.position.x, self._pose.pose.position.y]
        step = self._survey_speed / self._sp_rate_hz
        dx, dy = tx - self._sp[0], ty - self._sp[1]
        d = math.hypot(dx, dy)
        if d > 1e-6:
            f = min(1.0, step / d)
            self._sp[0] += dx * f
            self._sp[1] += dy * f
        self._pub_sp(self._sp[0], self._sp[1], self._target_alt)
        return math.hypot(self._pose.pose.position.x - tx,
                          self._pose.pose.position.y - ty)

    def _get_bgr_frame(self):
        """Return a copy of the latest BGR frame (thread-safe). None if not available."""
        with self._frame_lock:
            return None if self._latest_bgr is None else self._latest_bgr.copy()

    def _pub_sp(self, x: float, y: float, z: float):
        m = PoseStamped()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = "local_origin"
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        # Always mirror the drone's current orientation — we never command a yaw
        # change.  PX4 sees "stay at your current heading" every tick, which gives
        # a stable yaw hold without any code-driven rotation.
        m.pose.orientation = self._pose.pose.orientation
        self._sp_pub.publish(m)

    def _pub_sp_with_yaw(self, x: float, y: float, z: float, yaw_rad: float):
        """Position setpoint with an explicit target yaw.

        Used only by ALIGN_YAW ROTATE/VERIFY phases to physically rotate the
        drone to face perpendicular to the back wall tape.  PX4 uses its
        internal yaw controller (typically ~30°/s) so the rotation is smooth
        and controlled — VIO stays healthy throughout.
        """
        m = PoseStamped()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = "local_origin"
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        # Yaw-only quaternion: roll=pitch=0
        half = float(yaw_rad) * 0.5
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = math.sin(half)
        m.pose.orientation.w = math.cos(half)
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
        now = time.monotonic()
        if now - self._last_print >= 1.0:
            self._last_print = now
            self.get_logger().info(line)


# ─────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = CornerSurveyMission()
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
