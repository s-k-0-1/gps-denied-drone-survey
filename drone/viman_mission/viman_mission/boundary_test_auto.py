#!/usr/bin/env python3
"""
boundary_test_auto — autonomous yellow-line LAWNMOWER-SURVEY mission.
Team Viman Rakshak / IRoC-U 2026.

Arena = a rectangle bounded by FOUR yellow lines. HOME (the take-off /
arm point) is inside it, near the CENTRE. The drone finds one corner,
then mows the whole area stripe-by-stripe, counting all four corners,
then returns to the centre and lands. It flies the survey at 2 m and
holds 0.5 m (stop_dist) off EVERY line — it never crosses one.

  IDLE        preflight gate (incl. boundary detector alive) + CH5 latch
  ARM         OFFBOARD + arm; HOME locked (arena-centre take-off point).
              Yaw floats through takeoff so an EKF2 mag reset can't snap it.
  TAKEOFF     flow-only climb to takeoff_alt (2.0 m) — camera init altitude
  STABLE_OF   zero-velocity flow settle
  HOVER_HOME  back to the arm point, settle, slew to a FIXED heading
              (mission_yaw_deg, default 0 deg) and hold it — that is the
              forward / grid axis, locked for the whole mission
  SEED        RTAB reset + frame alignment (vio_gate)
  VALIDATE    IF >= threshold with a small motion square
  HANDOVER    gate opens — camera now feeds PX4 (fused VIO)
  CLIMB       fused-VIO settle at cruise_alt (2.0 m)
  ACQ_BACK    from the centre, fly BACKWARD to the back line, soft-stop 0.5 m
  ACQ_LEFT    slide LEFT along the back line to the left line = Corner 1
  CORNER1_HOLD count Corner 1, hold hover_duration (10 s), start the survey
  SURVEY_STRIPE fly a forward/back stripe to the end line (front or back);
              if a side line is also near at the end -> a new corner is
              counted; then step right
  SURVEY_STEP shift stripe_step (1 m) RIGHT holding the end line at 0.5 m,
              then reverse the stripe direction; repeat until 4 corners
  RETURN      crawl back to HOME (arena centre) on fused VIO
  FLOW_SETTLE gate closed, EKF settles on flow at altitude
  DESCEND     OFFBOARD precision descent, X/Y locked on home
  LAND        AUTO.LAND for the final touchdown + disarm
  FLOW_HOLD   VIO-fault recovery: flow hold -> re-seed -> re-validate ->
              resume the phase we were in (up to max_revalidations)
  SAFE_MANUAL CH5 HIGH at any time -> STABILIZED, pilot has control
  DISARM / DONE

Corner counting: a corner = an end line (front/back) reached with a
perpendicular side line (left/right) within stop_dist + corner_margin.
Corner 1 is the acquired back-left; the first forward stripe adds the
front-left; the far (right) edge adds the last two. After target_corners
(4) the mission returns home and lands.

Safety (same layered set as the survey mission):
  * preflight gate blocks arming until FCU / pose-rate / RC / RTAB /
    vio_gate / boundary-detector / MAVROS are all healthy
  * CH5 HIGH -> instant STABILIZED (pilot wins, always)
  * CH5 latch: must go HIGH once, then LOW, to start
  * Ctrl+C -> emergency AUTO.LAND
  * VIO fault anywhere airborne -> FLOW_HOLD + revalidate + resume
  * boundary detector silent -> motion blocked; silent too long -> land
  THE DRONE NEVER CROSSES A LINE: every setpoint is vetoed per-line against
  motion toward any boundary past the standoff.

Run:
  ros2 launch viman_mission boundary_corner.launch.py
"""

import math
import signal
import time
from enum import Enum, auto

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import (PoseStamped, Quaternion, TwistStamped,
                               Vector3Stamped)
from mavros_msgs.msg import RCIn, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float32MultiArray, UInt8
from std_srvs.srv import SetBool, Trigger

from viman_mission.common import (qos_best_effort, qos_reliable,
                                  yaw_deg_from_quaternion)

# vio_gate states (keep in sync with vio_gate.GateState)
GS_VALIDATING = 2
GS_OPEN       = 3
GS_FAULT_MIN  = 4   # any state >= this is a fault


class Phase(Enum):
    IDLE        = auto()
    ARM         = auto()
    TAKEOFF     = auto()
    STABLE_OF   = auto()
    HOVER_HOME  = auto()
    SEED        = auto()
    VALIDATE    = auto()
    HANDOVER    = auto()
    CLIMB       = auto()
    ACQ_BACK      = auto()   # centre -> back line (find first boundary)
    ACQ_LEFT      = auto()   # back line -> left line = Corner 1 (back-left)
    CORNER1_HOLD  = auto()   # settle at Corner 1, hold, count it
    SURVEY_STRIPE = auto()   # fly a fwd/back stripe to the end line
    SURVEY_STEP   = auto()   # shift 1 m right between stripes
    RETURN      = auto()
    FLOW_SETTLE = auto()
    DESCEND     = auto()
    FLOW_HOLD   = auto()
    LAND        = auto()
    DISARM      = auto()
    SAFE_MANUAL = auto()
    DONE        = auto()


class BoundaryTestAuto(Node):

    def __init__(self):
        super().__init__("boundary_test_auto")

        self.declare_parameters("", [
            # --- flight profile ---
            ("takeoff_alt",          2.0),
            ("cruise_alt",           2.0),   # (was 3.0) search altitude — lower
                                             # so the tape is big enough in the
                                             # image to detect with real margin
            ("climb_speed_ms",       0.2),
            ("alt_tolerance",        0.12),
            ("at_alt_confirm_s",     1.5),
            ("stable_of_secs",       4.0),
            ("seed_timeout_s",       10.0),
            ("validate_if_min",      0.7),
            ("validate_hold_s",      5.0),
            ("validate_dip_grace_s", 1.0),
            ("validate_timeout_s",   60.0),
            ("motion_test",          True),
            ("motion_amp_m",         0.2),
            ("motion_leg_s",         4.0),
            ("handover_settle_s",    2.0),
            # --- yaw: rotate to a FIXED heading at home, hold it, then keep
            #     it locked for the whole mission (no arm-heading capture) ---
            ("mission_yaw_deg",      0.0),   # TARGET heading [deg] in the same
                                             # frame the pose yaw is read in
                                             # (MAVROS local / ENU). 0 = the
                                             # drone yaws until its own reported
                                             # yaw reads 0 deg. If the Pixhawk /
                                             # QGC compass should instead read 0
                                             # (North), set this to 90. Tune
                                             # here to match the cube.
            ("yaw_slew_dps",         15.0),
            ("yaw_align_tol_deg",    3.0),   # (was 8.0) tighter alignment before
                                             # locking cruise_yaw to the drone's
                                             # ACTUAL settled heading (see HOVER_HOME).
                                             # Motion is then commanded exactly along
                                             # the drone's nose axis, so forward,
                                             # backward, left and right all fly
                                             # straight instead of skewing.
            ("yaw_align_hold_s",     1.0),   # hold aligned this long before seed
            ("yaw_align_timeout_s",  25.0),  # give up aligning, proceed anyway
            # --- boundary approach (SOFT potential-field braking) ---
            ("forward_speed_ms",     0.15),  # (was 0.2) slower = more reaction
                                             # time between detection and standoff
            ("strafe_speed_ms",      0.2),
            ("corner_speed_ms",      0.15),
            ("slow_dist_m",          1.2),   # begin easing off here
            ("stop_dist_m",          0.5),   # SOFT standoff — stop this far back
                                             # off EVERY yellow line (was 0.7)
            ("push_gain",            0.6),   # proportional retreat speed if the
                                             # drone ends up INSIDE the standoff
            ("push_speed_max_ms",    0.2),   # cap on that retreat speed
            ("recover_margin_m",     0.25),  # when pushed inside the standoff,
                                             # ease back out to (stop_dist + this)
                                             # = a CLEAR safe distance, not just
                                             # the edge, so the drone does not
                                             # immediately re-touch the line
            ("recover_settle_s",     1.0),   # hold at that safe distance this
                                             # long before resuming the mission
                                             # (kills the aage-peeche limit cycle)
            ("ahead_thresh",         0.3),   # line counts as "front" if
            ("left_thresh",          0.3),   # / "left" beyond this alignment
            ("corner_gain",          0.8),   # equal-standoff gradient gain
            ("veto_deadband_m",      0.15),  # wall-hold slack: don't correct
                                             # while within this of the standoff
                                             # (kills chatter on jumpy tape)
            ("corner_tol_m",         0.2),   # both dists within this of stop
            ("corner_hold_s",        2.0),   # stable this long -> stabilised
            ("reach_confirm_s",      0.5),   # confirm a transition this long
            ("max_forward_m",        6.0),   # no front line in this -> abort
            ("max_strafe_m",         6.0),   # no left line in this -> abort
            ("line_lost_grace_s",    2.0),   # corner line lost this long -> back
            # --- LAWNMOWER SURVEY between the yellow lines (finds all 4
            #     corners). Home is at the CENTRE of the bounded arena:
            #     from there the drone goes BACKWARD to the back line, LEFT
            #     to the left line = Corner 1 (back-left), holds, then flies
            #     forward/backward stripes stepping RIGHT until 4 corners. ---
            ("survey_speed_ms",      0.2),   # stripe cruise speed (fwd/back)
            ("stripe_step_m",        1.0),   # right shift between stripes
            ("corner_margin_m",      0.35),  # a perpendicular side line within
                                             # (stop_dist + this) at a stripe end
                                             # = a corner (two lines meet)
            ("target_corners",       4),     # mission ends after this many
            ("max_stripe_m",         12.0),  # a stripe spans the whole arena
                                             # depth — abort only if no end line
                                             # is found in this far (must be >
                                             # arena depth; NOT max_forward=6 m)
            ("max_stripes",          12),    # safety cap on stripe count
            ("acq_settle_s",         1.0),   # settle at a reached line/corner
            ("acq_left_back_lost_s", 1.5),   # HOLD the LEFT strafe if the back
                                             # line disappears from view for
                                             # this many seconds (safety —
                                             # otherwise the drone can cruise
                                             # blind past the arena when the
                                             # detector loses tracking).
            ("line_bridge_s",        1.0),   # once a line we are flying TOWARD
                                             # was seen within slow_dist, a
                                             # detector dropout shorter than this
                                             # makes the drone HOLD (speed 0)
                                             # instead of charging blind through
                                             # the standoff — the yellow detector
                                             # is intermittent (thin/faint tape).
            # --- hover / return / land ---
            ("hover_duration",       10.0),  # hold at Corner 1 only (was 30)
            ("goto_radius_m",        0.2),
            ("goto_timeout_s",       60.0),
            ("flow_settle_s",        2.5),
            ("descend_speed_ms",     0.25),
            ("descend_handoff_alt_m", 0.3),
            ("descend_timeout_s",    15.0),
            # --- recovery ---
            ("max_revalidations",    6),
            # --- fail-safe ---
            ("boundary_stale_s",     1.0),
            ("stale_land_s",         5.0),
            # --- terminal yellow-line readout ---
            ("yellow_log_period_s",  1.0),   # how often to print the yellow
                                             # coverage % + detection status
            # --- infra ---
            ("sp_rate_hz",           20.0),
            ("rc_ch5_index",         4),
            ("rc_start_low",         1200),
            ("rc_interrupt_high",    1700),
            ("preflight_pose_hz_min", 15.0),
            ("rtab_odom_topic",      "/rtabmap/rtabmap/odom"),
            ("repulsion_topic",      "/viman/boundary/repulsion"),
            ("nearest_topic",        "/viman/boundary/nearest_m"),
            ("coverage_topic",       "/viman/boundary/coverage_pct"),
            ("lines_topic",          "/viman/boundary/lines"),
        ])
        gp = lambda n: self.get_parameter(n).value
        self._takeoff_alt   = float(gp("takeoff_alt"))
        self._cruise_alt    = float(gp("cruise_alt"))
        self._climb_speed   = float(gp("climb_speed_ms"))
        self._alt_tol       = float(gp("alt_tolerance"))
        self._at_alt_confirm_s = float(gp("at_alt_confirm_s"))
        self._stable_of_secs = float(gp("stable_of_secs"))
        self._seed_timeout_s = float(gp("seed_timeout_s"))
        self._if_min        = float(gp("validate_if_min"))
        self._validate_hold_s = float(gp("validate_hold_s"))
        self._dip_grace_s   = float(gp("validate_dip_grace_s"))
        self._validate_timeout_s = float(gp("validate_timeout_s"))
        self._motion_test   = bool(gp("motion_test"))
        self._motion_amp    = float(gp("motion_amp_m"))
        self._motion_leg_s  = float(gp("motion_leg_s"))
        self._handover_settle_s = float(gp("handover_settle_s"))
        self._mission_yaw_deg = float(gp("mission_yaw_deg"))
        self._yaw_slew_dps  = float(gp("yaw_slew_dps"))
        self._yaw_align_tol_deg = float(gp("yaw_align_tol_deg"))
        self._yaw_align_hold_s  = float(gp("yaw_align_hold_s"))
        self._yaw_align_timeout_s = float(gp("yaw_align_timeout_s"))
        self._forward_speed = float(gp("forward_speed_ms"))
        self._strafe_speed  = float(gp("strafe_speed_ms"))
        self._corner_speed  = float(gp("corner_speed_ms"))
        self._slow_dist     = float(gp("slow_dist_m"))
        self._stop_dist     = float(gp("stop_dist_m"))
        self._push_gain     = float(gp("push_gain"))
        self._push_max      = float(gp("push_speed_max_ms"))
        self._recover_margin = float(gp("recover_margin_m"))
        self._recover_settle_s = float(gp("recover_settle_s"))
        self._ahead_thresh  = float(gp("ahead_thresh"))
        self._left_thresh   = float(gp("left_thresh"))
        self._corner_gain   = float(gp("corner_gain"))
        self._veto_deadband = float(gp("veto_deadband_m"))
        self._corner_tol    = float(gp("corner_tol_m"))
        self._corner_hold_s = float(gp("corner_hold_s"))
        self._reach_confirm_s = float(gp("reach_confirm_s"))
        self._max_forward   = float(gp("max_forward_m"))
        self._max_strafe    = float(gp("max_strafe_m"))
        self._line_lost_grace_s = float(gp("line_lost_grace_s"))
        self._survey_speed  = float(gp("survey_speed_ms"))
        self._stripe_step   = float(gp("stripe_step_m"))
        self._corner_margin = float(gp("corner_margin_m"))
        self._target_corners = int(gp("target_corners"))
        self._max_stripe_m  = float(gp("max_stripe_m"))
        self._max_stripes   = int(gp("max_stripes"))
        self._acq_settle_s  = float(gp("acq_settle_s"))
        self._acq_left_back_lost_s = float(gp("acq_left_back_lost_s"))
        self._line_bridge_s = float(gp("line_bridge_s"))
        self._hover_duration = float(gp("hover_duration"))
        self._goto_radius   = float(gp("goto_radius_m"))
        self._goto_timeout  = float(gp("goto_timeout_s"))
        self._flow_settle_s = float(gp("flow_settle_s"))
        self._descend_speed = float(gp("descend_speed_ms"))
        self._descend_handoff_alt = float(gp("descend_handoff_alt_m"))
        self._descend_timeout = float(gp("descend_timeout_s"))
        self._max_revalidations = int(gp("max_revalidations"))
        self._boundary_stale_s = float(gp("boundary_stale_s"))
        self._stale_land_s  = float(gp("stale_land_s"))
        self._yellow_log_period = float(gp("yellow_log_period_s"))
        self._sp_rate_hz    = float(gp("sp_rate_hz"))
        self._rc_ch5_idx    = int(gp("rc_ch5_index"))
        self._rc_start_low  = int(gp("rc_start_low"))
        self._rc_interrupt_high = int(gp("rc_interrupt_high"))
        self._pose_hz_min   = float(gp("preflight_pose_hz_min"))
        rtab_topic          = str(gp("rtab_odom_topic"))
        rep_topic           = str(gp("repulsion_topic"))
        near_topic          = str(gp("nearest_topic"))
        cov_topic           = str(gp("coverage_topic"))
        lines_topic         = str(gp("lines_topic"))

        self._phase, self._last_phase = Phase.IDLE, None

        # References
        self._home_x = self._home_y = 0.0
        self._arm_heading_q  = None      # heading captured at ARM (reference)
        self._hold_heading_q = None      # active heading target
        self._cmd_yaw_rad    = None      # slewed commanded yaw
        self._cruise_yaw     = 0.0       # locked arm-time yaw [rad] (grid axis)
        self._validate_anchor = (0.0, 0.0)
        self._sp = None                  # crawled setpoint [x, y]
        self._sp_z = 0.0                 # crawled altitude setpoint
        self._hold_xy = None             # hover / return hold ref
        self._fwd_start = None           # (x, y) at FORWARD entry
        self._strafe_start = None        # (x, y) at STRAFE_LEFT entry
        self._ret_sp = None
        self._mission_started = False    # first survey stripe reached
        self._resume_phase = None        # phase to resume after revalidation

        # --- lawnmower survey state ---
        self._corners_found = 0          # how many arena corners located
        self._acq_start = None           # (x, y) at an acquisition-leg entry
        self._settle_since = None        # generic "reached, settling" timer
        self._stripe_dir = 1             # +1 forward, -1 backward
        self._stripe_count = 0           # stripes flown (safety cap)
        self._step_start = None          # (x, y) at a right-step entry
        self._corner_counted_end = False  # already counted a corner at this
                                          # stripe end (reset on next stripe)
        self._appr_ns = 0                # last time the approached line was seen
        self._appr_dist = None           # its last-seen distance
        self._appr_acquired = False      # latched once the line is seen in range
        # ACQ_LEFT safety: track how long the BACK line has been invisible.
        # If the detector loses the back boundary during strafe, we can no
        # longer confirm we are within the standoff — HOLD instead of
        # cruising blind. Bridges brief flickers, stops on sustained loss.
        self._acq_left_back_lost_since = None
        # --- too-close recovery latch (ease away from a breached line, hold at
        #     a safe distance, then resume) ---
        self._breaching = False          # True while actively easing back out
        self._breach_since = None        # settle timer once safe distance reached

        # Live data
        self._pose = PoseStamped()
        self._pose.pose.orientation.w = 1.0
        self._state = State()
        self._rc = ()
        self._vio_state = 255
        self._init_factor = 0.0
        self._pose_stamps = []
        self._last_rtab_ns = 0
        self._ch5_latched = False

        # Boundary data
        self._rep_body = (0.0, 0.0)
        self._nearest = -1.0
        self._coverage = 0.0
        self._lines = []                 # per-line [{dist,nx,ny,strength}]
        self._last_boundary_ns = 0

        # Timers / flags
        self._at_alt_since = None
        self._stable_since = None
        self._hover_home_since = None
        self._yaw_align_start = None
        self._yaw_aligned_since = None
        self._seed_sent = False
        self._seed_start = None
        self._validate_start = None
        self._if_good_since = None
        self._if_low_since = None
        self._handover_sent = False
        self._handover_start = None
        self._climb_confirm_since = None
        self._reach_since = None
        self._detect_since = None
        self._corner_stable_since = None
        self._corner_body = None            # (fwd, left) metres, body-FLU
        self._corner_ns = 0                 # last corner msg time (staleness)
        self._line_lost_since = None
        self._hover_start = None
        self._ret_start = None
        self._ret_arrived_since = None
        self._settle_start = None
        self._desc_z = 0.0
        self._desc_start = None
        self._stale_since = None
        self._flow_hold_start = None
        self._offboard_req = self._arm_req = False
        self._gate_close_sent = False
        self._land_req = False
        self._ctrl_c = False
        self._last_print = 0.0
        signal.signal(signal.SIGINT, self._sigint)

        cb = ReentrantCallbackGroup()
        qos_be, qos_rel = qos_best_effort(), qos_reliable()

        self.create_subscription(State, "/mavros/state",
                                 self._state_cb, qos_rel, callback_group=cb)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._pose_cb, qos_be, callback_group=cb)
        self.create_subscription(RCIn, "/mavros/rc/in",
                                 self._rc_cb, qos_be, callback_group=cb)
        self.create_subscription(Odometry, rtab_topic, self._rtab_alive_cb,
                                 qos_rel, callback_group=cb)
        self.create_subscription(UInt8, "/viman/vio_state",
                                 self._vio_state_cb, 10, callback_group=cb)
        self.create_subscription(Float32, "/viman/init_factor",
                                 self._if_cb, 10, callback_group=cb)
        self.create_subscription(Vector3Stamped, rep_topic,
                                 self._rep_cb, 10, callback_group=cb)
        self.create_subscription(Float32, near_topic,
                                 self._near_cb, 10, callback_group=cb)
        self.create_subscription(Float32, cov_topic,
                                 self._cov_cb, 10, callback_group=cb)
        self.create_subscription(Float32MultiArray, lines_topic,
                                 self._lines_cb, 10, callback_group=cb)
        self.create_subscription(Vector3Stamped, "/viman/boundary/corner",
                                 self._corner_cb, 10, callback_group=cb)

        self._sp_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)
        self._vel_pub = self.create_publisher(
            TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10)

        self._arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming",
                                           callback_group=cb)
        self._mode_cli = self.create_client(SetMode, "/mavros/set_mode",
                                            callback_group=cb)
        self._seed_cli = self.create_client(Trigger, "/viman/seed",
                                            callback_group=cb)
        self._gate_cli = self.create_client(SetBool, "/viman/gate",
                                            callback_group=cb)

        self._handlers = {
            Phase.IDLE: self._do_idle, Phase.ARM: self._do_arm,
            Phase.TAKEOFF: self._do_takeoff,
            Phase.STABLE_OF: self._do_stable_of,
            Phase.HOVER_HOME: self._do_hover_home,
            Phase.SEED: self._do_seed, Phase.VALIDATE: self._do_validate,
            Phase.HANDOVER: self._do_handover,
            Phase.CLIMB: self._do_climb,
            Phase.ACQ_BACK: self._do_acq_back,
            Phase.ACQ_LEFT: self._do_acq_left,
            Phase.CORNER1_HOLD: self._do_corner1_hold,
            Phase.SURVEY_STRIPE: self._do_survey_stripe,
            Phase.SURVEY_STEP: self._do_survey_step,
            Phase.RETURN: self._do_return,
            Phase.FLOW_SETTLE: self._do_flow_settle,
            Phase.DESCEND: self._do_descend,
            Phase.FLOW_HOLD: self._do_flow_hold,
            Phase.LAND: self._do_land, Phase.DISARM: self._do_disarm,
            Phase.SAFE_MANUAL: self._do_safe_manual,
            Phase.DONE: lambda: None,
        }
        self.create_timer(1.0 / self._sp_rate_hz, self._loop,
                          callback_group=cb)
        # Always-on terminal readout of the yellow-line detection.
        self.create_timer(self._yellow_log_period, self._yellow_status,
                          callback_group=cb)
        self.get_logger().info(
            f"BoundaryTestAuto (LAWNMOWER-SURVEY mission): fly at "
            f"{self._cruise_alt:.1f} m, stop {self._stop_dist:.2f} m off every "
            f"yellow line. Home = arena CENTRE. Path: back->left = Corner 1 "
            f"(hold {self._hover_duration:.0f} s), then forward/back stripes "
            f"stepping {self._stripe_step:.1f} m right until "
            f"{self._target_corners} corners are found, then return home + land."
            " SAFETY LATCH: CH5 HIGH once, then LOW to start.")
        self.get_logger().warn(
            ">>> SURVEY build: counts all 4 corners via two-line meetings and "
            "sweeps the whole bounded area. WALL-HOLD veto keeps 0.5 m off "
            "every line (never crosses). If you do NOT see THIS line at "
            "startup, the new code is NOT running — rebuild & re-source.")

    # ── Callbacks ─────────────────────────────────────────────────
    def _state_cb(self, m): self._state = m
    def _vio_state_cb(self, m): self._vio_state = m.data
    def _if_cb(self, m): self._init_factor = m.data

    def _rtab_alive_cb(self, m):
        self._last_rtab_ns = self.get_clock().now().nanoseconds

    def _rep_cb(self, m: Vector3Stamped):
        self._rep_body = (m.vector.x, m.vector.y)
        self._last_boundary_ns = self.get_clock().now().nanoseconds

    def _near_cb(self, m: Float32):
        self._nearest = m.data

    def _cov_cb(self, m: Float32):
        self._coverage = m.data

    def _lines_cb(self, m: Float32MultiArray):
        d = list(m.data)
        lines = []
        if d:
            n = int(d[0])
            for i in range(n):
                b = 1 + 4 * i
                if b + 3 < len(d):
                    lines.append({'dist': d[b], 'nx': d[b + 1],
                                  'ny': d[b + 2], 'strength': d[b + 3]})
        # Light low-pass on each line's distance so the corner hold is steady:
        # the front line reads jumpy (head-on + downward camera). Only smooth
        # while the line count is stable, so a steady hold reads steady.
        if len(lines) == len(self._lines):
            for i, L in enumerate(lines):
                L['dist'] = 0.6 * L['dist'] + 0.4 * self._lines[i]['dist']
        self._lines = lines
        self._last_boundary_ns = self.get_clock().now().nanoseconds

    def _corner_cb(self, m: Vector3Stamped):
        """Inner-corner vertex in body-FLU metres (x=forward, y=left);
        z=1.0 when a corner is actually seen. This is the reliable geometric
        target — far better than guessing 'both arms' from per-line normals."""
        if m.vector.z > 0.5:
            bx, by = m.vector.x, m.vector.y
            if self._corner_body is None:
                self._corner_body = [bx, by]
            else:                       # light low-pass, same as line dists
                self._corner_body[0] = 0.6 * bx + 0.4 * self._corner_body[0]
                self._corner_body[1] = 0.6 * by + 0.4 * self._corner_body[1]
            self._corner_ns = self.get_clock().now().nanoseconds
        else:
            self._corner_body = None

    def _corner_fresh(self):
        """True if a corner vertex was seen within the last 0.6 s."""
        if self._corner_body is None:
            return False
        age = (self.get_clock().now().nanoseconds - self._corner_ns) * 1e-9
        return age < 0.6

    def _pose_cb(self, m):
        self._pose = m
        t = self.get_clock().now().nanoseconds
        self._pose_stamps.append(t)
        while self._pose_stamps and self._pose_stamps[0] < t - 2_000_000_000:
            self._pose_stamps.pop(0)

    def _rc_cb(self, m):
        self._rc = m.channels
        if self._ch5() >= 1300:
            self._ch5_latched = True
        if self._phase in (Phase.SAFE_MANUAL, Phase.DONE, Phase.IDLE):
            return
        if self._ch5() >= self._rc_interrupt_high:
            self.get_logger().warn(
                f"⚠ RC INTERRUPT CH5={self._ch5()} - STABILIZED")
            self._mode("STABILIZED")
            self._phase = Phase.SAFE_MANUAL

    def _sigint(self, *_):
        if self._phase in (Phase.IDLE, Phase.DONE, Phase.SAFE_MANUAL,
                           Phase.DISARM) or self._ctrl_c:
            raise KeyboardInterrupt
        self.get_logger().warn("Ctrl+C - emergency AUTO.LAND")
        self._ctrl_c = True

    # ── Loop ──────────────────────────────────────────────────────
    def _loop(self):
        if self._phase != self._last_phase:
            self.get_logger().info(f"== Phase: {self._phase.name} ==")
            self._last_phase = self._phase
            self._sp = None
            self._appr_ns = 0            # reset the line-approach latch
            self._appr_dist = None
            self._appr_acquired = False
            self._breaching = False      # reset the too-close recovery latch
            self._breach_since = None

        if self._ctrl_c and self._phase not in (
                Phase.LAND, Phase.DISARM, Phase.DONE, Phase.SAFE_MANUAL):
            self._phase = Phase.LAND
            return
        self._handlers[self._phase]()

    # ── Boundary / geometry helpers ───────────────────────────────
    def _boundary_age(self) -> float:
        if self._last_boundary_ns == 0:
            return float("inf")
        return (self.get_clock().now().nanoseconds
                - self._last_boundary_ns) / 1e9

    def _dir_vectors(self):
        """Forward and LEFT unit vectors (ENU) for the locked arm heading."""
        y = self._cruise_yaw
        return (math.cos(y), math.sin(y), -math.sin(y), math.cos(y))

    def _lines_enu(self):
        """Per-line (dist, ex, ey) with (ex, ey) a UNIT push-away vector in
        ENU. Falls back to the blended repulsion if no per-line data.

        Uses the LOCKED _cruise_yaw (not the live pose yaw) for the body→ENU
        rotation, matching what _dir_vectors() does. Boundary vectors and
        motion vectors therefore live in the SAME reference frame — a small
        live-yaw wobble a few degrees off the locked heading no longer
        rotates the boundary vectors relative to the motion axes, so
        forward / backward / left / right runs stay straight.  Pre-lock
        _cruise_yaw is 0.0 (a placeholder); boundary logic isn't consumed
        by motion phases before HOVER_HOME locks the yaw."""
        yaw = self._cruise_yaw
        cy, sy = math.cos(yaw), math.sin(yaw)
        out = []
        for L in self._lines:
            out.append((L['dist'],
                        L['nx'] * cy - L['ny'] * sy,
                        L['nx'] * sy + L['ny'] * cy))
        if not out and self._nearest >= 0.0:
            rx, ry = self._rep_body
            mag = math.hypot(rx, ry)
            if mag > 1e-6:
                out.append((self._nearest,
                            (rx * cy - ry * sy) / mag,
                            (rx * sy + ry * cy) / mag))
        return out

    def _lines_dir(self):
        """Per-line dict {dist, ex, ey, fwd, left} where fwd/left = how much
        moving forward / left carries the drone TOWARD that line (1 = dead
        ahead / dead left)."""
        fx, fy, lx, ly = self._dir_vectors()
        out = []
        for (dist, ex, ey) in self._lines_enu():
            out.append({'dist': dist, 'ex': ex, 'ey': ey,
                        'fwd': -(fx * ex + fy * ey),
                        'left': -(lx * ex + ly * ey)})
        return out

    def _classify(self, lines=None):
        """Sort the visible lines into the four body directions and return the
        NEAREST line in each. 'front'/'back' use the fwd alignment, 'left'/
        'right' use the left alignment. Returns a dict with keys front, back,
        left, right (each a line dict or None)."""
        if lines is None:
            lines = self._lines_dir()
        best = {'front': None, 'back': None, 'left': None, 'right': None}
        for L in lines:
            cands = []
            if L['fwd'] >= self._ahead_thresh:
                cands.append('front')
            if L['fwd'] <= -self._ahead_thresh:
                cands.append('back')
            if L['left'] >= self._left_thresh:
                cands.append('left')
            if L['left'] <= -self._left_thresh:
                cands.append('right')
            for k in cands:
                if best[k] is None or L['dist'] < best[k]['dist']:
                    best[k] = L
        return best

    def _side_line_near(self, cls):
        """True if a perpendicular SIDE line (left or right) is within
        (stop_dist + corner_margin) — the second arm of a corner."""
        lim = self._stop_dist + self._corner_margin
        for k in ('left', 'right'):
            if cls[k] is not None and cls[k]['dist'] <= lim:
                return True
        return False

    def _approach(self, line):
        """Speed control for a line the drone is flying TOWARD. Returns a 0..1
        speed factor and, as a side effect, sets self._appr_acquired once the
        line has been seen within slow_dist.

        The yellow tape detects only weakly and intermittently (thin/faint,
        small FOV at 2 m) — it drops out for SECONDS at a time. The old
        time-limited bridge (line_bridge_s) let the drone resume full-speed
        cruise after the gap and blow PAST the standoff. Fix = an ACQUIRE
        LATCH: the moment the line is first seen within slow_dist we 'commit'
        to it; from then on ANY dropout means HOLD (factor 0), never cruise
        into where the line was. Before it is ever seen we cruise (factor 1)
        to search; while visible we brake smoothly toward the standoff."""
        now = self.get_clock().now().nanoseconds
        span = max(1e-3, self._slow_dist - self._stop_dist)
        if line is not None:
            self._appr_ns = now
            self._appr_dist = line['dist']
            if line['dist'] <= self._slow_dist:
                self._appr_acquired = True
            return self._smoothstep((line['dist'] - self._stop_dist) / span)
        if self._appr_acquired:
            return 0.0        # committed to a line, now lost → HOLD (never cruise in)
        return 1.0            # not yet acquired → keep cruising to search

    def _count_corner(self, why):
        """Register a newly reached arena corner (debounced per stripe end)."""
        self._corners_found += 1
        self.get_logger().info(
            f"★ CORNER {self._corners_found}/{self._target_corners} found "
            f"({why}).")

    @staticmethod
    def _smoothstep(x):
        x = max(0.0, min(1.0, x))
        return x * x * (3.0 - 2.0 * x)

    def _apply_boundary(self, lines, dt):
        """Hold the drone at the standoff like a WALL, exactly like the side
        line: a hard per-line veto clamps the setpoint so it never leads
        toward a line past the standoff (never cross) — and NOTHING else.

        The old 'soft retreat' loop (which actively drove the setpoint
        backward whenever a line read closer than the standoff) is REMOVED.
        The FRONT line's distance reads jumpy (head-on approach + downward
        camera), so that retreat kept re-firing on the low readings and
        marched the drone backward until the line left the frame. Now every
        line simply STOPS the drone at the standoff and holds it there:
        toward-motion is blocked, backward / sideways stay free."""
        px, py = self._pose.pose.position.x, self._pose.pose.position.y
        for L in lines:
            lead_x = self._sp[0] - px
            lead_y = self._sp[1] - py
            toward = -(lead_x * L['ex'] + lead_y * L['ey'])
            allow = L['dist'] - self._stop_dist
            if allow >= 0.0:
                # OUTSIDE the standoff: hard clamp so the setpoint can never
                # lead past the standoff toward the line (never cross).
                if toward > allow:
                    excess = toward - allow
                    self._sp[0] += L['ex'] * excess
                    self._sp[1] += L['ey'] * excess
            else:
                # INSIDE the standoff (tape usually isn't seen until the drone
                # is already close). While a dedicated breach recovery is
                # running (_breach_recover), IT owns easing us back out — skip
                # here so the same line is not corrected twice.
                if self._breaching:
                    continue
                # ease the setpoint back OUT to the standoff. A deadband means
                # small overshoots are tolerated, and the outward step is capped
                # to corner_speed so the retreat is SMOOTH and doesn't chatter on
                # jumpy line-distance readings.
                deficit = -allow                     # how far inside (>0)
                if deficit > self._veto_deadband:
                    excess = max(toward, 0.0) + min(
                        deficit - self._veto_deadband,
                        self._corner_speed * dt)
                    self._sp[0] += L['ex'] * excess
                    self._sp[1] += L['ey'] * excess

    def _breach_recover(self, lines, dt):
        """TOO-CLOSE RECOVERY. The tape often is not seen until the drone is
        already inside the standoff, and its distance reads jumpy — the drone
        would then sit right on the line doing an aage-peeche limit cycle that
        never settled (and eventually shook VIO loose). Requested behaviour:

          1. drone gets TOO CLOSE to a yellow line  ->
          2. gently push it AWAY from that line (push_gain, capped at
             push_speed_max) until it reaches a SAFE distance
             (stop_dist + recover_margin),
          3. HOLD there briefly (recover_settle_s) so the motion settles,
          4. release -> the phase resumes its normal drive, which glides the
             drone slowly back to the standoff and holds (then left / corner).

        While a recovery is active the phase MUST NOT add its own forward /
        strafe drive (the caller returns as soon as this returns True), so
        nothing fights the retreat. The hard 'never cross' veto stays live on
        every OTHER line throughout. Returns True while recovering."""
        if not lines:
            # no line in view -> cannot be breached; drop any stale latch
            self._breaching = False
            self._breach_since = None
            return False

        nearest = min(lines, key=lambda L: L['dist'])   # the line we're closest to
        safe = self._stop_dist + self._recover_margin

        if not self._breaching:
            # engage only on a REAL breach (inside by more than the deadband) so
            # noise at the standoff can't trip it
            if (self._stop_dist - nearest['dist']) > self._veto_deadband:
                self._breaching = True
                self._breach_since = None
                self.get_logger().warn(
                    f"Too close to a line ({nearest['dist']:.2f} m < "
                    f"{self._stop_dist:.2f} m) — easing back to {safe:.2f} m "
                    "before continuing.")
            else:
                return False                             # not breached

        # --- active recovery: ease AWAY from the nearest (breached) line ---
        # ex,ey already point AWAY from the line (toward the drone / nadir).
        err = safe - nearest['dist']                    # >0 = still too close
        if err > 0.0:
            self._breach_since = None                   # not at safe distance yet
            v = min(self._push_max, self._push_gain * err)
            self._sp[0] += nearest['ex'] * v * dt
            self._sp[1] += nearest['ey'] * v * dt
        else:
            # reached the safe distance -> hold here to settle, then release
            if self._breach_since is None:
                self._breach_since = self.get_clock().now()
            elif self._secs(self._breach_since) >= self._recover_settle_s:
                self._breaching = False
                self._breach_since = None
                self.get_logger().info(
                    f"Back to safe distance ({nearest['dist']:.2f} m) — "
                    "resuming.")
                return False

        # keep the hard never-cross clamp live on every OTHER line meanwhile
        # (its inside-standoff branch is skipped while _breaching, so the
        #  breached line is only pushed once — by the retreat above).
        self._apply_boundary(lines, dt)
        return True

    def _hold_offset(self, L, dt):
        """Per-line standoff hold that will NOT oscillate on jumpy tape.

        The old version chased the exact standoff both ways (pull toward when
        too far, push away when too close). Because the tape distance reads
        jumpy (±0.2 m), that chase turned into an aage-peeche limit cycle right
        at the line. Fixed with a WIDE, ASYMMETRIC band:
          • too close (< standoff − deadband): firmly but smoothly ease AWAY to
            the safe distance (this is the "line ke paas aa gaya → thoda door
            bhejo" behaviour),
          • clearly too far (> standoff + deadband + 0.3 m): drift back TOWARD
            the line at HALF speed (slow, no overshoot),
          • anywhere in between: HOLD — no correction, so noise can't start a
            back-and-forth.
        ex,ey point AWAY from the line (toward the drone)."""
        err = L['dist'] - self._stop_dist           # >0 too far, <0 too close
        if err < -self._veto_deadband:
            # too close → move away, capped at corner_speed (smooth)
            v = min(self._corner_speed, self._corner_gain * (-err))
            return L['ex'] * v * dt, L['ey'] * v * dt
        if err > self._veto_deadband + 0.3:
            # clearly too far → ease back toward the line gently (half speed)
            v = min(0.5 * self._corner_speed, 0.5 * self._corner_gain * err)
            return -L['ex'] * v * dt, -L['ey'] * v * dt
        return 0.0, 0.0                             # safe band → hold steady

    def _vio_fault(self):
        """If a VIO fault is up, bank the current phase and drop to FLOW_HOLD.
        Returns True when handled (caller should return)."""
        if self._vio_state >= GS_FAULT_MIN:
            self.get_logger().error(
                f"VIO fault (gate {self._vio_state}) in {self._phase.name} "
                "- flow hold + revalidate")
            self._resume_phase = self._phase
            self._flow_hold_start = None
            self._phase = Phase.FLOW_HOLD
            return True
        return False

    def _boundary_dead(self):
        """If the detector went silent, hold; land if silent too long.
        Returns True when handled (caller should return)."""
        if self._boundary_age() <= self._boundary_stale_s:
            self._stale_since = None
            return False
        if self._stale_since is None:
            self._stale_since = self.get_clock().now()
        if self._secs(self._stale_since) > self._stale_land_s:
            self.get_logger().error(
                "Boundary detector silent too long - returning to land")
            self._begin_return()
            return True
        self.get_logger().warn("Boundary detector stale - holding",
                               throttle_duration_sec=1.0)
        hx = self._sp[0] if self._sp else self._pose.pose.position.x
        hy = self._sp[1] if self._sp else self._pose.pose.position.y
        self._pub_sp(hx, hy, self._cruise_alt)
        return True

    # ── Preflight ─────────────────────────────────────────────────
    def _preflight_failures(self):
        f = []
        if not self._state.connected:
            f.append("FCU")
        if len(self._pose_stamps) / 2.0 < self._pose_hz_min:
            f.append("pose rate")
        if len(self._rc) <= self._rc_ch5_idx:
            f.append("no RC")
        now = self.get_clock().now().nanoseconds
        if now - self._last_rtab_ns > 2_000_000_000:
            f.append("RTAB silent")
        if self._vio_state == 255:
            f.append("vio_gate silent")
        if self._boundary_age() > 2.0:
            f.append("boundary detector silent")
        if not self._seed_cli.service_is_ready():
            f.append("/viman/seed")
        if not self._arm_cli.service_is_ready() \
                or not self._mode_cli.service_is_ready():
            f.append("MAVROS srv")
        return f

    def _do_idle(self):
        self._pub_sp(0.0, 0.0, 0.3)
        fails = self._preflight_failures()
        if fails:
            self.get_logger().warn("PREFLIGHT BLOCKED: " + ", ".join(fails),
                                   throttle_duration_sec=5.0)
            return
        if not self._ch5_latched:
            self.get_logger().info(
                "Preflight OK - flip CH5 HIGH once then LOW to start",
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
            self._home_x = self._pose.pose.position.x
            self._home_y = self._pose.pose.position.y
            # Yaw is NOT captured or held here. It floats through takeoff
            # (refs None) so an EKF2 mag reset can't snap the airframe. The
            # mission heading is a FIXED value (mission_yaw_deg, default 0)
            # that we slew onto — and hold — at home in HOVER_HOME.
            self._arm_heading_q  = None
            self._hold_heading_q = None
            self._cmd_yaw_rad    = None
            self.get_logger().info(
                f"Armed. HOME=({self._home_x:.2f},{self._home_y:.2f})  "
                f"current heading="
                f"{yaw_deg_from_quaternion(self._pose.pose.orientation):.1f} deg"
                f" -> will lock to fixed {self._mission_yaw_deg:.1f} deg at home")
            self._phase = Phase.TAKEOFF

    def _do_takeoff(self):
        self._pub_sp(self._home_x, self._home_y, self._takeoff_alt)
        alt = self._pose.pose.position.z
        self._tele(f"TAKEOFF(flow) {alt:.2f}/{self._takeoff_alt:.1f} m")
        if abs(alt - self._takeoff_alt) <= self._alt_tol:
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
        """Fly back to the arm point at takeoff altitude, settle, then slew to
        a FIXED heading (mission_yaw_deg, default 0 deg) and HOLD it there
        until the reported yaw is actually aligned, before seeding. That 0 deg
        is then locked for the rest of the mission (forward / grid axis)."""
        self._pub_sp(self._home_x, self._home_y, self._takeoff_alt)
        px, py = self._pose.pose.position.x, self._pose.pose.position.y
        dist = math.hypot(px - self._home_x, py - self._home_y)

        # ── Stage 1: get home and settle before touching yaw ──────────
        if self._hold_heading_q is None:
            self._tele(f"HOVER_HOME dist={dist:.2f} m -> home")
            if dist <= self._goto_radius:
                if self._hover_home_since is None:
                    self._hover_home_since = self.get_clock().now()
                elif self._secs(self._hover_home_since) >= 2.0:
                    # Lock the FIXED mission heading and start slewing to it.
                    self._cruise_yaw = math.radians(self._mission_yaw_deg)
                    self._hold_heading_q = self._yaw_quat(self._cruise_yaw)
                    self._cmd_yaw_rad = math.radians(
                        yaw_deg_from_quaternion(self._pose.pose.orientation))
                    self._yaw_align_start = self.get_clock().now()
                    self._yaw_aligned_since = None
                    self.get_logger().info(
                        f"At home, settled. Slewing to FIXED "
                        f"{self._mission_yaw_deg:.1f} deg heading, then holding "
                        "until aligned before seeding.")
            else:
                self._hover_home_since = None
            return

        # ── Stage 2: _pub_sp (above) is slewing yaw toward the target —
        #    wait until the reported yaw actually reaches it (held briefly),
        #    or until the alignment times out. ─────────────────────────
        cur = yaw_deg_from_quaternion(self._pose.pose.orientation)
        err = abs((cur - self._mission_yaw_deg + 180.0) % 360.0 - 180.0)
        self._tele(f"HOVER_HOME yaw={cur:.1f} -> {self._mission_yaw_deg:.1f} deg"
                   f" (err={err:.1f} deg)")
        if err <= self._yaw_align_tol_deg:
            if self._yaw_aligned_since is None:
                self._yaw_aligned_since = self.get_clock().now()
        else:
            self._yaw_aligned_since = None
        aligned = (self._yaw_aligned_since is not None and
                   self._secs(self._yaw_aligned_since) >= self._yaw_align_hold_s)
        timeout = self._secs(self._yaw_align_start) > self._yaw_align_timeout_s
        if aligned or timeout:
            # RE-LOCK to the drone's ACTUAL settled yaw (not the intended
            # target).  PX4 stops here within yaw_align_tol_deg of the
            # target — usually a couple of degrees off.  If we keep the
            # mission frame at the target, every subsequent "forward" step
            # is skewed by that residual error and the drone slides
            # diagonally instead of flying straight ahead.  Anchoring the
            # mission frame to the actual heading makes forward = nose
            # direction exactly, and left / right / backward all fly
            # straight along the drone's own body axes.  Also freeze
            # _cmd_yaw_rad and _hold_heading_q to this same value so PX4
            # holds it for the whole mission — every setpoint from now on
            # commands this exact yaw.
            actual_yaw_rad = math.radians(cur)
            self._cruise_yaw = actual_yaw_rad
            self._cmd_yaw_rad = actual_yaw_rad
            self._hold_heading_q = self._yaw_quat(actual_yaw_rad)
            self.get_logger().info(
                f"Yaw locked at ACTUAL {cur:.1f} deg "
                f"(target was {self._mission_yaw_deg:.1f}, "
                f"{'aligned' if aligned else 'timeout - proceeding anyway'}). "
                "Mission frame anchored here — forward / back / left / right "
                "now fly straight along the drone's real body axes. Seeding.")
            self._seed_sent = False
            self._phase = Phase.SEED

    def _do_seed(self):
        self._pub_vel_hold()
        if not self._seed_sent:
            if not self._seed_cli.service_is_ready():
                return
            self._seed_cli.call_async(Trigger.Request())
            self._seed_sent = True
            self._seed_start = self.get_clock().now()
            return
        if self._vio_state == GS_VALIDATING:
            self._validate_start = self.get_clock().now()
            self._if_good_since = self._if_low_since = None
            self._validate_anchor = (self._pose.pose.position.x,
                                     self._pose.pose.position.y)
            self._phase = Phase.VALIDATE
        elif self._secs(self._seed_start) > self._seed_timeout_s:
            self.get_logger().warn("Seed timeout - flow hold, retry")
            self._flow_hold_start = None
            self._phase = Phase.FLOW_HOLD

    def _motion_offset(self):
        if not self._motion_test:
            return 0.0, 0.0
        c = ((0.0, 0.0), (self._motion_amp, 0.0),
             (self._motion_amp, self._motion_amp), (0.0, self._motion_amp))
        leg = int(self._secs(self._validate_start) / self._motion_leg_s)
        return c[leg % 4]

    def _do_validate(self):
        ox, oy = self._motion_offset()
        ax, ay = self._validate_anchor
        self._pub_sp(ax + ox, ay + oy, self._takeoff_alt)
        if self._vio_state >= GS_FAULT_MIN:
            self.get_logger().warn("VIO fault in VALIDATE - flow hold")
            self._flow_hold_start = None
            self._phase = Phase.FLOW_HOLD
            return
        el = self._secs(self._validate_start)
        if self._init_factor >= self._if_min:
            self._if_low_since = None
            if self._if_good_since is None:
                self._if_good_since = self.get_clock().now()
            if self._secs(self._if_good_since) >= self._validate_hold_s:
                self._handover_sent = False
                self._handover_start = None
                self._phase = Phase.HANDOVER
                return
        elif self._if_good_since is not None:
            if self._if_low_since is None:
                self._if_low_since = self.get_clock().now()
            elif self._secs(self._if_low_since) > self._dip_grace_s:
                self._if_good_since = self._if_low_since = None
        self._tele(f"VALIDATE IF={self._init_factor:.2f} "
                   f"(>={self._if_min:.2f})  t={el:.0f}s")
        if el > self._validate_timeout_s:
            self.get_logger().error("Validation timeout - flow hold")
            self._flow_hold_start = None
            self._phase = Phase.FLOW_HOLD

    def _do_handover(self):
        ax, ay = self._validate_anchor
        self._pub_sp(ax, ay, self._takeoff_alt if not self._mission_started
                     else self._cruise_alt)
        if not self._handover_sent:
            r = SetBool.Request(); r.data = True
            self._gate_cli.call_async(r)
            self._handover_sent = True
            self._handover_start = self.get_clock().now()
            return
        if self._vio_state == GS_OPEN:
            if self._secs(self._handover_start) >= self._handover_settle_s:
                if self._mission_started:
                    self.get_logger().info(
                        f"Gate OPEN - resuming {self._resume_phase.name}")
                    self._sp = None
                    self._phase = self._resume_phase or Phase.SURVEY_STRIPE
                else:
                    self._sp_z = self._pose.pose.position.z
                    self._climb_confirm_since = None
                    self.get_logger().info(
                        f"Gate OPEN - climbing to {self._cruise_alt:.1f} m.")
                    self._phase = Phase.CLIMB
        elif self._secs(self._handover_start) > 5.0:
            self.get_logger().warn("Gate did not open - flow hold")
            self._flow_hold_start = None
            self._phase = Phase.FLOW_HOLD

    def _do_climb(self):
        if self._vio_fault():
            return
        ax, ay = self._validate_anchor
        self._sp_z = min(self._cruise_alt,
                         self._sp_z + self._climb_speed / self._sp_rate_hz)
        self._pub_sp(ax, ay, self._sp_z)
        alt = self._pose.pose.position.z
        self._tele(f"CLIMB(VIO) {alt:.2f} -> {self._cruise_alt:.1f} m")
        if abs(alt - self._cruise_alt) <= self._alt_tol:
            if self._climb_confirm_since is None:
                self._climb_confirm_since = self.get_clock().now()
            elif self._secs(self._climb_confirm_since) >= self._at_alt_confirm_s:
                self._acq_start = (self._pose.pose.position.x,
                                   self._pose.pose.position.y)
                self._mission_started = True
                self._reach_since = None
                self.get_logger().info(
                    f"At {alt:.2f} m - flying BACKWARD from centre to find the "
                    "back line (first boundary).")
                self._phase = Phase.ACQ_BACK
        else:
            self._climb_confirm_since = None

    # ══════════════════════════════════════════════════════════════
    #  ACQUISITION: arena centre -> back line -> left line = Corner 1
    # ══════════════════════════════════════════════════════════════
    def _do_acq_back(self):
        """From the arena centre, fly BACKWARD (body -X) until the back line is
        at the standoff. The wall veto stops the drone 0.5 m short — it never
        crosses. When the back line is held, strafe left to find Corner 1."""
        if self._vio_fault():
            return
        if self._boundary_dead():
            return
        dt = 1.0 / self._sp_rate_hz
        if self._sp is None:
            self._sp = [self._pose.pose.position.x, self._pose.pose.position.y]
            if self._acq_start is None:
                self._acq_start = (self._pose.pose.position.x,
                                   self._pose.pose.position.y)
        fx, fy, _, _ = self._dir_vectors()
        lines = self._lines_dir()
        if self._breach_recover(lines, dt):
            self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)
            self._table("ACQ_BACK RECOVER (too close, easing back)",
                        0.0, self._nearest)
            return
        back = self._classify(lines)['back']

        # SAFETY: the drone flies a straight backward line here, so whatever
        # boundary is NEAREST is the one it is driving into — brake on that,
        # frame-independently. 'dist' does not depend on the camera-yaw mount
        # offset (only the normal direction does), so even if classification
        # mislabels the line, the acquire-latch still fires and the drone
        # HOLDS the moment any line is within reach instead of crossing it.
        nearest = min(lines, key=lambda L: L['dist']) if lines else None
        target = back if back is not None else nearest

        speed = self._survey_speed * self._approach(target)
        self._sp[0] -= fx * speed * dt           # backward = -forward
        self._sp[1] -= fy * speed * dt
        self._apply_boundary(lines, dt)
        self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)

        near_back = target['dist'] if target is not None else -1.0
        travelled = -((self._pose.pose.position.x - self._acq_start[0]) * fx
                      + (self._pose.pose.position.y - self._acq_start[1]) * fy)
        self._table(f"ACQ_BACK trav={travelled:.2f}/{self._max_forward:.1f}",
                    speed, near_back)

        # Transition on ACQUIRE, not on a precise stop. Once the back line has
        # been seen within slow_dist, _approach has latched _appr_acquired and
        # is now HOLDING the drone (the faint tape reads too jumpy to nail an
        # exact stop_dist). Settle briefly, then strafe LEFT — ACQ_LEFT keeps
        # the standoff (stop_dist) off the back line the whole way.
        if self._appr_acquired:
            if self._reach_since is None:
                self._reach_since = self.get_clock().now()
            elif self._secs(self._reach_since) >= self._acq_settle_s:
                self.get_logger().info(
                    f"Back line acquired (~{near_back:.2f} m) - strafing LEFT to "
                    "find the left line (Corner 1).")
                self._acq_start = (self._pose.pose.position.x,
                                   self._pose.pose.position.y)
                self._reach_since = None
                self._sp = None
                self._phase = Phase.ACQ_LEFT
                return
        else:
            self._reach_since = None

        if travelled > self._max_forward and not self._appr_acquired:
            self.get_logger().warn(
                f"No back line within {self._max_forward:.1f} m - returning.")
            self._begin_return()

    def _do_acq_left(self):
        """Slide body-LEFT holding stop_dist off the back line until the LEFT
        line appears. That intersection is Corner 1 (back-left)."""
        if self._vio_fault():
            return
        if self._boundary_dead():
            return
        dt = 1.0 / self._sp_rate_hz
        if self._sp is None:
            self._sp = [self._pose.pose.position.x, self._pose.pose.position.y]
            if self._acq_start is None:
                self._acq_start = (self._pose.pose.position.x,
                                   self._pose.pose.position.y)
        fx, fy, lx, ly = self._dir_vectors()
        lines = self._lines_dir()
        if self._breach_recover(lines, dt):
            self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)
            self._table("ACQ_LEFT RECOVER (too close, easing back)",
                        0.0, self._nearest)
            return
        cls = self._classify(lines)
        leftl = cls['left']

        # SAFETY: if the BACK line has disappeared from view for more than
        # acq_left_back_lost_s, we cannot confirm we are still within the
        # standoff band — HOLD instead of cruising blind. Bridges brief
        # detector flickers. Fixes the flight-log failure where the detector
        # lost tracking mid-strafe and the drone continued cruising for
        # 20+ s past the arena until VIO faulted.
        back_lost_dur = 0.0
        if cls['back'] is not None:
            self._acq_left_back_lost_since = None
        else:
            if self._acq_left_back_lost_since is None:
                self._acq_left_back_lost_since = self.get_clock().now()
            back_lost_dur = self._secs(self._acq_left_back_lost_since)
        back_lost_hold = back_lost_dur > self._acq_left_back_lost_s

        speed = self._strafe_speed * self._approach(leftl)
        if back_lost_hold:
            speed = 0.0        # HOLD — back boundary lost too long
        self._sp[0] += lx * speed * dt
        self._sp[1] += ly * speed * dt

        # Hold stop_dist off the BACK line while sliding along it, so the drone
        # tracks the back boundary parallel instead of drifting off it.
        if cls['back'] is not None:
            dbx, dby = self._hold_offset(cls['back'], dt)
            self._sp[0] += dbx
            self._sp[1] += dby

        self._apply_boundary(lines, dt)
        self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)

        near_left = leftl['dist'] if leftl is not None else -1.0
        strafed = ((self._pose.pose.position.x - self._acq_start[0]) * lx
                   + (self._pose.pose.position.y - self._acq_start[1]) * ly)
        state = (f" HOLD(back lost {back_lost_dur:.1f}s)" if back_lost_hold
                 else (f" back-flicker {back_lost_dur:.1f}s"
                       if back_lost_dur > 0 else ""))
        self._table(f"ACQ_LEFT strafe={strafed:.2f}/{self._max_strafe:.1f}"
                    f"{state}", speed, near_left)

        # The SECOND (left) line being detected IS Corner 1 — so transition as
        # soon as _approach has latched it (seen within slow_dist), after a
        # short settle. Same reason as ACQ_BACK: the tape is too jumpy to wait
        # for a clean stop_dist reading.
        if self._appr_acquired:
            if self._reach_since is None:
                self._reach_since = self.get_clock().now()
            elif self._secs(self._reach_since) >= self._acq_settle_s:
                self.get_logger().info(
                    f"Left line acquired (~{near_left:.2f} m) - Corner 1 "
                    "(back-left) found.")
                self._reach_since = None
                self._hold_xy = [self._pose.pose.position.x,
                                 self._pose.pose.position.y]
                self._hover_start = None
                self._sp = None
                self._phase = Phase.CORNER1_HOLD
                return
        else:
            self._reach_since = None

        if strafed > self._max_strafe and not self._appr_acquired:
            self.get_logger().warn(
                f"No left line within {self._max_strafe:.1f} m - returning.")
            self._begin_return()

    def _do_corner1_hold(self):
        """Corner 1 (back-left). Count it, hold position for hover_duration
        (10 s), then launch the lawnmower survey with a forward stripe."""
        if self._vio_fault():
            return
        if self._boundary_dead():
            return
        dt = 1.0 / self._sp_rate_hz
        if self._sp is None:
            self._sp = [self._hold_xy[0], self._hold_xy[1]]
        lines = self._lines_dir()
        # Hold the corner, wall veto stays live so we never drift over a line.
        self._apply_boundary(lines, dt)
        self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)

        if self._corners_found < 1:
            self._count_corner("back + left lines meet")
        if self._hover_start is None:
            self._hover_start = self.get_clock().now()
        el = self._secs(self._hover_start)
        self._table(f"CORNER1 hold {self._hover_duration - el:.0f}s left",
                    0.0, self._nearest)
        if el >= self._hover_duration:
            # Survey starts: first stripe runs FORWARD along the left line.
            self._stripe_dir = 1
            self._stripe_count = 0
            self._corner_counted_end = False
            self._reach_since = None
            self._sp = None
            self.get_logger().info(
                "Corner 1 hold complete - starting lawnmower survey "
                "(forward stripe along the left line).")
            self._phase = Phase.SURVEY_STRIPE

    # ══════════════════════════════════════════════════════════════
    #  LAWNMOWER SURVEY: stripes (fwd/back) stepping 1 m right, counting
    #  every corner where an end line meets a side line, until 4 corners.
    # ══════════════════════════════════════════════════════════════
    def _do_survey_stripe(self):
        """Fly a straight stripe in the current longitudinal direction until
        the end line (front if going forward, back if going backward) is at the
        standoff. At the end, if a perpendicular side line is also near, that is
        a new corner — count it. Then step 1 m right to the next stripe."""
        if self._vio_fault():
            return
        if self._boundary_dead():
            return
        if self._corners_found >= self._target_corners:
            self.get_logger().info(
                f"All {self._target_corners} corners found - returning home.")
            self._begin_return()
            return
        dt = 1.0 / self._sp_rate_hz
        if self._sp is None:
            self._sp = [self._pose.pose.position.x, self._pose.pose.position.y]
            self._acq_start = (self._pose.pose.position.x,
                               self._pose.pose.position.y)
            self._corner_counted_end = False
        fx, fy, _, _ = self._dir_vectors()
        d = self._stripe_dir
        lines = self._lines_dir()
        if self._breach_recover(lines, dt):
            self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)
            self._table("STRIPE RECOVER (too close, easing back)",
                        0.0, self._nearest)
            return
        cls = self._classify(lines)
        end = cls['front'] if d > 0 else cls['back']

        speed = self._survey_speed * self._approach(end)
        self._sp[0] += d * fx * speed * dt
        self._sp[1] += d * fy * speed * dt
        self._apply_boundary(lines, dt)
        self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)

        near_end = end['dist'] if end is not None else -1.0
        travelled = d * (
            (self._pose.pose.position.x - self._acq_start[0]) * fx
            + (self._pose.pose.position.y - self._acq_start[1]) * fy)
        dir_txt = "FWD" if d > 0 else "BACK"
        self._table(
            f"STRIPE#{self._stripe_count} {dir_txt} trav={travelled:.2f} "
            f"corners={self._corners_found}/{self._target_corners}",
            speed, near_end)

        if end is not None and near_end <= self._stop_dist + 0.15:
            if self._reach_since is None:
                self._reach_since = self.get_clock().now()
            elif self._secs(self._reach_since) >= self._reach_confirm_s:
                if not self._corner_counted_end and self._side_line_near(cls):
                    self._count_corner(
                        f"{dir_txt.lower()} line + side line meet")
                    self._corner_counted_end = True
                self._reach_since = None
                if self._corners_found >= self._target_corners:
                    self.get_logger().info(
                        f"All {self._target_corners} corners found - "
                        "returning home.")
                    self._begin_return()
                    return
                self._step_start = (self._pose.pose.position.x,
                                    self._pose.pose.position.y)
                self._sp = None
                self._phase = Phase.SURVEY_STEP
                return
        else:
            self._reach_since = None

        if travelled > self._max_stripe_m and end is None:
            self.get_logger().warn(
                f"Stripe found no end line in {self._max_stripe_m:.1f} m - "
                "returning home.")
            self._begin_return()

    def _do_survey_step(self):
        """Shift 1 m to the RIGHT to the next stripe, holding the end line just
        reached at the standoff. If the RIGHT line is already within reach we
        are at the far edge — the next stripe runs along it and picks up the
        last corner(s). Then reverse the stripe direction."""
        if self._vio_fault():
            return
        if self._boundary_dead():
            return
        dt = 1.0 / self._sp_rate_hz
        if self._sp is None:
            self._sp = [self._pose.pose.position.x, self._pose.pose.position.y]
            if self._step_start is None:
                self._step_start = (self._pose.pose.position.x,
                                    self._pose.pose.position.y)
        fx, fy, lx, ly = self._dir_vectors()
        rx, ry = -lx, -ly                        # RIGHT = -LEFT
        lines = self._lines_dir()
        if self._breach_recover(lines, dt):
            self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)
            self._table("STEP RECOVER (too close, easing back)",
                        0.0, self._nearest)
            return
        cls = self._classify(lines)

        brake = 1.0
        span = max(1e-3, self._slow_dist - self._stop_dist)
        if cls['right'] is not None:
            brake = self._smoothstep(
                (cls['right']['dist'] - self._stop_dist) / span)
        speed = self._strafe_speed * brake
        self._sp[0] += rx * speed * dt
        self._sp[1] += ry * speed * dt

        # Hold the end line (front or back) at the standoff while stepping.
        end = cls['front'] if self._stripe_dir > 0 else cls['back']
        if end is not None:
            dex, dey = self._hold_offset(end, dt)
            self._sp[0] += dex
            self._sp[1] += dey

        self._apply_boundary(lines, dt)
        self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)

        stepped = ((self._pose.pose.position.x - self._step_start[0]) * rx
                   + (self._pose.pose.position.y - self._step_start[1]) * ry)
        near_right = cls['right']['dist'] if cls['right'] is not None else -1.0
        self._table(f"STEP right={stepped:.2f}/{self._stripe_step:.1f} "
                    f"corners={self._corners_found}/{self._target_corners}",
                    speed, near_right)

        blocked = (cls['right'] is not None
                   and near_right <= self._stop_dist + 0.15)
        if stepped >= self._stripe_step or blocked:
            self._stripe_dir *= -1               # next stripe reverses
            self._stripe_count += 1
            self._reach_since = None
            self._sp = None
            if self._stripe_count > self._max_stripes:
                self.get_logger().warn(
                    f"Stripe cap ({self._max_stripes}) hit with "
                    f"{self._corners_found} corners - returning home.")
                self._begin_return()
                return
            self.get_logger().info(
                f"Stepped {stepped:.2f} m right"
                + (" (right line reached)" if blocked else "")
                + f" - stripe #{self._stripe_count} "
                + ("FORWARD." if self._stripe_dir > 0 else "BACKWARD."))
            self._phase = Phase.SURVEY_STRIPE

    # ── RETURN home, then land ────────────────────────────────────
    def _begin_return(self):
        self._ret_sp = [self._pose.pose.position.x,
                        self._pose.pose.position.y]
        self._ret_start = self.get_clock().now()
        self._ret_arrived_since = None
        self._phase = Phase.RETURN

    def _do_return(self):
        if self._vio_state >= GS_FAULT_MIN:
            self.get_logger().warn("VIO fault during return - landing here")
            self._phase = Phase.LAND
            return
        if self._secs(self._ret_start) > self._goto_timeout:
            self.get_logger().warn("Return timeout - landing here")
            self._phase = Phase.LAND
            return
        step = self._forward_speed / self._sp_rate_hz
        dx = self._home_x - self._ret_sp[0]
        dy = self._home_y - self._ret_sp[1]
        d = math.hypot(dx, dy)
        if d > 1e-6:
            f = min(1.0, step / d)
            self._ret_sp[0] += dx * f
            self._ret_sp[1] += dy * f
        self._pub_sp(self._ret_sp[0], self._ret_sp[1], self._cruise_alt)
        err = math.hypot(self._pose.pose.position.x - self._home_x,
                         self._pose.pose.position.y - self._home_y)
        self._tele(f"RETURN dist={err:.2f} m -> home")
        if err <= self._goto_radius:
            if self._ret_arrived_since is None:
                self._ret_arrived_since = self.get_clock().now()
            elif self._secs(self._ret_arrived_since) >= 1.0:
                if self._gate_cli.service_is_ready():
                    r = SetBool.Request(); r.data = False
                    self._gate_cli.call_async(r)
                self._gate_close_sent = True
                self._settle_start = self.get_clock().now()
                self.get_logger().info("Home reached - camera off, flow settle")
                self._phase = Phase.FLOW_SETTLE
        else:
            self._ret_arrived_since = None

    def _do_flow_settle(self):
        self._pub_sp(self._home_x, self._home_y, self._cruise_alt)
        if self._secs(self._settle_start) >= self._flow_settle_s:
            self._desc_z = self._pose.pose.position.z
            self._desc_start = self.get_clock().now()
            self._phase = Phase.DESCEND

    def _do_descend(self):
        self._desc_z = max(0.0, self._desc_z
                           - self._descend_speed / self._sp_rate_hz)
        self._pub_sp(self._home_x, self._home_y, self._desc_z)
        alt = self._pose.pose.position.z
        self._tele(f"DESCEND alt={alt:.2f} -> {self._descend_handoff_alt:.2f} m")
        if alt <= self._descend_handoff_alt:
            self._phase = Phase.LAND
        elif self._secs(self._desc_start) > self._descend_timeout:
            self._phase = Phase.LAND

    # ── FLOW_HOLD: VIO-fault recovery ─────────────────────────────
    def _do_flow_hold(self):
        self._pub_vel_hold()
        if self._flow_hold_start is None:
            self._flow_hold_start = self.get_clock().now()
        if self._secs(self._flow_hold_start) < self._stable_of_secs:
            return
        self._flow_hold_start = None
        self._revalidations = getattr(self, "_revalidations", 0) + 1
        if self._revalidations > self._max_revalidations:
            self.get_logger().error(
                f"Max revalidations ({self._max_revalidations}) - landing")
            self._phase = Phase.LAND
            return
        self.get_logger().warn(
            f"Re-validation {self._revalidations}/{self._max_revalidations} "
            f"- will resume {(self._resume_phase or Phase.SURVEY_STRIPE).name}")
        self._seed_sent = False
        self._phase = Phase.SEED

    # ── LAND / DISARM ─────────────────────────────────────────────
    def _do_land(self):
        if not self._gate_close_sent:
            if self._gate_cli.service_is_ready():
                r = SetBool.Request(); r.data = False
                self._gate_cli.call_async(r)
            self._gate_close_sent = True
            return
        if not self._land_req:
            self._mode("AUTO.LAND")
            self._land_req = True
            self._phase = Phase.DISARM

    def _do_disarm(self):
        if not self._state.armed:
            self.get_logger().info("Disarmed - corner mission complete.")
            self._phase = Phase.DONE

    def _do_safe_manual(self):
        self.get_logger().info("SAFE MANUAL - pilot has control.",
                               throttle_duration_sec=5.0)

    # ── Helpers ───────────────────────────────────────────────────
    @staticmethod
    def _yaw_quat(yaw_rad):
        """Quaternion (about +Z) for a target yaw in the MAVROS/ENU frame."""
        q = Quaternion()
        q.w = math.cos(yaw_rad / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw_rad / 2.0)
        return q

    def _pub_sp(self, x, y, z):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "local_origin"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = \
            float(x), float(y), float(z)
        if self._hold_heading_q is not None and self._cmd_yaw_rad is not None:
            target = math.radians(yaw_deg_from_quaternion(self._hold_heading_q))
            max_step = math.radians(self._yaw_slew_dps) / self._sp_rate_hz
            diff = (target - self._cmd_yaw_rad + math.pi) % (2 * math.pi) \
                - math.pi
            self._cmd_yaw_rad += max(min(diff, max_step), -max_step)
            yaw = self._cmd_yaw_rad
            m.pose.orientation.w = math.cos(yaw / 2.0)
            m.pose.orientation.x = 0.0
            m.pose.orientation.y = 0.0
            m.pose.orientation.z = math.sin(yaw / 2.0)
        else:
            m.pose.orientation = self._pose.pose.orientation
        self._sp_pub.publish(m)

    def _pub_vel_hold(self):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "local_origin"
        self._vel_pub.publish(m)

    def _mode(self, mode):
        if self._mode_cli.service_is_ready():
            r = SetMode.Request()
            r.custom_mode = mode
            self._mode_cli.call_async(r)

    def _arm(self, v):
        if self._arm_cli.service_is_ready():
            r = CommandBool.Request()
            r.value = v
            self._arm_cli.call_async(r)

    def _secs(self, t):
        return 0.0 if t is None else \
            (self.get_clock().now() - t).nanoseconds * 1e-9

    def _ch5(self):
        return int(self._rc[self._rc_ch5_idx]) \
            if len(self._rc) > self._rc_ch5_idx else 1500

    def _tele(self, line):
        now = time.monotonic()
        if now - self._last_print >= 1.0:
            self._last_print = now
            self.get_logger().info(line)

    def _yellow_status(self):
        """Always-on terminal readout of the yellow-line detection, printed
        in EVERY phase so you can watch the yellow coverage % and see exactly
        when a line comes into view. Data comes from yellow_boundary_detector
        (/viman/boundary/coverage_pct, /nearest_m, /lines)."""
        age = self._boundary_age()
        if age > self._boundary_stale_s:
            self.get_logger().warn(
                f"[YELLOW] detector SILENT ({age:.1f}s) — no detection data",
                throttle_duration_sec=5.0)
            return
        # During flight phases the status table (_table) already prints the
        # yellow readout every second — don't double-print it here.
        if time.monotonic() - self._last_print < 1.5:
            return
        # In the passive phases (takeoff / hover / seed / validate) there is no
        # table. Only speak up when a line is actually in view — the endless
        # "no line | 0.00%" lines are noise, so they are dropped.
        n = len(self._lines)
        if not (self._nearest >= 0.0 or n > 0):
            return
        dists = ", ".join(f"{L['dist']:.2f}" for L in self._lines) or "-"
        near_txt = f"{self._nearest:.2f}" if self._nearest >= 0 else "--"
        self.get_logger().info(
            f"[YELLOW] line in view | yellow={self._coverage:.2f}% | "
            f"lines={n} dist=[{dists}] m | nearest={near_txt} m")

    def _table(self, label, speed, near):
        now = time.monotonic()
        if now - self._last_print < 1.0:
            return
        self._last_print = now
        n = len(self._lines)
        near_txt = f"{near:.2f}" if near >= 0 else "--"
        dists = ", ".join(f"{L['dist']:.2f}" for L in self._lines) or "-"
        # One clean status line per phase tick: phase/progress, distance to the
        # target line, commanded speed, every line in view + coverage %, and the
        # VIO health (IF). Replaces the old separate [YELLOW] line during flight.
        self.get_logger().info(
            f"{label}  near={near_txt}m spd={speed:.2f}  "
            f"lines={n} dist=[{dists}] yellow={self._coverage:.1f}%  "
            f"IF={self._init_factor:.2f}")


def main():
    rclpy.init()
    node = BoundaryTestAuto()
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