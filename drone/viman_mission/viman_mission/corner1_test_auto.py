#!/usr/bin/env python3
"""
corner1_test_auto — autonomous yellow-line 4-CORNER mission (mission_luma).
Team Viman Rakshak / IRoC-U 2026.

Trimmed from boundary_test_auto: fly to cruise_alt, return to the arena
CENTRE, seed/validate VIO, then centre -> BACK line -> strafe LEFT ->
Corner 1 (back+left lines meet) -> hold corner1_hold_s -> return HOME + land.
No lawnmower survey. Reuses the same detector topics and VIO gate.

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
  CLIMB       fused-VIO climb from takeoff_alt (2.0 m) to cruise_alt (3.0 m)
  ACQ_BACK    from the centre, fly BACKWARD to the back line, soft-stop 0.5 m
  ACQ_LEFT    slide LEFT along the back line, keep sliding until BOTH the
              back and left lines sit in the 0.5-1.2 m band = Corner 1
  CORNER1_HOLD active in-band 5 s hold -> declare Corner 1 (1/4)
  FOLLOW_LEFT_FWD six-mode controller: fly FORWARD tangent along the LEFT
              line holding its band, bring front+left in-band = Corner 2
              candidate. The survey has NOT started yet.
  CORNER_HOLD reusable active 5 s in-band validator for Corners 2/3/4
              (branch by _pending_corner_number)
  SURVEY_STEP shift stripe_step (2 m) RIGHT holding the current end line
              in-band; far right lines are RIGHT-FAR-IGNORED until the
              0.5-1.8 m right acquisition gate latches, then run the
              six-mode end+right Corner 3 approach
  SURVEY_STRIPE alternate BACK/FORWARD stripes; ignore far right while
              the gate is closed; gate opens mid-stripe -> six-mode
              right-reference follow toward the active end for Corner 3
  FOLLOW_RIGHT_END after Corner 3, six-mode follow the RIGHT boundary
              BACKWARD (Corner 3 at FRONT) or FORWARD (Corner 3 at BACK)
              to the opposite end = Corner 4
  RETURN      crawl back to HOME (arena centre) on fused VIO
  FLOW_SETTLE gate closed, EKF settles on flow at altitude
  DESCEND     OFFBOARD precision descent, X/Y locked on home
  LAND        AUTO.LAND for the final touchdown + disarm
  FLOW_HOLD   VIO-fault recovery: flow hold -> re-seed -> re-validate ->
              resume the phase we were in (up to max_revalidations)
  SAFE_MANUAL CH5 HIGH at any time -> STABILIZED, pilot has control
  DISARM / DONE

CORNER RULE (applies to EVERY corner 1-4): a corner is validated only
once the drone sits inside the 0.5-1.2 m band from BOTH meeting lines at
the same time, held for corner_hold_s (5 s). If one line is in-band but
the other is not, the six-mode range-band controller manoeuvres to bring
BOTH lines into the band BEFORE the 5 s hold starts. Never validate on
one line alone. Each line's band term is zero while it is inside the
band, so there is no single-point gradient that can stall (the old
"drone freezes at the corner" bug).

RIGHT ACQUISITION GATE (Corners 3/4): at cruise altitude the right tape
can be visible from far away. Detection is NOT acquisition — a correctly
classified right line participates in Corner 3 logic only after its
distance holds inside [right_gate_lo_m, right_gate_hi_m] = 0.5-1.8 m for
right_gate_confirm_s. Beyond 1.8 m it is RIGHT-FAR-IGNORED: telemetry
shows it, _apply_boundary()/_breach_recover() still respect it, but it
adds no motion, no braking, no corner timer.

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
  ros2 launch viman_mission corner1.launch.py
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
    ACQ_LEFT      = auto()   # slide left until back+left both in-band
    CORNER1_HOLD  = auto()   # active 5 s in-band hold -> Corner 1
    FOLLOW_LEFT_FWD = auto() # left-line tangent FORWARD -> Corner 2 cand.
    CORNER_HOLD   = auto()   # reusable 5 s validator for Corners 2/3/4
    SURVEY_STRIPE = auto()   # fly a fwd/back stripe to the end line
    SURVEY_STEP   = auto()   # shift 2 m right between stripes
    FOLLOW_RIGHT_END = auto()# after Corner 3: right-line tangent -> C4
    RETURN      = auto()
    FLOW_SETTLE = auto()
    DESCEND     = auto()
    FLOW_HOLD   = auto()
    LAND        = auto()
    DISARM      = auto()
    SAFE_MANUAL = auto()
    DONE        = auto()


class Corner1TestAuto(Node):

    def __init__(self):
        super().__init__("boundary_test_auto")

        self.declare_parameters("", [
            # --- flight profile ---
            ("takeoff_alt",          2.0),
            ("cruise_alt",           3.0),   # fused-VIO corner-search and
                                             # lawnmower altitude (mission_luma).
                                             # Distant tape visibility at 3 m is
                                             # handled by the explicit right /
                                             # partner acquisition gates, not by
                                             # flying lower.
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
            # --- yaw: SURVEY method — hold the ARM-TIME heading (default), or a
            #     fixed compass angle if yaw_use_arm_heading is false ---
            ("yaw_use_arm_heading",  True),  # True = hold whatever heading the
                                             # drone had at ARM (Pixhawk compass),
                                             # exactly like survey_mission. Set
                                             # the yaw by facing the drone before
                                             # arming. False = use mission_yaw_deg.
            ("mission_yaw_deg",      0.0),   # ONLY used if yaw_use_arm_heading is
                                             # false: fixed TARGET heading [deg] in
                                             # frame the pose yaw is read in
                                             # (MAVROS local / ENU). 0 = the
                                             # drone yaws until its own reported
                                             # yaw reads 0 deg. If the Pixhawk /
                                             # QGC compass should instead read 0
                                             # (North), set this to 90. Tune
                                             # here to match the cube.
            ("yaw_slew_dps",         15.0),
            ("yaw_align_tol_deg",    3.0),   # (was 8.0) tighter alignment before
                                             # locking cruise_yaw. Motion is then
                                             # commanded along the drone's ACTUAL
                                             # settled nose direction, so forward /
                                             # backward / left / right all fly
                                             # straight instead of skewing by the
                                             # residual yaw error.
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
            ("breach_floor_m",       0.20),  # IGNORE line readings closer than
                                             # this when deciding "too close": at
                                             # cruise alt a THICK tape band's near
                                             # EDGE reads ~0.05-0.20 m even when
                                             # the drone is ~standoff from it, and
                                             # recovering from that phantom froze
                                             # the drone (never strafed). The hard
                                             # per-line veto still prevents any
                                             # real crossing.
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
            ("stripe_step_m",        2.0),   # right shift between stripes (2 m)
            ("corner_margin_m",      0.35),  # (legacy) a perpendicular side line
                                             # within (stop_dist + this) — kept
                                             # for reference; corner detection now
                                             # uses corner_side_reach_m below.
            ("corner_side_reach_m",  1.5),   # at a stripe end (front/back line at
                                             # standoff), if a SIDE line (left or
                                             # right) is within THIS -> a corner.
                                             # User rule: right line within 1.5 m.
            ("corner_dedup_m",       1.2),   # a new corner must be at least this
                                             # far from every already-counted one
                                             # (else it is the SAME corner again).
            ("target_corners",       4),     # mission ends after this many
            ("max_stripe_m",         12.0),  # a stripe spans the whole arena
                                             # depth — abort only if no end line
                                             # is found in this far (must be >
                                             # arena depth; NOT max_forward=6 m)
            ("max_stripes",          12),    # safety cap on stripe count
            ("acq_settle_s",         1.0),   # settle at a reached line/corner
            ("corner_bridge_s",      2.0),   # during a survey-corner 5 s hold, a
                                             # detector flicker that briefly drops
                                             # the 2nd arm shorter than this is
                                             # BRIDGED (timer keeps running, drone
                                             # holds) instead of resetting — the
                                             # tape flickers 2<->1 lines at a corner
            ("line_bridge_s",        1.0),   # once a line we are flying TOWARD
                                             # was seen within slow_dist, a
                                             # detector dropout shorter than this
                                             # makes the drone HOLD (speed 0)
                                             # instead of charging blind through
                                             # the standoff — the yellow detector
                                             # is intermittent (thin/faint tape).
            # --- hover / return / land ---
            ("hover_duration",       5.0),   # legacy alias for corner1_hold_s
            ("corner1_hold_s",       5.0),   # IN-BAND hold at a corner this long
                                             # (both lines in the 0.5-1.2 m band)
                                             # BEFORE declaring it and moving on.
            ("acq_left_lo_m",        0.5),   # CORNER BAND low  = never closer
            ("acq_left_hi_m",        1.2),   # CORNER BAND high = a line counts as
                                             # "in range/reached" at <= this. Both
                                             # the back and left (and later front)
                                             # lines must sit inside [lo, hi] at
                                             # the same time to validate a corner.
            ("acq_left_back_lost_s", 1.5),   # HOLD the strafe if the back line
                                             # disappears from view for THIS
                                             # many seconds. Without this the
                                             # drone cruised blind for 20 s
                                             # after the detector dropped out
                                             # mid-strafe (flight log), flew
                                             # 3+ m past the arena, VIO faulted.
            ("corner_perp_dot",      0.5),   # |n1·n2| below this = two lines are
                                             # "perpendicular". 0.5 ~ 60–120°;
                                             # 0.34 ~ 70–110° (tighter). Buffer
                                             # so the left line need not be an
                                             # exact 90° to count as the corner.
            ("corner_reach_m",       2.5),   # COMMIT gate: first time the corner
                                             # vertex is seen within this, latch
                                             # onto it and start closing in. Big
                                             # so we commit even if it appears far.
            ("corner_target_m",      1.0),   # after latch, creep toward the vertex
                                             # until it is THIS far, then stop. The
                                             # back line is held ~0.75 m, so a 1.0 m
                                             # vertex ≈ 0.5-1.0 m off BOTH lines.
                                             # Raise to stop farther, lower to sit
                                             # closer to the corner.
            ("corner_confirm_s",     1.0),   # BOTH lines must read in-band this
                                             # long (candidate settle) before the
                                             # full 5 s CORNER_HOLD begins. Does
                                             # NOT count toward the 5 s.
            # --- RIGHT-boundary acquisition gate (Corners 3 & 4) ---
            # At cruise altitude the right tape is visible from far away;
            # detection is NOT acquisition. A correctly classified RIGHT
            # line opens the gate only after its distance holds inside
            # [lo, hi] for confirm_s. Above hi it is RIGHT-FAR-IGNORED
            # (telemetry only; wall veto / breach recovery still apply).
            ("right_gate_lo_m",      0.5),   # below this = breach recovery,
                                             # never corner acquisition
            ("right_gate_hi_m",      1.8),   # acquisition-only far edge; the
                                             # corner band stays 0.5-1.2 m
            ("right_gate_confirm_s", 0.5),   # in-window hold before latch
            # --- survey progress watchdog (finite stuck-sweep recovery) ---
            ("survey_stall_timeout_s",     10.0),  # active motion cmd with no
                                                   # pose progress this long =
                                                   # a stall
            ("survey_stall_min_progress_m", 0.20), # projected progress that
                                                   # resets the window
            ("survey_stall_max_retries",   3),     # bounded recovery cycles
                                                   # without progress, then
                                                   # RETURN (progress resets
                                                   # the count)
            ("stall_recover_timeout_s",    30.0),  # one BAND/DRIVE stuck-
                                                   # recovery may run at most
                                                   # this long -> RETURN
            ("hold_escalate_s",            6.0),   # a reference-lost HOLD
                                                   # longer than this starts
                                                   # the stuck recovery — the
                                                   # mission NEVER freezes
            ("corner_blind_validate_s", 5.0),  # at the vertex the tape often
                                               # vanishes under the drone
                                               # (detection 0, speed 0). If it
                                               # follows a confirmed in-band
                                               # candidate/hold, validate the
                                               # corner after this long and
                                               # PROCEED with the mission.
            ("corner_hold_max_s",       20.0), # absolute cap on a corner-hold
                                               # phase: validate + proceed even
                                               # if jumpy tape keeps the 5 s
                                               # in-band timer from finishing.
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
        self._yaw_use_arm_heading = bool(gp("yaw_use_arm_heading"))
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
        self._breach_floor = float(gp("breach_floor_m"))
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
        self._corner_side_reach = float(gp("corner_side_reach_m"))
        self._corner_dedup_m = float(gp("corner_dedup_m"))
        self._target_corners = int(gp("target_corners"))
        self._max_stripe_m  = float(gp("max_stripe_m"))
        self._max_stripes   = int(gp("max_stripes"))
        self._acq_settle_s  = float(gp("acq_settle_s"))
        self._line_bridge_s = float(gp("line_bridge_s"))
        self._corner_bridge_s = float(gp("corner_bridge_s"))
        self._hover_duration = float(gp("hover_duration"))
        self._corner1_hold_s = float(gp("corner1_hold_s"))
        self._acq_left_lo = float(gp("acq_left_lo_m"))
        self._acq_left_hi = float(gp("acq_left_hi_m"))
        self._acq_left_back_lost_s = float(gp("acq_left_back_lost_s"))
        self._corner_perp = float(gp("corner_perp_dot"))
        self._corner_reach = float(gp("corner_reach_m"))
        self._corner_target = float(gp("corner_target_m"))
        self._corner_confirm_s = float(gp("corner_confirm_s"))
        self._right_gate_lo = float(gp("right_gate_lo_m"))
        self._right_gate_hi = float(gp("right_gate_hi_m"))
        self._right_gate_confirm_s = float(gp("right_gate_confirm_s"))
        self._stall_timeout_s = float(gp("survey_stall_timeout_s"))
        self._stall_min_progress = float(gp("survey_stall_min_progress_m"))
        self._stall_max_retries = int(gp("survey_stall_max_retries"))
        self._stall_recover_timeout_s = float(gp("stall_recover_timeout_s"))
        self._hold_escalate_s = float(gp("hold_escalate_s"))
        self._corner_blind_validate_s = float(gp("corner_blind_validate_s"))
        self._corner_hold_max_s = float(gp("corner_hold_max_s"))
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
        self._corner_positions = []      # world (x, y) of each counted corner
        self._first_stripe = False       # (legacy) stripe 1 hugged the LEFT
                                         # line — superseded by FOLLOW_LEFT_FWD

        # --- CORNER_HOLD pending-corner descriptor (Corners 2/3/4) -----
        # The reusable validator needs to know WHICH pair it is holding,
        # what number to declare, and where to go next.
        self._pending_corner_number = 0   # 2, 3 or 4
        self._pending_line_a = None       # role string: front/back/left/right
        self._pending_line_b = None       # role string (the partner)
        self._pending_corner_reason = ""  # human log text
        self._corner_source_phase = None  # phase to resume if de-duped
        self._after_corner = None         # Phase to enter once counted
        self._corner_hold_flicker_since = None  # <= line_bridge_s bridged
        self._corner_blind_since = None   # detection-0 timer inside a hold
        self._hold_phase_start = None     # absolute corner-hold phase timer
        self._cand_last_at_target_ns = 0  # last BOTH-IN-BAND tick (blind carry)
        self._cand_blind_since = None     # candidate blind-spot carry timer

        # --- Corner 2 dedicated left-line follow (FOLLOW_LEFT_FWD) -----
        self._follow_start = None         # (x, y) at phase entry
        self._follow_left_lost_since = None
        self._corner_candidate_since = None  # BOTH-IN-BAND settle timer

        # --- RIGHT-boundary acquisition gate (Corners 3 & 4) ------------
        self._right_gate_latched = False
        self._right_gate_since = None     # in-window confirmation timer
        self._right_lost_since = None     # right reference flicker/HOLD

        # --- SURVEY_STEP / FOLLOW_RIGHT_END roles -----------------------
        self._step_end_role = "front"     # end line held during the step
        self._corner3_end_role = None     # "front"/"back" — NEVER assumed
        self._right_follow_dir = -1       # -1 back, +1 fwd (set by Corner 3)
        self._right_follow_target = "back"
        self._right_follow_start = None

        # --- survey progress watchdog ------------------------------------
        self._stall_kind = None           # "right" / "forward" / "backward"
        self._stall_vec = None            # locked ENU unit vector
        self._stall_anchor = None         # (x, y) progress anchor
        self._stall_since = None          # window start
        self._stall_retries = 0
        self._stall_settling = False      # zero-velocity settle in progress
        self._stall_settle_start = None
        self._stall_retry_active = False  # retry at corner_speed until proven
        self._stall_recover_mode = None   # None / "band" / "drive"
        self._stall_recover_anchor = None # (x, y) at recovery start
        self._stall_recover_start = None  # recovery wall-clock cap
        self._hold_escalate_since = None  # continuous-HOLD anti-freeze timer
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
        # When the detector shows lines=0 during the strafe we can no longer
        # confirm we are within the standoff band, so we must HOLD instead of
        # cruising blind. A brief dropout (motion blur, momentary jump in the
        # filter) is bridged; sustained loss stops the strafe entirely.
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
        self._corner_hold_since = None       # 5 s in-band hold timer at a
                                             # survey-stripe corner (Corner 2-4),
                                             # mirrors Corner 1's CORNER1_HOLD
        self._corner_lost_since = None       # bridge timer: how long the corner
                                             # arms have been out of view during
                                             # a hold (flicker tolerance)
        self._corner_body = None            # (fwd, left) metres, body-FLU
        self._corner_ns = 0                 # last corner msg time (staleness)
        self._corner_latched = False        # committed to closing in on a corner
        self._corner_latch_d = -1.0         # vertex distance captured at latch
        self._corner_at_target = False      # reached corner_target -> stop & hold
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
            Phase.FOLLOW_LEFT_FWD: self._do_follow_left_fwd,
            Phase.CORNER_HOLD: self._do_corner_hold,
            Phase.SURVEY_STRIPE: self._do_survey_stripe,
            Phase.SURVEY_STEP: self._do_survey_step,
            Phase.FOLLOW_RIGHT_END: self._do_follow_right_end,
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
            f"Corner1TestAuto (4-CORNER mission): fly at {self._cruise_alt:.1f} "
            f"m, corner band {self._acq_left_lo:.1f}-{self._acq_left_hi:.1f} m "
            f"off every yellow line. Home = arena CENTRE. Path: centre -> BACK "
            f"line -> slide LEFT -> Corner 1 (back+left in-band, hold "
            f"{self._corner1_hold_s:.0f} s) -> FOLLOW LEFT line FORWARD -> "
            f"Corner 2 (front+left) -> lawnmower (2 m RIGHT steps, back/fwd "
            f"stripes) with the {self._right_gate_lo:.1f}-"
            f"{self._right_gate_hi:.1f} m RIGHT acquisition gate -> Corner 3 -> "
            "follow RIGHT line to the opposite end -> Corner 4 -> HOME -> land. "
            "SAFETY LATCH: CH5 HIGH once, then LOW to start.")
        self.get_logger().warn(
            ">>> SIX-MODE RANGE-BAND build (mission_luma): every corner needs "
            f"BOTH meeting lines inside {self._acq_left_lo:.1f}-"
            f"{self._acq_left_hi:.1f} m held {self._corner1_hold_s:.0f} s; far "
            "right lines are RIGHT-FAR-IGNORED until the acquisition gate "
            f"latches; survey stall watchdog {self._stall_timeout_s:.0f} s / "
            f"{self._stall_max_retries} retries. WALL-HOLD veto keeps "
            f"{self._stop_dist:.2f} m off every line (never crosses). If you do "
            "NOT see THIS line at startup, the new code is NOT running — "
            "rebuild & re-source.")

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
            self._hold_escalate_since = None   # HOLD timer is per-phase
            self._cand_blind_since = None      # blind-carry is per-phase

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
        rotation.  _dir_vectors() also uses _cruise_yaw, so boundary vectors
        and motion vectors live in the SAME reference frame.  Without this,
        a small live-yaw wobble (a few degrees off the locked heading)
        rotated the boundary vectors relative to the motion axes and the
        drone's forward / backward / left / right runs came out skewed.
        Pre-lock _cruise_yaw is 0.0 (a placeholder) — that's harmless because
        no motion phase consumes boundary vectors before HOVER_HOME locks the
        yaw (same convention _dir_vectors uses)."""
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
        corner_side_reach (1.5 m) — the second arm of a corner. User rule: the
        end line sits at the standoff (0.5-1.0 m) AND a side line is within
        1.5 m -> the two lines meet = a corner."""
        for k in ('left', 'right'):
            if cls[k] is not None and cls[k]['dist'] <= self._corner_side_reach:
                return True
        return False

    # ════════════════════════════════════════════════════════════
    #  LINE-FRAME MOTION HELPERS (yellow-line-area behaviour)
    # ════════════════════════════════════════════════════════════
    # Once a yellow line is visible, motion is defined RELATIVE TO THE LINE
    # (perpendicular = distance control; tangent = slide along it). This
    # frees the drone from small yaw errors: even if the drone is yawed a
    # few degrees off the arena's frame, the line's own detected geometry
    # dictates the direction — the drone follows the tape, not its own
    # body axes.
    #
    # All inputs are ENU (post my earlier fix, _lines_enu uses _cruise_yaw
    # for the body→ENU rotation, so the (ex, ey) unit vectors are stable).
    # (ex, ey) is the unit "push-away" from the line, so -(ex, ey) points
    # TOWARD the line.

    def _line_perp_approach_step(self, line, hold_dist, dt,
                                 speed_cap=None):
        """Move the setpoint STRAIGHT toward `line` (perpendicular approach)
        until the perpendicular distance reaches `hold_dist`. Tapers as we
        approach — no overshoot, no yaw-tilt drift. Used by ACQ_BACK after
        the line is first seen so the drone stops PERPENDICULAR to the wall,
        not at the yaw-offset diagonal from body-back motion."""
        if line is None:
            return 0.0
        err = line['dist'] - hold_dist          # + = still too far, need to close
        cap = speed_cap if speed_cap is not None else self._forward_speed
        speed = max(0.0, min(cap, self._corner_gain * err))
        # Toward line = negative push-away
        self._sp[0] -= line['ex'] * speed * dt
        self._sp[1] -= line['ey'] * speed * dt
        return err

    def _line_follow_step(self, line, hold_dist, tangent_hint_enu, dt,
                          tan_speed=None):
        """Slide along `line` at `hold_dist` perpendicular standoff.

        Motion is decomposed into TANGENT (along line) + PERPENDICULAR
        (toward/away from line to hold hold_dist). Yaw is irrelevant to
        the direction — the tangent is derived from the line's normal.

          tangent_hint_enu = ENU unit vector for the intended direction
                             (e.g. locked-left for ACQ_LEFT strafe). The
                             chosen tangent points the same way as this
                             hint, so "keep moving left" still means
                             "move along the wall in the leftward sense".
          tan_speed        = along-line cruise speed (default strafe_speed).
        """
        if line is None:
            return 0.0
        ex, ey = line['ex'], line['ey']
        # Two tangent options (perpendicular to normal, horizontal)
        tx, ty = -ey, ex
        if tx * tangent_hint_enu[0] + ty * tangent_hint_enu[1] < 0.0:
            tx, ty = -tx, -ty                        # flip to align with hint
        # Perpendicular correction (toward line if too far, away if too close)
        err = line['dist'] - hold_dist
        perp_v = max(-self._corner_speed,
                     min(self._corner_speed, self._corner_gain * err))
        # Tangent cruise speed
        ts = tan_speed if tan_speed is not None else self._strafe_speed
        # Total delta: tangent + toward-line correction
        self._sp[0] += (tx * ts - ex * perp_v) * dt
        self._sp[1] += (ty * ts - ey * perp_v) * dt
        return err

    def _corner_equalize_step(self, line_a, line_b, hold_dist, dt):
        """Move the setpoint toward the point where BOTH lines are exactly
        `hold_dist` away. Gradient descent: each line contributes a pull
        of magnitude (dist - hold_dist) in its own toward-line direction.

        Returns (err_a, err_b): perpendicular errors for each line, so the
        caller can decide when we're 'at' the equal-distance target."""
        if line_a is None or line_b is None:
            return None, None
        err_a = line_a['dist'] - hold_dist
        err_b = line_b['dist'] - hold_dist
        # Move TOWARD each line by its own error (positive err → close in).
        vx = -line_a['ex'] * err_a - line_b['ex'] * err_b
        vy = -line_a['ey'] * err_a - line_b['ey'] * err_b
        # Scale to corner_gain, cap magnitude at corner_speed
        vx *= self._corner_gain
        vy *= self._corner_gain
        mag = math.hypot(vx, vy)
        if mag > self._corner_speed:
            vx *= self._corner_speed / mag
            vy *= self._corner_speed / mag
        self._sp[0] += vx * dt
        self._sp[1] += vy * dt
        return err_a, err_b

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

    def _count_corner(self, why, pos=None):
        """Register a newly reached arena corner, DEDUPED by world position so
        the same physical corner is never counted twice (e.g. the back-left
        corner seen again on the second, 1 m-right backward stripe). Returns
        True if this was a new corner, False if it was already counted."""
        if pos is None:
            pos = (self._pose.pose.position.x, self._pose.pose.position.y)
        for (cx, cy) in self._corner_positions:
            if math.hypot(pos[0] - cx, pos[1] - cy) < self._corner_dedup_m:
                return False                 # same corner as one already found
        self._corner_positions.append(pos)
        self._corners_found += 1
        self.get_logger().info(
            f"★ CORNER {self._corners_found}/{self._target_corners} found "
            f"({why}) at ({pos[0]:.2f}, {pos[1]:.2f}).")
        return True

    def _pos_near_counted(self, pos, radius):
        """True if pos is within `radius` of any already-counted corner. Used to
        SUPPRESS re-validating a corner we just left — e.g. Corner 1's leftover
        back+left arms as the drone starts the forward run to Corner 2. Without
        this, a 45°-rotated arena makes the hugged left line register in both the
        'front' and 'left' buckets, and the drone re-holds on Corner 1 forever."""
        for (cx, cy) in self._corner_positions:
            if math.hypot(pos[0] - cx, pos[1] - cy) < radius:
                return True
        return False

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

        # Ignore ULTRA-CLOSE readings (< breach_floor): at cruise altitude the
        # near EDGE of a thick tape band (or a stray pixel) directly under the
        # drone reads ~0.05-0.20 m while its centre is really ~standoff away.
        # Recovering from that phantom pushed the drone away from a 0.05 m line
        # forever — it sat "easing back" for ~50 s and never strafed. Drop those;
        # the hard per-line veto (_apply_boundary) still blocks any real crossing.
        usable = [L for L in lines if L['dist'] >= self._breach_floor]
        if not usable:
            self._breaching = False
            self._breach_since = None
            return False
        nearest = min(usable, key=lambda L: L['dist'])   # closest REAL line
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

    # ── Corner range-band helpers (Corner 1 logic) ────────────────
    def _line_in_band(self, L):
        """True if line L's perpendicular distance sits inside the corner band
        [acq_left_lo, acq_left_hi] = 0.5–1.2 m. A None line is never in-band.
        Bounds are inclusive; the low edge equals stop_dist (the hard wall)."""
        return (L is not None
                and self._acq_left_lo <= L['dist'] <= self._acq_left_hi)

    def _band_correction(self, L, dt):
        """Two-sided DEAD-BAND perpendicular correction that eases line L into
        the [acq_left_lo, acq_left_hi] band and then GOES QUIET inside it.

        ex,ey point AWAY from the line (toward the drone), so:
          dist > band_hi (too far)   -> move TOWARD the line  (-ex,-ey)
          dist < band_lo (too close) -> move AWAY  from it    (+ex,+ey)
          band_lo <= dist <= band_hi -> no correction (0,0)   <- the WIDE dead
                                                                 zone that kills
                                                                 the old single-
                                                                 point stall.
        Speed is proportional to how far OUTSIDE the band, capped at
        corner_speed, so it eases in smoothly with no overshoot. Returns the
        (dx, dy) ENU setpoint delta for this tick. NOTE: when a line is too far,
        'toward the line' is also the direction that carries the drone along the
        corner (e.g. toward the LEFT line = leftward), so this doubles as the
        approach drive — no separate tangent term is needed while closing in."""
        if L is None:
            return 0.0, 0.0
        if L['dist'] > self._acq_left_hi:            # too far -> ease TOWARD
            v = min(self._corner_speed,
                    self._corner_gain * (L['dist'] - self._acq_left_hi))
            return -L['ex'] * v * dt, -L['ey'] * v * dt
        if L['dist'] < self._acq_left_lo:            # too close -> ease AWAY
            v = min(self._corner_speed,
                    self._corner_gain * (self._acq_left_lo - L['dist']))
            return L['ex'] * v * dt, L['ey'] * v * dt
        return 0.0, 0.0                              # in band -> hold steady

    def _perp_partner(self, back, lines):
        """The nearest OTHER line whose normal is ~perpendicular to `back`'s
        normal (|n1·n2| < corner_perp_dot) — i.e. the corner arm (left line)."""
        partner = None
        if back is not None:
            for L in lines:
                if L is back:
                    continue
                perp = abs(L['ex'] * back['ex'] + L['ey'] * back['ey'])
                if perp < self._corner_perp:     # ~perpendicular -> corner arm
                    if partner is None or L['dist'] < partner['dist']:
                        partner = L
        return partner

    def _pair_perp(self, a, b):
        """True if lines a and b meet the perpendicular-pair test
        (|n1·n2| < corner_perp_dot) — i.e. they can form a corner."""
        if a is None or b is None or a is b:
            return False
        return abs(a['ex'] * b['ex'] + a['ey'] * b['ey']) < self._corner_perp

    def _pick_back(self, cls, lines):
        """The BACK reference for the Corner-1 path. Prefer the classified
        back line; if the classifier flickers, fall back to the most
        BACK-ALIGNED visible line (push-away normal along body-forward) —
        NEVER a side line, so the reference role cannot swap to the left
        arm when it gets closer than the back arm near the corner."""
        back = cls['back']
        if back is None and lines:
            cand = min(lines, key=lambda L: L['fwd'])
            if cand['fwd'] < 0.0:
                back = cand
        return back

    def _pick_left_partner(self, cls, back, lines):
        """The LEFT corner arm for Corner 1 (back-LEFT by definition): the
        classified left line when it is perpendicular to back; otherwise
        the NEAREST unclassified perpendicular arm — the right line is
        excluded from the search entirely (never mistaken for the left
        arm, even when it happens to read nearer)."""
        partner = cls['left']
        if partner is not None and not self._pair_perp(back, partner):
            partner = None
        if partner is None:
            for L in lines:
                if L is back or L is cls['right']:
                    continue
                if not self._pair_perp(back, L):
                    continue
                if partner is None or L['dist'] < partner['dist']:
                    partner = L
        return partner

    def _sp_add_capped(self, dx, dy, dt):
        """Add a combined band-correction delta to the setpoint, capping the
        equivalent velocity magnitude at corner_speed_ms (the two band terms
        may sum above the cap when both lines are outside the band)."""
        cap = self._corner_speed * dt
        mag = math.hypot(dx, dy)
        if mag > cap and mag > 1e-9:
            dx *= cap / mag
            dy *= cap / mag
        self._sp[0] += dx
        self._sp[1] += dy

    @staticmethod
    def _mode_label(mode, ref_name, partner_name, slide_txt):
        """Map the generic six-mode name onto the phase-specific telemetry
        label (e.g. APPROACH-PARTNER-INTO-BAND → APPROACH-RIGHT-INTO-BAND)."""
        if mode == "SLIDE":
            return slide_txt
        if mode == "HOLD":
            return f"HOLD-{ref_name.upper()}-LOST"
        if mode.startswith("REF-FLICKER"):
            return f"{ref_name.upper()}-FLICKER" + mode[len("REF-FLICKER"):]
        if mode == "APPROACH-PARTNER-INTO-BAND":
            return f"APPROACH-{partner_name.upper()}-INTO-BAND"
        if mode == "APPROACH-REF-INTO-BAND":
            return f"APPROACH-{ref_name.upper()}-INTO-BAND"
        return mode

    # ════════════════════════════════════════════════════════════
    #  SIX-MODE RANGE-BAND CORNER CONTROLLER (mission_luma.md)
    # ════════════════════════════════════════════════════════════
    # One primitive drives the Corner 2, Corner 3 and Corner 4
    # approaches — only the role mapping changes:
    #   Corner 2: ref=left,  partner=front, tangent=locked FORWARD
    #   Corner 3 (step):   ref=end,   partner=right, tangent=locked RIGHT
    #   Corner 3 (stripe): ref=right, partner=end,   tangent=stripe dir
    #   Corner 4: ref=right, partner=opposite end, tangent=follow dir
    # Mutually exclusive mode order:
    #   HOLD > REF-FLICKER > BOTH-IN-BAND > APPROACH-PARTNER >
    #   APPROACH-REFERENCE > SLIDE (+ hold reference band)
    # Every band term is ZERO while its line is inside 0.5-1.2 m, so
    # there is no fixed-point gradient that can stall.

    def _six_mode_step(self, ref, partner, partner_usable, tangent_enu,
                       tangent_speed, ref_lost_since, dt):
        """Run one tick of the shared controller, mutating self._sp.

        ref            reference line dict (or None if not visible)
        partner        partner line dict (or None)
        partner_usable partner passed its phase-specific gate (right gate
                       latched / correct role + perp + corner_side_reach)
                       — 'usable' NEVER means merely 'visible'
        tangent_enu    (tx, ty) desired ENU travel sense for SLIDE
        tangent_speed  SLIDE cruise speed [m/s]
        ref_lost_since rclpy Time when ref first went missing (caller-owned)

        Returns (mode, at_target, ref_lost_since, sliding):
          mode      telemetry string
          at_target True only in BOTH-IN-BAND (feeds the candidate timer)
          sliding   True while a genuine tangent motion command is active
                    (feeds the survey progress watchdog)."""
        at_target = False
        sliding = False

        # Reference visibility / flicker / HOLD bookkeeping
        if ref is not None:
            ref_lost_since = None
        elif ref_lost_since is None:
            ref_lost_since = self.get_clock().now()
        lost_dur = self._secs(ref_lost_since) if ref_lost_since else 0.0
        ref_lost_hold = ref is None and lost_dur > self._line_bridge_s

        if ref_lost_hold:
            # 1. HOLD — never continue tangent travel toward / along a
            #    boundary that can no longer be measured.
            mode = "HOLD"
        elif ref is None:
            # 6. REF-FLICKER — freeze the advancing setpoint during the
            #    grace window; resume if the line returns.
            mode = f"REF-FLICKER {lost_dur:.1f}/{self._line_bridge_s:.1f}s"
        else:
            ref_in = self._line_in_band(ref)
            p_in = (partner_usable and partner is not None
                    and self._line_in_band(partner))
            if ref_in and p_in:
                # 5. BOTH-IN-BAND — stop tangent travel, gentle two-sided
                #    band control keeps both lines nudged into the band.
                dxa, dya = self._band_correction(ref, dt)
                dxb, dyb = self._band_correction(partner, dt)
                self._sp_add_capped(dxa + dxb, dya + dyb, dt)
                at_target = True
                mode = "BOTH-IN-BAND"
            elif partner_usable and partner is not None \
                    and ref_in and not p_in:
                # 3. APPROACH-PARTNER-INTO-BAND — reference is good; ease
                #    the partner into 0.5-1.2 m while protecting the ref.
                dxa, dya = self._band_correction(partner, dt)
                dxb, dyb = self._band_correction(ref, dt)
                self._sp_add_capped(dxa + dxb, dya + dyb, dt)
                mode = "APPROACH-PARTNER-INTO-BAND"
            elif partner_usable and partner is not None \
                    and p_in and not ref_in:
                # 4. APPROACH-REFERENCE-INTO-BAND — partner is good; pull
                #    the reference back into its band.
                dxa, dya = self._band_correction(ref, dt)
                dxb, dyb = self._band_correction(partner, dt)
                self._sp_add_capped(dxa + dxb, dya + dyb, dt)
                mode = "APPROACH-REF-INTO-BAND"
            else:
                # 2. SLIDE along the reference tangent + hold its band.
                #    Stays active while the partner is far / unusable and
                #    while BOTH lines are outside the band.
                ex, ey = ref['ex'], ref['ey']
                tx, ty = -ey, ex
                if tx * tangent_enu[0] + ty * tangent_enu[1] < 0.0:
                    tx, ty = -tx, -ty
                dxb, dyb = self._band_correction(ref, dt)
                self._sp[0] += tx * tangent_speed * dt + dxb
                self._sp[1] += ty * tangent_speed * dt + dyb
                sliding = True
                mode = "SLIDE"
        return mode, at_target, ref_lost_since, sliding

    def _corner_candidate(self, at_target):
        """Candidate settle timer shared by all six-mode phases: BOTH lines
        must read in-band for corner_confirm_s CONSECUTIVE seconds before
        the 5 s CORNER_HOLD begins. Any non-BOTH mode resets it. Returns
        True when the candidate is confirmed. Every BOTH-IN-BAND tick is
        also time-stamped for the corner blind-spot carry
        (_candidate_blind)."""
        if not at_target:
            self._corner_candidate_since = None
            return False
        self._cand_last_at_target_ns = self.get_clock().now().nanoseconds
        if self._corner_candidate_since is None:
            self._corner_candidate_since = self.get_clock().now()
            return False
        return (self._secs(self._corner_candidate_since)
                >= self._corner_confirm_s)

    def _candidate_blind(self, lines):
        """CORNER VERTEX BLIND SPOT: right at a corner the tape often sits
        straight under the drone and detection drops to 0 while the drone
        stands still — a flight validated nothing and froze there. If
        BOTH-IN-BAND was seen within the last 1.5 s and ALL detection then
        vanished, carry the candidate through the blind spot: after
        corner_blind_validate_s of continuous blindness the corner is
        treated as reached anyway and the mission PROCEEDS to its hold /
        next step. Returns True on the tick it fires."""
        if lines:
            self._cand_blind_since = None
            return False
        if self._cand_blind_since is None:
            recent = (self.get_clock().now().nanoseconds
                      - self._cand_last_at_target_ns) / 1e9
            if self._cand_last_at_target_ns > 0 and recent <= 1.5:
                self._cand_blind_since = self.get_clock().now()
            return False
        if self._secs(self._cand_blind_since) \
                >= self._corner_blind_validate_s:
            self._cand_blind_since = None
            return True
        return False

    def _enter_corner_hold(self, number, line_a, line_b, source, after):
        """Set up the pending-corner descriptor and enter CORNER_HOLD."""
        self._pending_corner_number = number
        self._pending_line_a = line_a
        self._pending_line_b = line_b
        self._pending_corner_reason = f"{line_a} + {line_b} lines in band"
        self._corner_source_phase = source
        self._after_corner = after
        self._corner_hold_since = None
        self._corner_hold_flicker_since = None
        self._corner_candidate_since = None
        self._corner_at_target = False
        self._sp = None                     # re-init at the live pose
        self._phase = Phase.CORNER_HOLD

    # ── RIGHT-boundary acquisition gate (Corners 3 & 4) ───────────
    def _update_right_gate(self, right):
        """Pre-latch confirmation of the RIGHT boundary. `right` must be
        the line CLASSIFIED against locked mission RIGHT (never an
        arbitrary nearest line). Latch after right_gate_confirm_s inside
        [right_gate_lo, right_gate_hi]; once latched, a noisy sample above
        the window does NOT close it (flicker/HOLD rules take over).
        A reading below right_gate_lo is a breach-recovery matter, not an
        acquisition sample."""
        if self._right_gate_latched:
            return
        in_window = (right is not None
                     and self._right_gate_lo <= right['dist']
                     <= self._right_gate_hi)
        if in_window:
            if self._right_gate_since is None:
                self._right_gate_since = self.get_clock().now()
            elif (self._secs(self._right_gate_since)
                  >= self._right_gate_confirm_s):
                self._right_gate_latched = True
                self.get_logger().info(
                    f"RIGHT gate OPEN at {right['dist']:.2f}m "
                    f"(window {self._right_gate_lo:.1f}-"
                    f"{self._right_gate_hi:.1f}m held "
                    f"{self._right_gate_confirm_s:.1f}s) - six-mode "
                    "Corner 3 approach enabled.")
        else:
            self._right_gate_since = None

    def _right_gate_confirming(self):
        """True while an in-window right sample is being confirmed
        (pre-latch). SURVEY_STRIPE holds the end during this short window
        instead of racing into SURVEY_STEP."""
        return (not self._right_gate_latched
                and self._right_gate_since is not None)

    def _reset_right_gate(self):
        self._right_gate_latched = False
        self._right_gate_since = None
        self._right_lost_since = None

    # ════════════════════════════════════════════════════════════
    #  SURVEY PROGRESS WATCHDOG (finite recovery from a stuck sweep)
    # ════════════════════════════════════════════════════════════
    # Runs ONLY during genuine motion commands (open cruise / SLIDE) in
    # SURVEY_STEP, SURVEY_STRIPE and FOLLOW_RIGHT_END. Gate confirmation,
    # flicker, HOLD, corner alignment, breach recovery, detector-stale
    # hold and approach taper are intentional non-progress states and
    # PAUSE the window instead of counting toward a stall.

    def _watchdog_reset(self):
        """Full reset — call when leaving the survey or on a new segment."""
        self._stall_kind = None
        self._stall_vec = None
        self._stall_anchor = None
        self._stall_since = None
        self._stall_retries = 0
        self._stall_settling = False
        self._stall_settle_start = None
        self._stall_retry_active = False
        self._stall_recover_mode = None
        self._stall_recover_anchor = None
        self._stall_recover_start = None
        self._hold_escalate_since = None

    def _watchdog_pause(self):
        """Intentional non-progress tick (hold/taper/align/veto): keep the
        remembered direction but restart the timing window from here."""
        self._stall_anchor = (self._pose.pose.position.x,
                              self._pose.pose.position.y)
        self._stall_since = self.get_clock().now()

    def _watchdog_recovering(self):
        """True while the stuck recovery owns the drone (the caller must
        return immediately). Full pipeline — THE MISSION NEVER FREEZES:

          SETTLE    zero velocity for recover_settle_s after a declared
                    stall (guards re-check health every tick because the
                    phase runs them BEFORE this call).
          CLASSIFY  the live detector picture once settled:
            * yellow line(s) VISIBLE -> BAND recovery: ease the nearest
              line into the 0.5-1.2 m corner band, then release — the
              phase's own logic proceeds the mission (a corner is just
              two lines: the band hold + wall veto handle both arms, and
              the corner logic continues after release).
            * NO yellow visible -> DRIVE recovery: remember what the
              drone was doing before it froze and CONTINUE that motion
              (forward stays forward, backward stays backward) at
              corner_speed until a yellow line comes into view, then
              switch to BAND recovery, then release.
          LIMITS    _breach_recover / _apply_boundary stay authoritative
                    inside every recovery tick; a recovery is bounded by
                    stall_recover_timeout_s and max_stripe_m of blind
                    travel; repeated stalls without progress are bounded
                    by survey_stall_max_retries. Every limit ends in
                    RETURN home + land — the drone always either resumes
                    the mission or completes it at home; it never sits
                    still, and it NEVER drives blind toward the latched
                    right boundary."""
        dt = 1.0 / self._sp_rate_hz
        px, py = self._pose.pose.position.x, self._pose.pose.position.y

        # ── active BAND / DRIVE recovery ──────────────────────────
        if self._stall_recover_mode is not None:
            if self._secs(self._stall_recover_start) \
                    > self._stall_recover_timeout_s:
                self.get_logger().warn(
                    f"STUCK-RECOVERY timeout "
                    f"({self._stall_recover_timeout_s:.0f}s) - "
                    "returning home.")
                self._begin_return()
                return True
            if self._sp is None:
                self._sp = [px, py]
            lines = self._lines_dir()
            if self._breach_recover(lines, dt):
                self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)
                return True

            if self._stall_recover_mode == "drive":
                if lines:
                    self.get_logger().info(
                        "STUCK-RECOVERY: yellow line found - easing into "
                        f"the {self._acq_left_lo:.1f}-"
                        f"{self._acq_left_hi:.1f} m band, then resuming "
                        f"{self._phase.name}.")
                    self._stall_recover_mode = "band"
                else:
                    vx, vy = self._stall_vec
                    self._sp[0] += vx * self._corner_speed * dt
                    self._sp[1] += vy * self._corner_speed * dt
                    self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)
                    trav = ((px - self._stall_recover_anchor[0]) * vx
                            + (py - self._stall_recover_anchor[1]) * vy)
                    self._table(
                        f"STUCK-RECOVERY[DRIVE-"
                        f"{(self._stall_kind or 'last').upper()}] "
                        f"trav={trav:.2f}/{self._max_stripe_m:.1f} - "
                        "continuing last motion until yellow is found",
                        self._corner_speed, self._nearest)
                    if abs(trav) > self._max_stripe_m:
                        self.get_logger().warn(
                            "STUCK-RECOVERY drive found no yellow within "
                            f"{self._max_stripe_m:.1f} m - returning home.")
                        self._begin_return()
                    return True

            if self._stall_recover_mode == "band":
                if not lines:
                    # line vanished mid-recovery: fall back to DRIVE along
                    # the remembered direction — but NEVER drive blind
                    # toward the latched right boundary; hold instead
                    # (the recovery timeout still guarantees an exit).
                    if self._stall_vec is not None \
                            and self._stall_kind != "right":
                        self._stall_recover_mode = "drive"
                    self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)
                    return True
                nearest = min(lines, key=lambda L: L['dist'])
                if self._line_in_band(nearest):
                    self.get_logger().info(
                        "STUCK-RECOVERY complete: line at "
                        f"{nearest['dist']:.2f} m (in band) - resuming "
                        f"{self._phase.name}.")
                    self._stall_recover_mode = None
                    self._stall_recover_start = None
                    self._stall_recover_anchor = None
                    self._sp = [px, py]
                    self._stall_retry_active = True
                    self._stall_anchor = None
                    self._stall_since = None
                    return True
                dx, dy = self._band_correction(nearest, dt)
                self._sp[0] += dx
                self._sp[1] += dy
                self._apply_boundary(lines, dt)
                self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)
                self._table(
                    f"STUCK-RECOVERY[BAND] nearest={nearest['dist']:.2f}m "
                    f"-> {self._acq_left_lo:.1f}-{self._acq_left_hi:.1f}m",
                    0.0, nearest['dist'])
                return True
            return True

        # ── zero-velocity settle after a declared stall ───────────
        if not self._stall_settling:
            return False
        if self._stall_settle_start is None:
            self._stall_settle_start = self.get_clock().now()
        if self._secs(self._stall_settle_start) < self._recover_settle_s:
            self._pub_vel_hold()
            return True
        self._stall_settling = False
        self._stall_settle_start = None
        self._stall_retries += 1
        if self._stall_retries > self._stall_max_retries:
            self.get_logger().warn(
                f"SURVEY STALL unresolved after {self._stall_max_retries} "
                "recovery cycles - returning home.")
            self._begin_return()
            return True

        # ── classify the stuck situation and pick the recovery ────
        lines = self._lines_dir()
        self._sp = [px, py]
        self._stall_recover_anchor = (px, py)
        self._stall_recover_start = self.get_clock().now()
        if lines:
            nearest = min(lines, key=lambda L: L['dist'])
            self._stall_recover_mode = "band"
            self.get_logger().warn(
                f"STUCK-RECOVERY {self._stall_retries}/"
                f"{self._stall_max_retries}: yellow line in view at "
                f"{nearest['dist']:.2f} m - easing into the "
                f"{self._acq_left_lo:.1f}-{self._acq_left_hi:.1f} m band, "
                f"then resuming {self._phase.name}.")
        elif self._stall_vec is not None and self._stall_kind != "right":
            self._stall_recover_mode = "drive"
            self.get_logger().warn(
                f"STUCK-RECOVERY {self._stall_retries}/"
                f"{self._stall_max_retries}: no yellow in view - "
                f"continuing the last motion "
                f"({(self._stall_kind or 'last').upper()}) at "
                f"{self._corner_speed:.2f} m/s until a line is found.")
        else:
            # no safe direction memory (or it pointed at the latched right
            # boundary): plain in-place retry of the phase at low speed.
            self._stall_recover_start = None
            self._stall_recover_anchor = None
            self._stall_retry_active = True
            self._stall_anchor = None
            self._stall_since = None
            self.get_logger().warn(
                f"SURVEY STALL retry {self._stall_retries}/"
                f"{self._stall_max_retries}: resuming the phase from the "
                f"live pose at {self._corner_speed:.2f}m/s.")
        return True

    def _watchdog_track(self, kind, vec):
        """Genuine motion command active this tick: measure ACTUAL pose
        progress along the remembered direction; declare a stall after
        survey_stall_timeout_s below survey_stall_min_progress_m."""
        now = self.get_clock().now()
        px, py = self._pose.pose.position.x, self._pose.pose.position.y
        if kind != self._stall_kind:
            # new movement segment / direction change → fresh window
            self._stall_kind = kind
            self._stall_vec = vec
            self._stall_anchor = (px, py)
            self._stall_since = now
            self._stall_retries = 0
            self._stall_retry_active = False
            return
        self._stall_vec = vec
        if self._stall_anchor is None or self._stall_since is None:
            self._stall_anchor = (px, py)
            self._stall_since = now
            return
        progress = ((px - self._stall_anchor[0]) * vec[0]
                    + (py - self._stall_anchor[1]) * vec[1])
        if progress >= self._stall_min_progress:
            self._stall_anchor = (px, py)
            self._stall_since = now
            if self._stall_retry_active:
                self._stall_retry_active = False
                self._stall_retries = 0
                self.get_logger().info(
                    "SURVEY progress restored - normal speed resumes.")
            return
        if self._secs(self._stall_since) >= self._stall_timeout_s:
            self.get_logger().warn(
                f"SURVEY STALL: {kind.upper()} commanded, "
                f"progress={max(0.0, progress):.2f}m/"
                f"{self._stall_min_progress:.2f}m in "
                f"{self._stall_timeout_s:.1f}s - hold + health check.")
            # reset the runaway setpoint so it can't cause a later lunge
            self._sp = [px, py]
            self._stall_settling = True
            self._stall_settle_start = None

    def _watchdog_speed(self, normal_speed):
        """Retry speed rule: min(normal, corner_speed) until progress is
        proven again after a stall retry."""
        if self._stall_retry_active:
            return min(normal_speed, self._corner_speed)
        return normal_speed

    def _hold_escalate(self, holding, kind_hint, vec_hint):
        """ANTI-FREEZE for reference-lost HOLD states. Holding position
        after losing the line that defines the track is correct for a
        short while — but the mission must NEVER freeze. After
        hold_escalate_s of continuous HOLD, start the stuck recovery
        (settle -> BAND if yellow is visible / DRIVE the phase's own
        travel direction if not). Returns True on the tick it fires;
        the caller publishes a hold and returns."""
        if not holding:
            self._hold_escalate_since = None
            return False
        if self._hold_escalate_since is None:
            self._hold_escalate_since = self.get_clock().now()
            return False
        if self._secs(self._hold_escalate_since) < self._hold_escalate_s:
            return False
        self._hold_escalate_since = None
        if self._stall_vec is None or self._stall_kind != kind_hint:
            self._stall_kind = kind_hint
            self._stall_vec = vec_hint
        self.get_logger().warn(
            f"HOLD in {self._phase.name} exceeded "
            f"{self._hold_escalate_s:.0f}s - starting stuck recovery "
            "(the mission must not freeze).")
        self._sp = [self._pose.pose.position.x, self._pose.pose.position.y]
        self._stall_settling = True
        self._stall_settle_start = None
        return True

    def _vio_fault(self):
        """If a VIO fault is up, bank the current phase and drop to FLOW_HOLD.
        Returns True when handled (caller should return).

        Recovery cache rules (mission_luma.md, FLOW_HOLD):
          * ALL cached line/candidate state is invalidated — a corner hold
            or candidate timer never resumes with pre-fault time on it.
          * Banked phase is SURVEY_STEP / SURVEY_STRIPE before Corner 3 →
            clear the right-gate latch; after revalidation the live right
            line must pass the complete 0.5-1.8 m gate again.
          * Banked phase is FOLLOW_RIGHT_END (Corner 3 already counted) →
            keep _corner3_end_role / travel direction, but clear the
            right-line flicker cache so it HOLDs until a fresh classified
            right reference returns."""
        if self._vio_state >= GS_FAULT_MIN:
            self.get_logger().error(
                f"VIO fault (gate {self._vio_state}) in {self._phase.name} "
                "- flow hold + revalidate")
            self._resume_phase = self._phase
            # never resume from stale high-altitude line measurements
            self._corner_candidate_since = None
            self._corner_at_target = False
            self._corner_hold_since = None       # pre-fault hold time never
            self._corner_hold_flicker_since = None   # counts after reseeding
            self._corner_blind_since = None
            self._cand_blind_since = None
            self._cand_last_at_target_ns = 0
            self._follow_left_lost_since = None
            self._right_lost_since = None
            self._acq_left_back_lost_since = None
            self._watchdog_reset()
            if self._phase in (Phase.SURVEY_STEP, Phase.SURVEY_STRIPE) \
                    and self._corner3_end_role is None:
                self._reset_right_gate()
                self.get_logger().warn(
                    "Right-gate latch cleared - the live right line must "
                    "re-pass the full acquisition gate after revalidation.")
            elif self._phase == Phase.CORNER_HOLD \
                    and self._pending_corner_number == 3:
                # pending Corner 3 keeps its expected roles, but the gate
                # acquisition must be re-proven from live data
                self._reset_right_gate()
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
            # SURVEY-STYLE yaw: capture the ARM-TIME heading NOW (whatever way
            # the drone is facing when armed = the mission reference, read from
            # the Pixhawk/EKF compass). Do NOT hold it yet — refs stay None so
            # yaw FLOATS through takeoff and an EKF2 mag reset cannot snap the
            # airframe. It is gently applied at home in HOVER_HOME. (Exactly
            # like survey_mission. See yaw_use_arm_heading.)
            self._arm_heading_q  = self._pose.pose.orientation
            self._hold_heading_q = None
            self._cmd_yaw_rad    = None
            tgt = ("ARM-TIME heading" if self._yaw_use_arm_heading
                   else f"fixed {self._mission_yaw_deg:.1f} deg")
            self.get_logger().info(
                f"Armed. HOME=({self._home_x:.2f},{self._home_y:.2f})  "
                f"ARM-TIME heading = "
                f"{yaw_deg_from_quaternion(self._arm_heading_q):.1f} deg "
                f"-> will hold {tgt} at home (like the survey).")
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
                    # APPLY the survey-style heading: default = the ARM-TIME
                    # heading (Pixhawk compass); or the fixed mission_yaw_deg if
                    # yaw_use_arm_heading is false. Gently slew from the current
                    # heading onto it and hold for the rest of the mission.
                    if self._yaw_use_arm_heading and self._arm_heading_q is not None:
                        self._hold_heading_q = self._arm_heading_q
                    else:
                        self._hold_heading_q = self._yaw_quat(
                            math.radians(self._mission_yaw_deg))
                    self._cruise_yaw = math.radians(
                        yaw_deg_from_quaternion(self._hold_heading_q))
                    self._cmd_yaw_rad = math.radians(
                        yaw_deg_from_quaternion(self._pose.pose.orientation))
                    self._yaw_align_start = self.get_clock().now()
                    self._yaw_aligned_since = None
                    self.get_logger().info(
                        f"At home, settled. Slewing onto heading "
                        f"{math.degrees(self._cruise_yaw):.1f} deg, then holding "
                        "until aligned before seeding.")
            else:
                self._hover_home_since = None
            return

        # ── Stage 2: _pub_sp (above) is slewing yaw toward the target —
        #    wait until the reported yaw actually reaches it (held briefly),
        #    or until the alignment times out. ─────────────────────────
        cur = yaw_deg_from_quaternion(self._pose.pose.orientation)
        tgt = math.degrees(self._cruise_yaw)
        err = abs((cur - tgt + 180.0) % 360.0 - 180.0)
        self._tele(f"HOVER_HOME yaw={cur:.1f} -> {tgt:.1f} deg (err={err:.1f} deg)")
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
            # target). PX4 stops here within yaw_align_tol_deg of the target
            # — usually a couple of degrees off. If we keep the mission
            # frame at the target, every subsequent "forward" step is skewed
            # by that residual error and the drone slides diagonally instead
            # of flying straight ahead. Anchoring the mission frame to the
            # actual heading makes forward = nose-direction exactly, and
            # left / right / backward all fly straight along the drone's
            # own body axes. Also freeze _cmd_yaw_rad and _hold_heading_q to
            # this same value so PX4 holds it for the whole mission — every
            # setpoint from now on commands this exact yaw.
            actual_yaw_rad = math.radians(cur)
            self._cruise_yaw = actual_yaw_rad
            self._cmd_yaw_rad = actual_yaw_rad
            self._hold_heading_q = self._yaw_quat(actual_yaw_rad)
            self.get_logger().info(
                f"Yaw locked at ACTUAL {cur:.1f} deg "
                f"(target was {tgt:.1f}, "
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
                    self._phase = self._resume_phase or Phase.ACQ_BACK
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
        """From the arena centre, fly backward until a yellow line is seen,
        then switch to LINE-FRAME motion (perpendicular approach) so the
        drone settles at exactly stop_dist off the line even if its yaw is
        a few degrees off the arena's frame.

        Two sub-modes, based on whether the line is visible:
          OPEN     — no yellow visible. Body -X cruise (fixed 4-direction),
                     the way the drone searches for the wall.
          FOLLOW   — yellow visible. Motion switches to the line's own
                     frame: -normal (toward line) to close on stop_dist,
                     tangent speed = 0 (pure perpendicular approach).
                     This eliminates the 'stuck diagonally' behaviour the
                     yaw offset used to cause.
        """
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
        if self._watchdog_recovering():
            return

        # CORNER 1 IS BACK-LEFT: reach it by flying BACK first — no matter
        # which yellow line becomes visible first (a front / left / right
        # line is wall-veto'd and shown in telemetry, but NEVER becomes
        # the approach target). Prefer the classified 'back' line; if the
        # classifier flickers, fall back to the most BACK-ALIGNED visible
        # line — never a side line.
        cls = self._classify(lines)
        target = self._pick_back(cls, lines)

        # Register the target with _approach so the acquire-latch fires and
        # we track "seen within slow_dist" for the transition test below.
        _ = self._approach(target)

        # ANTI-FREEZE: committed to a line (acquire latch) but it has been
        # gone a while -> the approach factor is 0 and the drone would sit
        # forever. Escalate into the stuck recovery (drive BACKWARD, this
        # phase's own travel direction, until yellow is found again).
        if self._hold_escalate(self._appr_acquired and target is None,
                               "backward", (-fx, -fy)):
            self._pub_vel_hold()
            return

        mode = "OPEN"
        if target is not None:
            # ── LINE-FOLLOW mode: perpendicular approach in the line's frame.
            #    Yaw offset no longer matters — we push directly against the
            #    line's own normal, so we always end up at stop_dist
            #    perpendicular from the wall.
            self._line_perp_approach_step(target, self._stop_dist, dt,
                                          speed_cap=self._survey_speed)
            mode = "FOLLOW-PERP"
        else:
            # ── OPEN mode: fixed 4-direction backward cruise on locked yaw.
            #    _approach() returns 1.0 pre-acquire so we cruise at full
            #    survey speed until we see something.
            speed = self._survey_speed  # pre-acquire cruise
            self._sp[0] -= fx * speed * dt           # backward = -forward
            self._sp[1] -= fy * speed * dt
            if lines:
                mode = "OPEN SIDE-LINE-IGNORED (back line first)"

        self._apply_boundary(lines, dt)
        self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)

        near_back = target['dist'] if target is not None else -1.0
        travelled = -((self._pose.pose.position.x - self._acq_start[0]) * fx
                      + (self._pose.pose.position.y - self._acq_start[1]) * fy)
        self._table(
            f"ACQ_BACK[{mode}] trav={travelled:.2f}/{self._max_forward:.1f}",
            0.0 if target is None else abs(near_back - self._stop_dist),
            near_back)

        # Transition on ACQUIRE + settle. Line-frame perpendicular approach
        # brings the drone straight to stop_dist regardless of yaw offset,
        # so the settle happens perpendicular to the wall — not diagonal.
        if self._appr_acquired:
            if self._reach_since is None:
                self._reach_since = self.get_clock().now()
            elif self._secs(self._reach_since) >= self._acq_settle_s:
                self.get_logger().info(
                    f"Back line acquired (~{near_back:.2f} m) — entering "
                    "YELLOW-LINE-AREA, following back line LEFT to find "
                    "Corner 1.")
                self._acq_start = (self._pose.pose.position.x,
                                   self._pose.pose.position.y)
                self._reach_since = None
                self._corner_latched = False
                self._corner_latch_d = -1.0
                self._corner_at_target = False
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
        """Slide LEFT along the BACK line, holding the back line inside the
        0.5-1.2 m band, and KEEP sliding until the LEFT line is ALSO inside that
        band. Only when BOTH lines are in-band at the same time do we stop and
        hand off to the 5 s corner hold (CORNER1_HOLD).

        This REPLACES the old exact-point 'equalize to 0.5 m each' scheme, which
        stopped the leftward slide the instant the left line appeared and then
        stalled at a false equilibrium where the two distance gradients
        cancelled before either line reached 0.5 m -> the 'drone freezes at the
        corner and never moves' bug. A wide dead-band REGION (not a single
        point) has no such stall: each line only pushes while it is OUTSIDE the
        band and goes quiet once inside, so there is always a clear region to
        settle in.

        Motion modes (priority order):
          HOLD                     back lost > acq_left_back_lost_s (safety).
          BOTH-IN-BAND             back AND left both in 0.5-1.2 m -> hold both,
                                   this is the corner condition.
          APPROACH-BACK-INTO-BAND  left in-band, back OUT of band -> ease back in
                                   (left range me hai par back nahi).
          APPROACH-LEFT-INTO-BAND  back in-band, left visible but OUT of band ->
                                   ease toward the left line to pull it in-band
                                   (back range me hai par left nahi).
          SLIDE-LEFT               only the back line usable so far -> tangent
                                   slide left while holding the back band.
          back-flicker             back briefly out (< grace) -> keep setpoint.
        """
        if self._vio_fault():
            return
        if self._boundary_dead():
            return
        dt = 1.0 / self._sp_rate_hz
        if self._sp is None:
            self._sp = [self._pose.pose.position.x, self._pose.pose.position.y]
            self._corner_at_target = False
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
        if self._watchdog_recovering():
            return

        # Corner 1 = BACK-LEFT by definition: the reference is the BACK
        # line (back-aligned fallback — never a side line, so the roles
        # cannot swap when the left arm gets closer than the back arm near
        # the corner) and the partner is the LEFT arm (classified left, or
        # an unclassified perpendicular arm — NEVER the right line).
        cls = self._classify(lines)
        back = self._pick_back(cls, lines)
        partner = self._pick_left_partner(cls, back, lines)

        # Back-lost safety timer (unchanged): a sustained loss of the back line
        # means we can no longer confirm the standoff -> HOLD instead of cruising
        # blind; brief flickers are bridged.
        back_visible = back is not None
        back_lost_dur = 0.0
        if back_visible:
            self._acq_left_back_lost_since = None
        else:
            if self._acq_left_back_lost_since is None:
                self._acq_left_back_lost_since = self.get_clock().now()
            back_lost_dur = self._secs(self._acq_left_back_lost_since)
        back_lost_hold = back_lost_dur > self._acq_left_back_lost_s
        # ANTI-FREEZE: a back-lost HOLD is safe but must not last forever —
        # escalate into the stuck recovery (drive LEFT, the strafe the drone
        # was flying, until yellow returns; band-recover if yellow visible).
        if self._hold_escalate(back_lost_hold, "left", (lx, ly)):
            self._pub_vel_hold()
            return

        back_in_band = self._line_in_band(back)
        left_visible = partner is not None
        left_in_band = self._line_in_band(partner)

        # ── Motion by mode ──────────────────────────────────────────────
        self._corner_at_target = False           # edge-triggered each tick
        mode = "OPEN"
        if back_lost_hold:
            mode = f"HOLD (back lost {back_lost_dur:.1f}s)"

        elif back_in_band and left_in_band:
            # CORNER: both lines already inside the band. Stop the slide; gently
            # hold both in-band (each correction is 0 while the line is in-band).
            dxb, dyb = self._band_correction(back, dt)
            dxl, dyl = self._band_correction(partner, dt)
            self._sp[0] += dxb + dxl
            self._sp[1] += dyb + dyl
            self._corner_at_target = True
            mode = "BOTH-IN-BAND"

        elif left_in_band and back_visible and not back_in_band:
            # APPROACH-BACK-INTO-BAND: left already good, back drifted out of the
            # band -> bring the BACK line back into range (keep left in band).
            dxb, dyb = self._band_correction(back, dt)
            dxl, dyl = self._band_correction(partner, dt)
            self._sp[0] += dxb + dxl
            self._sp[1] += dyb + dyl
            mode = "APPROACH-BACK-INTO-BAND"

        elif back_in_band and left_visible and not left_in_band:
            # APPROACH-LEFT-INTO-BAND: back good, left visible but out of band ->
            # ease TOWARD the left line to pull it into range (that push is
            # itself leftward), while a light controller keeps the back in band.
            dxl, dyl = self._band_correction(partner, dt)
            dxb, dyb = self._band_correction(back, dt)
            self._sp[0] += dxl + dxb
            self._sp[1] += dyl + dyb
            mode = "APPROACH-LEFT-INTO-BAND"

        elif back_visible:
            # SLIDE-LEFT: only the back line is usable so far (left not in view).
            # Slide left along the back tangent while holding the back band. Do
            # NOT stop just because a second line becomes visible — only stop
            # (above) once the left line is actually in-band.
            ex, ey = back['ex'], back['ey']
            tx, ty = -ey, ex                     # tangent perpendicular to normal
            if tx * lx + ty * ly < 0.0:          # align with locked-LEFT sense
                tx, ty = -tx, -ty
            self._sp[0] += tx * self._strafe_speed * dt
            self._sp[1] += ty * self._strafe_speed * dt
            dxb, dyb = self._band_correction(back, dt)
            self._sp[0] += dxb
            self._sp[1] += dyb
            mode = "SLIDE-LEFT"

        elif not back_lost_hold:
            mode = f"back-flicker {back_lost_dur:.1f}s"
        # else: HOLD, no setpoint update

        self._apply_boundary(lines, dt)
        self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)

        near_back = back['dist'] if back is not None else -1.0
        near_left = partner['dist'] if partner is not None else -1.0
        strafed = ((self._pose.pose.position.x - self._acq_start[0]) * lx
                   + (self._pose.pose.position.y - self._acq_start[1]) * ly)
        self._table(
            f"ACQ_LEFT[{mode}] strafe={strafed:.2f}/{self._max_strafe:.1f} "
            f"back={near_back:.2f}m left={near_left:.2f}m "
            f"band={self._acq_left_lo:.1f}-{self._acq_left_hi:.1f}m",
            0.0, near_back if partner is None else min(near_back, near_left))

        # Corner 1 candidate confirmed when BOTH lines are in-band for
        # acq_settle_s consecutive seconds -> hand off to the 5 s hold.
        if self._corner_at_target:
            if self._reach_since is None:
                self._reach_since = self.get_clock().now()
            elif self._secs(self._reach_since) >= self._acq_settle_s:
                self.get_logger().info(
                    f"Corner 1 (back-left): both lines in band "
                    f"(back {near_back:.2f} m, left {near_left:.2f} m, band "
                    f"{self._acq_left_lo:.1f}-{self._acq_left_hi:.1f} m) - "
                    f"holding {self._corner1_hold_s:.0f} s to confirm.")
                self._reach_since = None
                self._hold_xy = [self._pose.pose.position.x,
                                 self._pose.pose.position.y]
                self._hover_start = None
                self._sp = None
                self._phase = Phase.CORNER1_HOLD
                return
        else:
            self._reach_since = None

        # CORNER 1 BLIND SPOT: both lines were just in-band, then the
        # detection dropped to 0 (the vertex sits right under the drone).
        # Carry the candidate through the blind spot into the Corner 1
        # hold (which also blind-validates) instead of resetting and
        # sitting at the corner forever.
        if self._corner_at_target:
            self._cand_last_at_target_ns = self.get_clock().now().nanoseconds
        if self._candidate_blind(lines):
            self.get_logger().warn(
                "Corner 1 vertex blind spot (detection 0 right after both "
                "lines were in band) - proceeding to the Corner 1 hold.")
            self._hold_xy = [self._pose.pose.position.x,
                             self._pose.pose.position.y]
            self._hover_start = None
            self._sp = None
            self._phase = Phase.CORNER1_HOLD
            return

        if strafed > self._max_strafe:
            self.get_logger().warn(
                f"No left line in band within {self._max_strafe:.1f} m - "
                "returning.")
            self._begin_return()

    def _do_corner1_hold(self):
        """Corner 1 validator: an ACTIVE 5 s in-band hold. The timer
        accumulates ONLY while BOTH the back and left lines are inside the
        0.5-1.2 m band at the same time (never validate on one line alone):

          * both in-band       -> two-sided band control on both (each term
                                  is zero in-band, so no exact-point hunting)
                                  + the 5 s timer runs;
          * a line OUT of band -> correct it, timer RESETS — validation time
                                  cannot accumulate on one valid line;
          * a line flickers    -> bridge for <= line_bridge_s (hold position,
                                  timer keeps running); longer -> timer resets;
          * nothing visible    -> hold the live setpoint; the wall veto and
                                  boundary-stale handling stay active.

        Declaration happens at the END of the uninterrupted hold, then the
        mission enters the dedicated LEFT-line forward follow (the survey has
        NOT started — Corner 2 has its own path)."""
        if self._vio_fault():
            return
        if self._boundary_dead():
            return
        dt = 1.0 / self._sp_rate_hz
        if self._sp is None:
            self._sp = [self._hold_xy[0], self._hold_xy[1]]
            self._corner_hold_since = None
            self._corner_hold_flicker_since = None
            self._corner_blind_since = None
            self._hold_phase_start = self.get_clock().now()
        lines = self._lines_dir()
        if self._breach_recover(lines, dt):
            self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)
            self._table("CORNER1 RECOVER (too close, easing back)",
                        0.0, self._nearest)
            self._corner_hold_since = None
            return

        cls = self._classify(lines)
        back = self._pick_back(cls, lines)
        partner = self._pick_left_partner(cls, back, lines)

        both_visible = back is not None and partner is not None
        both_in_band = (both_visible and self._line_in_band(back)
                        and self._line_in_band(partner))

        # Keep BOTH lines in-band (corrections are 0 while inside the band),
        # capped at corner_speed so noisy tape cannot start a limit cycle.
        dxb, dyb = self._band_correction(back, dt)
        dxl, dyl = self._band_correction(partner, dt)
        self._sp_add_capped(dxb + dxl, dyb + dyl, dt)

        self._apply_boundary(lines, dt)
        self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)

        # ── in-band-gated hold timer with flicker bridging ──────────
        if both_in_band:
            self._corner_hold_flicker_since = None
            self._corner_blind_since = None
            if self._corner_hold_since is None:
                self._corner_hold_since = self.get_clock().now()
        elif not both_visible:
            # a line dropped out — bridge briefly, then reset the timer
            if self._corner_hold_flicker_since is None:
                self._corner_hold_flicker_since = self.get_clock().now()
            elif (self._secs(self._corner_hold_flicker_since)
                  > self._line_bridge_s):
                self._corner_hold_since = None
            # CORNER BLIND SPOT: the vertex sits under the drone and the
            # tape can vanish ENTIRELY (detection 0, speed 0) — track it.
            if not lines:
                if self._corner_blind_since is None:
                    self._corner_blind_since = self.get_clock().now()
            else:
                self._corner_blind_since = None
        else:
            # visible but OUT of band → correction is running; the 5 s
            # validation time cannot accumulate while only one line is valid
            self._corner_hold_since = None
            self._corner_hold_flicker_since = None
            self._corner_blind_since = None

        el = self._secs(self._corner_hold_since) \
            if self._corner_hold_since else 0.0
        hold_s = self._corner1_hold_s
        near_back = back['dist'] if back is not None else -1.0
        near_left = partner['dist'] if partner is not None else -1.0
        self._table(
            f"CORNER1 hold {el:.0f}/{hold_s:.0f}s  "
            f"back={near_back:.2f}m left={near_left:.2f}m "
            f"band={self._acq_left_lo:.1f}-{self._acq_left_hi:.1f}m",
            0.0, self._nearest)

        blind = (self._corner_blind_since is not None
                 and self._secs(self._corner_blind_since)
                 >= self._corner_blind_validate_s)
        overdue = self._secs(self._hold_phase_start) >= self._corner_hold_max_s
        if blind or overdue:
            self.get_logger().warn(
                ("Corner 1 blind spot: detection 0 at the vertex for "
                 f"{self._corner_blind_validate_s:.0f}s"
                 if blind else
                 f"Corner 1 hold ran {self._corner_hold_max_s:.0f}s without "
                 "an uninterrupted in-band window")
                + " - the in-band candidate was already confirmed, so the "
                "corner is validated NOW and the mission proceeds.")
        if (self._corner_hold_since is not None and el >= hold_s) \
                or blind or overdue:
            # Hold complete -> NOW validate + declare Corner 1.
            if self._count_corner("back + left lines in band"):
                self.get_logger().info(
                    f"{self._corners_found}/{self._target_corners} "
                    "corner detected")
            self.get_logger().info(
                f"Corner 1 hold complete ({hold_s:.0f} s) - following the "
                "LEFT line FORWARD to find Corner 2. Survey has NOT started.")
            self._follow_start = (self._pose.pose.position.x,
                                  self._pose.pose.position.y)
            self._corner_candidate_since = None
            self._follow_left_lost_since = None
            self._corner_hold_since = None
            self._sp = None
            self._reach_since = None
            self._phase = Phase.FOLLOW_LEFT_FWD

    # ══════════════════════════════════════════════════════════════
    #  FOLLOW_LEFT_FWD: hold the LEFT-line band, fly FORWARD to Corner 2
    # ══════════════════════════════════════════════════════════════
    def _do_follow_left_fwd(self):
        """Dedicated line-frame phase (NOT a lawnmower stripe): fly tangent
        to the LEFT boundary in the locked forward direction while holding
        that line inside [band_lo, band_hi]; bring front+left in-band =
        Corner 2 candidate → CORNER_HOLD validates it for 5 s.

        Six-mode mapping: reference=left, partner=front, tangent=locked
        FORWARD at survey_speed. `front` becomes a usable partner ONLY when
        it passes the front classification, the perpendicular-pair test and
        sits within corner_side_reach_m — a raw second contour or a distant
        front line does not stop the tangent travel."""
        if self._vio_fault():
            return
        if self._boundary_dead():
            return
        dt = 1.0 / self._sp_rate_hz
        if self._sp is None:
            self._sp = [self._pose.pose.position.x, self._pose.pose.position.y]
            if self._follow_start is None:
                self._follow_start = (self._pose.pose.position.x,
                                      self._pose.pose.position.y)
            self._corner_candidate_since = None
        fx, fy, lx, ly = self._dir_vectors()
        lines = self._lines_dir()
        if self._breach_recover(lines, dt):
            self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)
            self._table("FOLLOW_LEFT_FWD RECOVER (too close, easing back)",
                        0.0, self._nearest)
            self._corner_candidate_since = None
            return
        if self._watchdog_recovering():
            return

        cls = self._classify(lines)
        left = cls['left']
        front = cls['front']
        # partner_usable NEVER means merely 'visible': correct role AND
        # perpendicular to the reference AND within corner_side_reach_m
        # (in-band ≤ 1.2 m is inside the 1.5 m reach by construction).
        front_usable = (front is not None
                        and self._pair_perp(front, left)
                        and front['dist'] <= self._corner_side_reach)

        mode, at_target, self._follow_left_lost_since, sliding = \
            self._six_mode_step(left, front, front_usable, (fx, fy),
                                self._watchdog_speed(self._survey_speed),
                                self._follow_left_lost_since, dt)
        if sliding:
            self._watchdog_track("forward", (fx, fy))
        else:
            self._watchdog_pause()
        if self._hold_escalate(mode == "HOLD", "forward", (fx, fy)):
            self._pub_vel_hold()
            return

        self._apply_boundary(lines, dt)
        self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)

        near_left = f"{left['dist']:.2f}m" if left is not None else "---"
        near_front = f"{front['dist']:.2f}m" if front is not None else "---"
        travelled = ((self._pose.pose.position.x - self._follow_start[0]) * fx
                     + (self._pose.pose.position.y - self._follow_start[1])
                     * fy)
        label = self._mode_label(mode, "left", "front", "SLIDE-FORWARD")
        if mode == "HOLD":
            label += " - no blind forward motion"
        elif mode == "SLIDE" and front is not None and not front_usable:
            label += " FRONT-FAR-IGNORED"
        self._table(
            f"FOLLOW_LEFT_FWD[{label}] trav={travelled:.2f}/"
            f"{self._max_stripe_m:.1f} left={near_left} front={near_front} "
            f"band={self._acq_left_lo:.1f}-{self._acq_left_hi:.1f}m",
            0.0, left['dist'] if left is not None else -1.0)

        # Corner 2 candidate: BOTH front + left in-band for corner_confirm_s,
        # away from every already-counted corner (Corner 1's leftover arms
        # must not re-trigger).
        here = (self._pose.pose.position.x, self._pose.pose.position.y)
        if at_target and self._pos_near_counted(here, self._corner_dedup_m):
            at_target = False
        if self._corner_candidate(at_target):
            self.get_logger().info(
                f"Corner 2 (front-left): both lines in band "
                f"(front {front['dist']:.2f} m, left {left['dist']:.2f} m, "
                f"band {self._acq_left_lo:.1f}-{self._acq_left_hi:.1f} m) - "
                f"holding {self._corner1_hold_s:.0f} s to confirm.")
            self._enter_corner_hold(2, "front", "left",
                                    Phase.FOLLOW_LEFT_FWD, Phase.SURVEY_STEP)
            return
        if self._candidate_blind(lines):
            self.get_logger().warn(
                "Corner 2 vertex blind spot (detection 0 right after both "
                "lines were in band) - proceeding to the Corner 2 hold.")
            self._enter_corner_hold(2, "front", "left",
                                    Phase.FOLLOW_LEFT_FWD, Phase.SURVEY_STEP)
            return

        if travelled > self._max_stripe_m:
            self.get_logger().warn(
                f"No front-left corner within {self._max_stripe_m:.1f} m - "
                "returning.")
            self._begin_return()

    # ══════════════════════════════════════════════════════════════
    #  CORNER_HOLD: one identical 5 s validator for Corners 2, 3 and 4
    # ══════════════════════════════════════════════════════════════
    def _do_corner_hold(self):
        """Reusable active in-band hold. _pending_line_a/_pending_line_b
        name the expected pair (by locked-frame role), _after_corner names
        the next phase. Every tick: reclassify both roles, verify the
        perpendicular-pair test, band-correct both (capped), wall-veto,
        publish. The hold timer accumulates ONLY while both lines remain in
        [band_lo, band_hi]; a flicker <= line_bridge_s is bridged; leaving
        the band, a longer dropout or a failed pair test RESETS the full
        timer. A de-duplicated candidate resumes _corner_source_phase."""
        if self._vio_fault():
            return
        if self._boundary_dead():
            return
        dt = 1.0 / self._sp_rate_hz
        if self._sp is None:
            self._sp = [self._pose.pose.position.x, self._pose.pose.position.y]
            self._corner_hold_since = None
            self._corner_hold_flicker_since = None
            self._corner_blind_since = None
            self._hold_phase_start = self.get_clock().now()
        lines = self._lines_dir()
        if self._breach_recover(lines, dt):
            self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)
            self._table(
                f"CORNER{self._pending_corner_number} RECOVER "
                "(too close, easing back)", 0.0, self._nearest)
            self._corner_hold_since = None
            return

        cls = self._classify(lines)
        line_a = cls.get(self._pending_line_a)
        line_b = cls.get(self._pending_line_b)

        both_visible = line_a is not None and line_b is not None
        pair_ok = both_visible and self._pair_perp(line_a, line_b)
        both_in_band = (pair_ok and self._line_in_band(line_a)
                        and self._line_in_band(line_b))

        dxa, dya = self._band_correction(line_a, dt)
        dxb, dyb = self._band_correction(line_b, dt)
        self._sp_add_capped(dxa + dxb, dya + dyb, dt)

        self._apply_boundary(lines, dt)
        self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)

        if both_in_band:
            self._corner_hold_flicker_since = None
            self._corner_blind_since = None
            if self._corner_hold_since is None:
                self._corner_hold_since = self.get_clock().now()
        elif not both_visible:
            if self._corner_hold_flicker_since is None:
                self._corner_hold_flicker_since = self.get_clock().now()
            elif (self._secs(self._corner_hold_flicker_since)
                  > self._line_bridge_s):
                self._corner_hold_since = None
            # CORNER BLIND SPOT: the vertex sits under the drone and the
            # tape can vanish ENTIRELY (detection 0, speed 0) — track it.
            if not lines:
                if self._corner_blind_since is None:
                    self._corner_blind_since = self.get_clock().now()
            else:
                self._corner_blind_since = None
        else:
            # out of band, or the pair test failed → full timer reset
            self._corner_hold_since = None
            self._corner_hold_flicker_since = None
            self._corner_blind_since = None

        el = self._secs(self._corner_hold_since) \
            if self._corner_hold_since else 0.0
        da = f"{line_a['dist']:.2f}m" if line_a is not None else "---"
        db = f"{line_b['dist']:.2f}m" if line_b is not None else "---"
        self._table(
            f"CORNER{self._pending_corner_number} hold "
            f"{el:.0f}/{self._corner1_hold_s:.0f}s "
            f"{self._pending_line_a}={da} {self._pending_line_b}={db} "
            f"band={self._acq_left_lo:.1f}-{self._acq_left_hi:.1f}m",
            0.0, self._nearest)

        blind = (self._corner_blind_since is not None
                 and self._secs(self._corner_blind_since)
                 >= self._corner_blind_validate_s)
        overdue = self._secs(self._hold_phase_start) >= self._corner_hold_max_s
        if not (blind or overdue) and (self._corner_hold_since is None
                                       or el < self._corner1_hold_s):
            return
        if blind or overdue:
            self.get_logger().warn(
                (f"Corner {self._pending_corner_number} blind spot: "
                 "detection 0 at the vertex for "
                 f"{self._corner_blind_validate_s:.0f}s"
                 if blind else
                 f"Corner {self._pending_corner_number} hold ran "
                 f"{self._corner_hold_max_s:.0f}s without an uninterrupted "
                 "in-band window")
                + " - the in-band candidate was already confirmed, so the "
                "corner is validated NOW and the mission proceeds.")

        # ── full 5 s in-band hold complete → count (with dedup) ─────
        new_corner = self._count_corner(self._pending_corner_reason)
        self._corner_hold_since = None
        self._corner_hold_flicker_since = None
        if not new_corner:
            self.get_logger().info(
                f"Corner candidate de-duplicated (within "
                f"{self._corner_dedup_m:.1f} m of a counted corner) - "
                f"resuming {self._corner_source_phase.name}.")
            self._sp = None
            self._corner_candidate_since = None
            self._phase = self._corner_source_phase or Phase.SURVEY_STRIPE
            return
        self.get_logger().info(
            f"{self._corners_found}/{self._target_corners} corner detected")

        if self._corners_found >= self._target_corners:
            self.get_logger().info(
                f"All {self._target_corners} corners found - returning home.")
            self._begin_return()
            return

        n = self._pending_corner_number
        if n == 2:
            # start the sweep: the FIRST survey action is always the 2 m
            # RIGHT step holding the FRONT line just validated.
            self._stripe_dir = 1          # at the front; step 1 flips to -1
            self._stripe_count = 0
            self._step_start = (self._pose.pose.position.x,
                                self._pose.pose.position.y)
            self._step_end_role = "front"
            self._corner3_end_role = None
            self._reset_right_gate()
            self._corner_at_target = False
            self._corner_candidate_since = None
            self._watchdog_reset()
            self._sp = None
            self.get_logger().info(
                f"Corner 2 hold complete ({self._corner1_hold_s:.0f} s) - "
                f"starting sweep: {self._stripe_step:.2f} m RIGHT holding "
                "FRONT, then BACKWARD.")
            self._phase = Phase.SURVEY_STEP
        elif n == 3:
            # Corner 3 fixes the final travel direction — NEVER assumed.
            if self._corner3_end_role == "front":
                self._right_follow_dir = -1
                self._right_follow_target = "back"
                where, sense = "FRONT", "BACKWARD"
            else:
                self._right_follow_dir = 1
                self._right_follow_target = "front"
                where, sense = "BACK", "FORWARD"
            self._right_follow_start = (self._pose.pose.position.x,
                                        self._pose.pose.position.y)
            self._right_lost_since = None
            self._corner_at_target = False
            self._corner_candidate_since = None
            self._watchdog_reset()
            self._sp = None
            self.get_logger().info(
                f"Corner 3 found at {where} - following RIGHT boundary "
                f"{sense} to find Corner 4.")
            self._phase = Phase.FOLLOW_RIGHT_END
        else:
            # Corner 4 (or any residual branch) → go where the descriptor
            # says; RETURN is the Corner 4 default.
            self._sp = None
            self._phase = self._after_corner or Phase.RETURN

    # ══════════════════════════════════════════════════════════════
    #  LAWNMOWER SURVEY: 2 m RIGHT steps + BACK/FORWARD stripes, with the
    #  0.5-1.8 m RIGHT acquisition gate feeding the Corner 3 approach.
    # ══════════════════════════════════════════════════════════════
    def _do_survey_stripe(self):
        """Alternate BACK/FORWARD stripes until Corner 3.

        RIGHT-GATE-CLOSED — normal survey: open cruise along the locked
        stripe direction. Only the expected END line tapers the approach
        (_approach); a classified right line farther than right_gate_hi_m
        is RIGHT-FAR-IGNORED — it appears in telemetry, the wall veto and
        breach recovery still see it, but it adds ZERO corner motion and
        ZERO corner state (the stripe never turns diagonally toward it).
        The end held in-band for reach_confirm_s while the gate is closed
        is a plain mid-edge → SURVEY_STEP. If an in-window right sample
        begins while already at the end, the end is held long enough to
        finish right_gate_confirm_s before deciding the next phase.

        RIGHT-GATE-OPEN — six-mode follow: reference=right, partner=the
        active end (FRONT for d>0, BACK for d<0), tangent = d × locked
        FORWARD at survey_speed. Corner 3 direction-independence: the
        candidate is FRONT+RIGHT or BACK+RIGHT from the LIVE stripe
        direction, never assumed. Only a CLASSIFIED right line may
        participate — never the old left boundary or a nearest line."""
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
            self._reach_since = None
            self._corner_candidate_since = None
            self._right_lost_since = None
        fx, fy, _, _ = self._dir_vectors()
        d = self._stripe_dir
        dir_txt = "FWD" if d > 0 else "BACK"
        end_role = "front" if d > 0 else "back"
        lines = self._lines_dir()
        if self._breach_recover(lines, dt):
            self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)
            self._table("STRIPE RECOVER (too close, easing back)",
                        0.0, self._nearest)
            self._watchdog_pause()
            return
        if self._watchdog_recovering():
            return
        cls = self._classify(lines)
        end = cls[end_role]
        right = cls['right']
        self._update_right_gate(right)

        near_end = end['dist'] if end is not None else -1.0
        near_right = right['dist'] if right is not None else -1.0
        end_txt = f"{near_end:.2f}m" if end is not None else "---"
        right_txt = f"{near_right:.2f}m" if right is not None else "---"
        travelled = d * (
            (self._pose.pose.position.x - self._acq_start[0]) * fx
            + (self._pose.pose.position.y - self._acq_start[1]) * fy)

        if self._right_gate_latched:
            # ── RIGHT-GATE-OPEN: six-mode right follow to the active end ──
            end_usable = (end is not None
                          and self._pair_perp(end, right)
                          and end['dist'] <= self._corner_side_reach)
            speed = self._watchdog_speed(self._survey_speed)
            mode, at_target, self._right_lost_since, sliding = \
                self._six_mode_step(right, end, end_usable,
                                    (d * fx, d * fy), speed,
                                    self._right_lost_since, dt)
            if sliding:
                self._watchdog_track("forward" if d > 0 else "backward",
                                     (d * fx, d * fy))
            else:
                self._watchdog_pause()
            if self._hold_escalate(mode == "HOLD",
                                   "forward" if d > 0 else "backward",
                                   (d * fx, d * fy)):
                self._pub_vel_hold()
                return

            self._apply_boundary(lines, dt)
            self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)

            label = self._mode_label(mode, "right", "end", "SLIDE-RIGHT-LINE")
            if mode == "SLIDE" and end is not None and not end_usable:
                label += " END-FAR-IGNORED"
            self._table(
                f"STRIPE#{self._stripe_count} {dir_txt}[{label}] "
                f"trav={travelled:.2f}/{self._max_stripe_m:.1f} "
                f"right={right_txt} end={end_txt} "
                f"band={self._acq_left_lo:.1f}-{self._acq_left_hi:.1f}m",
                0.0, near_right)

            if self._corner_candidate(at_target):
                self._corner3_end_role = end_role   # from LIVE stripe dir
                self.get_logger().info(
                    f"Corner 3 ({end_role}-right): both lines in band "
                    f"({end_role} {near_end:.2f} m, right {near_right:.2f} m, "
                    f"band {self._acq_left_lo:.1f}-{self._acq_left_hi:.1f} m)"
                    f" - holding {self._corner1_hold_s:.0f} s to confirm.")
                self._enter_corner_hold(3, end_role, "right",
                                        Phase.SURVEY_STRIPE,
                                        Phase.FOLLOW_RIGHT_END)
                return
            if self._candidate_blind(lines):
                self._corner3_end_role = end_role
                self.get_logger().warn(
                    "Corner 3 vertex blind spot (detection 0 right after "
                    "both lines were in band) - proceeding to the Corner 3 "
                    "hold.")
                self._enter_corner_hold(3, end_role, "right",
                                        Phase.SURVEY_STRIPE,
                                        Phase.FOLLOW_RIGHT_END)
                return
        else:
            # ── RIGHT-GATE-CLOSED: normal survey; far right is ignored ──
            end_in_band = self._line_in_band(end)
            if end_in_band and self._right_gate_confirming():
                # in-window right sample while already at the end: hold the
                # end long enough to finish the gate confirmation instead of
                # racing into SURVEY_STEP. If it fails, step as normal.
                self._watchdog_pause()
                mode = (f"GATE-CONFIRM right={right_txt} "
                        f"{self._secs(self._right_gate_since):.1f}/"
                        f"{self._right_gate_confirm_s:.1f}s")
            else:
                factor = self._approach(end)
                speed = self._watchdog_speed(self._survey_speed) * factor
                self._sp[0] += d * fx * speed * dt
                self._sp[1] += d * fy * speed * dt
                if factor >= 1.0 - 1e-6:
                    # genuine full-speed cruise → the watchdog measures
                    # progress (no yellow in open space is normal, NOT a
                    # stall — the test is ACTUAL pose progress along the
                    # remembered direction, not detector response)
                    self._watchdog_track(
                        "forward" if d > 0 else "backward", (d * fx, d * fy))
                else:
                    # approach taper / acquire-hold is intentional
                    self._watchdog_pause()
                # ANTI-FREEZE: committed to the end line but it has been
                # gone (approach factor pinned at 0) -> escalate into the
                # stuck recovery instead of sitting forever.
                if self._hold_escalate(
                        self._appr_acquired and end is None,
                        "forward" if d > 0 else "backward",
                        (d * fx, d * fy)):
                    self._pub_vel_hold()
                    return
                mode = "RIGHT-FAR-IGNORED" if (
                    right is not None
                    and near_right > self._right_gate_hi) else "OPEN"

            self._apply_boundary(lines, dt)
            self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)

            self._table(
                f"STRIPE#{self._stripe_count} {dir_txt}[{mode}] "
                f"trav={travelled:.2f}/{self._max_stripe_m:.1f} "
                f"end={end_txt} right={right_txt} "
                f"gate={self._right_gate_lo:.1f}-{self._right_gate_hi:.1f}m",
                0.0, near_end)

            # plain mid-edge: end in-band for reach_confirm_s with the gate
            # still closed (and not mid-confirmation) → step right
            if end_in_band and not self._right_gate_confirming():
                if self._reach_since is None:
                    self._reach_since = self.get_clock().now()
                elif self._secs(self._reach_since) >= self._reach_confirm_s:
                    self._reach_since = None
                    self._step_start = (self._pose.pose.position.x,
                                        self._pose.pose.position.y)
                    self._step_end_role = end_role
                    self._sp = None
                    self._watchdog_reset()
                    self.get_logger().info(
                        f"{'Front' if d > 0 else 'Back'} edge reached; RIGHT "
                        "is outside the acquisition gate - stepping "
                        f"{self._stripe_step:.2f} m RIGHT, then "
                        f"{'BACKWARD' if d > 0 else 'FORWARD'}.")
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
        """Shift stripe_step (2 m) RIGHT holding the current end line
        (_step_end_role, FRONT or BACK — set by Corner 2 / the last stripe)
        inside the 0.5-1.2 m band.

        RIGHT-GATE-CLOSED: full strafe_speed step + end band hold. There is
        deliberately NO pre-gate right-distance brake: a classified right
        line at 4 m, 3 m, 2.2 m or any value above right_gate_hi_m logs
        RIGHT-FAR-IGNORED and the step continues at full speed —
        _apply_boundary() remains the final safety veto. An end line lost
        beyond line_bridge_s → HOLD-END-LOST (no blind right motion);
        lost far longer while blocked → RETURN, never reverse blindly.

        RIGHT-GATE-OPEN: six-mode with reference=end, partner=right,
        tangent=locked RIGHT at strafe_speed → the Corner 3 candidate.
        A missing right line after the latch also freezes the setpoint —
        never a rightward tangent toward an unmeasured boundary."""
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
            self._corner_candidate_since = None
            self._line_lost_since = None       # end-reference dropout timer
        fx, fy, lx, ly = self._dir_vectors()
        rx, ry = -lx, -ly                        # RIGHT = -LEFT
        lines = self._lines_dir()
        if self._breach_recover(lines, dt):
            self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)
            self._table("STEP RECOVER (too close, easing back)",
                        0.0, self._nearest)
            self._watchdog_pause()
            return
        if self._watchdog_recovering():
            return
        cls = self._classify(lines)
        end = cls[self._step_end_role]
        right = cls['right']
        self._update_right_gate(right)

        stepped = max(0.0, (self._pose.pose.position.x - self._step_start[0])
                      * rx
                      + (self._pose.pose.position.y - self._step_start[1])
                      * ry)
        near_end = end['dist'] if end is not None else -1.0
        near_right = right['dist'] if right is not None else -1.0
        end_txt = f"{near_end:.2f}m" if end is not None else "---"
        right_txt = f"{near_right:.2f}m" if right is not None else "---"
        role_txt = self._step_end_role.upper()

        if self._right_gate_latched:
            # ── RIGHT-GATE-OPEN: six-mode end-reference + right-partner ──
            if right is None:
                # partner-loss guard: never continue the rightward tangent
                # toward a latched boundary that can no longer be measured
                if self._right_lost_since is None:
                    self._right_lost_since = self.get_clock().now()
                self._corner_candidate_since = None
                self._watchdog_pause()
                mode = (f"RIGHT-LOST hold "
                        f"{self._secs(self._right_lost_since):.1f}s")
                at_target = False
                if self._secs(self._right_lost_since) > self._stale_land_s:
                    self.get_logger().warn(
                        "Right line lost too long after the gate latched "
                        "during the step - returning home (never blind "
                        "rightward motion).")
                    self._begin_return()
                    return
            else:
                self._right_lost_since = None
                speed = self._watchdog_speed(self._strafe_speed)
                mode, at_target, self._line_lost_since, sliding = \
                    self._six_mode_step(end, right, True, (rx, ry), speed,
                                        self._line_lost_since, dt)
                if sliding:
                    self._watchdog_track("right", (rx, ry))
                else:
                    self._watchdog_pause()
                mode = self._mode_label(mode, "end", "right", "SLIDE-RIGHT")
                if self._hold_escalate(mode.startswith("HOLD"),
                                       "right", (rx, ry)):
                    self._pub_vel_hold()
                    return

            self._apply_boundary(lines, dt)
            self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)
            self._table(
                f"STEP RIGHT[{mode}] {stepped:.2f}/{self._stripe_step:.1f} "
                f"end={role_txt} end={end_txt} right={right_txt} "
                f"band={self._acq_left_lo:.1f}-{self._acq_left_hi:.1f}m",
                0.0, near_right)

            if self._corner_candidate(at_target):
                self._corner3_end_role = self._step_end_role  # never assumed
                self.get_logger().info(
                    f"Corner 3 ({self._step_end_role}-right): both lines in "
                    f"band ({self._step_end_role} {near_end:.2f} m, right "
                    f"{near_right:.2f} m, band {self._acq_left_lo:.1f}-"
                    f"{self._acq_left_hi:.1f} m) - holding "
                    f"{self._corner1_hold_s:.0f} s to confirm.")
                self._enter_corner_hold(3, self._step_end_role, "right",
                                        Phase.SURVEY_STEP,
                                        Phase.FOLLOW_RIGHT_END)
                return
            if self._candidate_blind(lines):
                self._corner3_end_role = self._step_end_role
                self.get_logger().warn(
                    "Corner 3 vertex blind spot (detection 0 right after "
                    "both lines were in band) - proceeding to the Corner 3 "
                    "hold.")
                self._enter_corner_hold(3, self._step_end_role, "right",
                                        Phase.SURVEY_STEP,
                                        Phase.FOLLOW_RIGHT_END)
            return

        # ── RIGHT-GATE-CLOSED: normal 2 m step; far right is IGNORED ──
        if end is not None:
            self._line_lost_since = None
        elif self._line_lost_since is None:
            self._line_lost_since = self.get_clock().now()
        end_lost_dur = self._secs(self._line_lost_since) \
            if self._line_lost_since else 0.0
        end_lost_hold = end is None and end_lost_dur > self._line_bridge_s

        if end_lost_hold:
            # never blind right motion without the end reference; if this
            # persists (wall veto blocking, tape gone) → return, never
            # reverse blindly or falsely declare a corner
            self._watchdog_pause()
            mode = "HOLD-END-LOST"
            if end_lost_dur > self._stale_land_s:
                self.get_logger().warn(
                    f"End line lost {end_lost_dur:.1f}s during the right "
                    "step - returning home.")
                self._begin_return()
                return
        else:
            speed = self._watchdog_speed(self._strafe_speed)
            dex, dey = self._band_correction(end, dt)   # (0,0) if None
            self._sp[0] += rx * speed * dt + dex
            self._sp[1] += ry * speed * dt + dey
            self._watchdog_track("right", (rx, ry))
            if self._right_gate_confirming():
                mode = (f"GATE-CONFIRM "
                        f"{self._secs(self._right_gate_since):.1f}/"
                        f"{self._right_gate_confirm_s:.1f}s")
            elif right is not None and near_right > self._right_gate_hi:
                mode = "RIGHT-FAR-IGNORED"
            else:
                mode = "OPEN"

        self._apply_boundary(lines, dt)
        self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)
        self._table(
            f"STEP RIGHT[{mode}] {stepped:.2f}/{self._stripe_step:.1f} "
            f"end={role_txt} end={end_txt} right={right_txt} "
            f"gate={self._right_gate_lo:.1f}-{self._right_gate_hi:.1f}m",
            0.0, near_right)

        # normal completion — ONLY while the right gate is still closed
        if stepped >= self._stripe_step:
            self._stripe_dir *= -1               # next stripe reverses
            self._stripe_count += 1
            self._reach_since = None
            self._acq_start = (self._pose.pose.position.x,
                               self._pose.pose.position.y)
            self._sp = None
            self._watchdog_reset()
            if self._stripe_count > self._max_stripes:
                self.get_logger().warn(
                    f"Stripe cap ({self._max_stripes}) hit with "
                    f"{self._corners_found} corners - returning home.")
                self._begin_return()
                return
            self.get_logger().info(
                f"Stepped {stepped:.2f} m RIGHT - stripe "
                f"#{self._stripe_count} "
                + ("FORWARD." if self._stripe_dir > 0 else "BACKWARD."))
            self._phase = Phase.SURVEY_STRIPE

    # ══════════════════════════════════════════════════════════════
    #  FOLLOW_RIGHT_END: after Corner 3, follow RIGHT to the opposite end
    # ══════════════════════════════════════════════════════════════
    def _do_follow_right_end(self):
        """This phase does NOT resume the lawnmower. Corner 3 fixed the
        final travel direction: at FRONT → follow the RIGHT boundary
        BACKWARD (target = back line); at BACK → FORWARD (target = front).

        Six-mode mapping: reference=right, partner=_right_follow_target,
        tangent=_right_follow_dir × locked FORWARD at survey_speed. The
        opposite end is a usable partner only with the correct role, the
        perpendicular-pair test and dist ≤ corner_side_reach_m — a distant
        end line is END-FAR-IGNORED (no diagonal aiming). Corner 4 =
        right + opposite end both in the 0.5-1.2 m band, 5 s CORNER_HOLD,
        then RETURN."""
        if self._vio_fault():
            return
        if self._boundary_dead():
            return
        dt = 1.0 / self._sp_rate_hz
        if self._sp is None:
            self._sp = [self._pose.pose.position.x, self._pose.pose.position.y]
            if self._right_follow_start is None:
                self._right_follow_start = (self._pose.pose.position.x,
                                            self._pose.pose.position.y)
            self._corner_candidate_since = None
        fx, fy, _, _ = self._dir_vectors()
        rd = self._right_follow_dir
        dir_txt = "FWD" if rd > 0 else "BACK"
        lines = self._lines_dir()
        if self._breach_recover(lines, dt):
            self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)
            self._table("FOLLOW_RIGHT_END RECOVER (too close, easing back)",
                        0.0, self._nearest)
            self._watchdog_pause()
            return
        if self._watchdog_recovering():
            return
        cls = self._classify(lines)
        right = cls['right']
        target_end = cls[self._right_follow_target]
        end_usable = (target_end is not None
                      and self._pair_perp(target_end, right)
                      and target_end['dist'] <= self._corner_side_reach)

        speed = self._watchdog_speed(self._survey_speed)
        mode, at_target, self._right_lost_since, sliding = \
            self._six_mode_step(right, target_end, end_usable,
                                (rd * fx, rd * fy), speed,
                                self._right_lost_since, dt)
        if sliding:
            self._watchdog_track("forward" if rd > 0 else "backward",
                                 (rd * fx, rd * fy))
        else:
            self._watchdog_pause()
        if self._hold_escalate(mode == "HOLD",
                               "forward" if rd > 0 else "backward",
                               (rd * fx, rd * fy)):
            self._pub_vel_hold()
            return

        self._apply_boundary(lines, dt)
        self._pub_sp(self._sp[0], self._sp[1], self._cruise_alt)

        near_right = right['dist'] if right is not None else -1.0
        near_end = target_end['dist'] if target_end is not None else -1.0
        right_txt = f"{near_right:.2f}m" if right is not None else "---"
        end_txt = f"{near_end:.2f}m" if target_end is not None else "---"
        travelled = rd * (
            (self._pose.pose.position.x - self._right_follow_start[0]) * fx
            + (self._pose.pose.position.y - self._right_follow_start[1]) * fy)
        label = self._mode_label(mode, "right",
                                 self._right_follow_target.upper(),
                                 "SLIDE-RIGHT-LINE")
        if mode == "SLIDE" and target_end is not None and not end_usable:
            label += " END-FAR-IGNORED"
        self._table(
            f"FOLLOW_RIGHT_END[{dir_txt}][{label}] "
            f"trav={travelled:.2f}/{self._max_stripe_m:.1f} "
            f"right={right_txt} {self._right_follow_target}={end_txt} "
            f"band={self._acq_left_lo:.1f}-{self._acq_left_hi:.1f}m",
            0.0, near_right)

        here = (self._pose.pose.position.x, self._pose.pose.position.y)
        if at_target and self._pos_near_counted(here, self._corner_dedup_m):
            at_target = False                    # Corner 3's own position
        if self._corner_candidate(at_target):
            self.get_logger().info(
                f"Corner 4 (right-{self._right_follow_target}): both lines "
                f"in band (right {near_right:.2f} m, "
                f"{self._right_follow_target} {near_end:.2f} m, band "
                f"{self._acq_left_lo:.1f}-{self._acq_left_hi:.1f} m) - "
                f"holding {self._corner1_hold_s:.0f} s to confirm.")
            self._enter_corner_hold(4, "right", self._right_follow_target,
                                    Phase.FOLLOW_RIGHT_END, Phase.RETURN)
            return
        if self._candidate_blind(lines):
            self.get_logger().warn(
                "Corner 4 vertex blind spot (detection 0 right after both "
                "lines were in band) - proceeding to the Corner 4 hold.")
            self._enter_corner_hold(4, "right", self._right_follow_target,
                                    Phase.FOLLOW_RIGHT_END, Phase.RETURN)
            return

        if travelled > self._max_stripe_m:
            self.get_logger().warn(
                f"No {self._right_follow_target} end within "
                f"{self._max_stripe_m:.1f} m along the right boundary - "
                "returning home.")
            self._begin_return()

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
            f"IF={self._init_factor:.2f}  "
            f"corners={self._corners_found}/{self._target_corners}")


def main():
    rclpy.init()
    node = Corner1TestAuto()
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