#!/usr/bin/env python3
"""
mission_director — final mission state machine (FINAL_ARCHITECTURE.md).

Runs as its own process, parallel to vio_gate and the RTAB-Map stack —
all started together by bringup.launch.py on the ground. This node
spawns NOTHING: RTAB-Map belongs to the launch system, the bridge is
the vio_gate node. In-air transitions are service calls and booleans.

Phases:
  IDLE       preflight health gate + wait CH5 LOW
  ARM        setpoint stream → OFFBOARD → arm; home + heading lock
  TAKEOFF    flow-only climb to target_alt (3 m)
  STABLE_OF  zero-velocity hold; flow must be stable
  SEED       /viman/seed → RTAB odom reset + frame alignment
  VALIDATE   initialization factor ≥ threshold for confirm window,
             with optional small motion square (frame errors are
             invisible at a standstill — motion exposes them, on flow,
             where they're survivable)
  HANDOVER   /viman/gate true (gate re-anchors → zero-innovation start)
  HOVER      fused VIO hold; vio_gate watchdogs monitored
  FLOW_HOLD  vision fault recovery: flow hold → re-seed (max retries)
  LAND       gate closed FIRST, then AUTO.LAND (flow-only descent)
  DISARM     wait for PX4 self-disarm
  SAFE_MANUAL CH5 HIGH at any time → STABILIZED, pilot has control

Abort hierarchy: RC CH5 HIGH (primary, always works) > Ctrl+C when run
standalone. NOTE: under `ros2 launch`, Ctrl+C SIGINTs every node at
once — PX4's offboard-loss failsafe (COM_OBL_RC_ACT) is what catches
the drone then. Set it to AUTO.LAND in QGC.
"""

import math
import signal
import time
from enum import Enum, auto

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import RCIn, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, UInt8
from std_srvs.srv import SetBool, Trigger

from viman_mission.common import (qos_best_effort, qos_reliable,
                                  yaw_deg_from_quaternion)

# vio_gate states (keep in sync with vio_gate.GateState)
GS_VALIDATING = 2
GS_OPEN       = 3
GS_FAULT_MIN  = 4   # any state ≥ this is a fault


class Phase(Enum):
    IDLE        = auto()
    ARM         = auto()
    TAKEOFF     = auto()
    STABLE_OF   = auto()
    SEED        = auto()
    VALIDATE    = auto()
    HANDOVER    = auto()
    HOVER       = auto()
    RETURN      = auto()
    FLOW_SETTLE = auto()   # camera cut off, EKF settles on flow at altitude
    DESCEND     = auto()   # OFFBOARD precision descent, X/Y locked on home
    FLOW_HOLD   = auto()
    LAND        = auto()   # AUTO.LAND: final touchdown + disarm only
    DISARM      = auto()
    SAFE_MANUAL = auto()
    DONE        = auto()


class MissionDirector(Node):

    def __init__(self):
        super().__init__("mission_director")

        # ── Parameters ───────────────────────────────────────────
        p = self.declare_parameters("", [
            ("target_alt",        3.0),    # flow-only takeoff altitude (m)
            ("alt_tolerance",     0.12),
            ("at_alt_confirm_s",  1.5),
            ("stable_of_secs",    4.0),    # flow settle before seeding
            ("seed_timeout_s",    10.0),
            ("validate_if_min",   0.8),    # IF threshold
            ("validate_hold_s",   5.0),    # IF must hold this long
            ("validate_dip_grace_s", 1.0), # brief dips below threshold
                                           # tolerated without resetting
            ("validate_timeout_s", 45.0),
            ("motion_test",       True),   # ±amp square during VALIDATE
            ("motion_amp_m",      0.3),
            ("motion_leg_s",      3.0),
            ("handover_settle_s", 2.0),
            ("hover_duration",    30.0),
            ("max_revalidations", 2),
            ("sp_rate_hz",        20.0),
            ("rc_ch5_index",      4),
            ("rc_start_low",      1200),
            ("rc_interrupt_high", 1700),
            ("preflight_pose_hz_min", 15.0),
            ("rtab_odom_topic",   "/rtabmap/rtabmap/odom"),
            ("return_enabled",    True),   # fly back to home before landing
            ("return_speed_ms",   0.3),    # gentle approach speed
            ("return_radius_m",   0.20),   # arrival acceptance radius
            ("return_timeout_s",  20.0),   # give up → land where we are
            ("land_settle_s",     2.5),    # flow-only settle before descent
            ("descend_enabled",   True),   # OFFBOARD precision descent
            ("descend_speed_ms",  0.25),   # gentle sink rate
            ("descend_handoff_alt_m", 0.3),  # below this AUTO.LAND takes
                                             # over (touchdown + disarm)
            ("descend_timeout_s", 15.0),   # stuck (ground effect?) → AUTO.LAND
        ])
        (self._target_alt, self._alt_tol, self._at_alt_confirm_s,
         self._stable_of_secs, self._seed_timeout_s, self._if_min,
         self._validate_hold_s, self._dip_grace_s,
         self._validate_timeout_s,
         self._motion_test, self._motion_amp, self._motion_leg_s,
         self._handover_settle_s, self._hover_duration,
         self._max_revalidations, self._sp_rate_hz, self._rc_ch5_idx,
         self._rc_start_low, self._rc_interrupt_high,
         self._pose_hz_min, rtab_topic,
         self._return_enabled, self._return_speed,
         self._return_radius, self._return_timeout,
         self._land_settle_s, self._descend_enabled,
         self._descend_speed, self._descend_handoff_alt,
         self._descend_timeout) = (x.value for x in p)

        self._phase      = Phase.IDLE
        self._last_phase = None

        # Locked references
        self._hold_x = 0.0
        self._hold_y = 0.0
        self._home_x = 0.0               # pad position (captured at arm) —
        self._home_y = 0.0               # the RETURN phase flies back here
        self._hold_heading_q = None
        self._validate_anchor = None     # (x, y) for the motion square
        self._ret_sp = None              # crawling return setpoint
        self._ret_start = None
        self._ret_arrived_since = None
        self._settle_start = None        # flow-settle timer before landing
        self._desc_z = 0.0               # OFFBOARD descent setpoint
        self._desc_start = None

        # Live data
        self._pose = PoseStamped()
        self._pose.pose.orientation.w = 1.0
        self._state = State()
        self._rc    = ()
        self._vio_state   = 255          # unknown until gate reports
        self._init_factor = 0.0
        self._pose_stamps = []           # ns, for preflight rate check
        self._last_rtab_ns = 0           # rtab odom aliveness

        # Timers / counters
        self._at_alt_since   = None
        self._stable_since   = None
        self._seed_sent      = False
        self._seed_start     = None
        self._validate_start = None
        self._if_good_since  = None
        self._handover_sent  = False
        self._handover_start = None
        self._hover_start    = None
        self._revalidations  = 0
        self._offboard_requested = False
        self._arm_requested      = False
        self._gate_close_sent    = False
        self._land_requested     = False

        self._ctrl_c = False
        signal.signal(signal.SIGINT, self._sigint)
        self._last_print = 0.0

        # ── ROS interfaces ───────────────────────────────────────
        cb = ReentrantCallbackGroup()
        qos_be, qos_rel = qos_best_effort(), qos_reliable()

        self.create_subscription(State, "/mavros/state",
                                 self._state_cb, qos_rel, callback_group=cb)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._pose_cb, qos_be, callback_group=cb)
        self.create_subscription(RCIn, "/mavros/rc/in",
                                 self._rc_cb, qos_be, callback_group=cb)
        self.create_subscription(Odometry, rtab_topic,
                                 self._rtab_alive_cb, qos_rel,
                                 callback_group=cb)
        self.create_subscription(UInt8, "/viman/vio_state",
                                 self._vio_state_cb, 10, callback_group=cb)
        self.create_subscription(Float32, "/viman/init_factor",
                                 self._if_cb, 10, callback_group=cb)

        self._sp_pub  = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)
        self._vel_pub = self.create_publisher(
            TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10)

        self._arm_cli  = self.create_client(
            CommandBool, "/mavros/cmd/arming", callback_group=cb)
        self._mode_cli = self.create_client(
            SetMode, "/mavros/set_mode", callback_group=cb)
        self._seed_cli = self.create_client(
            Trigger, "/viman/seed", callback_group=cb)
        self._gate_cli = self.create_client(
            SetBool, "/viman/gate", callback_group=cb)

        self._handlers = {
            Phase.IDLE:        self._do_idle,
            Phase.ARM:         self._do_arm,
            Phase.TAKEOFF:     self._do_takeoff,
            Phase.STABLE_OF:   self._do_stable_of,
            Phase.SEED:        self._do_seed,
            Phase.VALIDATE:    self._do_validate,
            Phase.HANDOVER:    self._do_handover,
            Phase.HOVER:       self._do_hover,
            Phase.RETURN:      self._do_return,
            Phase.FLOW_SETTLE: self._do_flow_settle,
            Phase.DESCEND:     self._do_descend,
            Phase.FLOW_HOLD:   self._do_flow_hold,
            Phase.LAND:        self._do_land,
            Phase.DISARM:      self._do_disarm,
            Phase.SAFE_MANUAL: self._do_safe_manual,
            Phase.DONE:        lambda: None,
        }

        self.create_timer(1.0 / self._sp_rate_hz, self._loop,
                          callback_group=cb)
        self.get_logger().info(
            "MissionDirector up. Preflight checks running; "
            f"CH5 ≤ {self._rc_start_low} to start once all pass.")

    # ════════════════════════════════════════════════════════════
    #  CALLBACKS
    # ════════════════════════════════════════════════════════════

    def _state_cb(self, m):  self._state = m
    def _vio_state_cb(self, m): self._vio_state = m.data
    def _if_cb(self, m):     self._init_factor = m.data
    def _rtab_alive_cb(self, m):
        self._last_rtab_ns = self.get_clock().now().nanoseconds

    def _pose_cb(self, m):
        self._pose = m
        now_ns = self.get_clock().now().nanoseconds
        self._pose_stamps.append(now_ns)
        cutoff = now_ns - 2_000_000_000
        while self._pose_stamps and self._pose_stamps[0] < cutoff:
            self._pose_stamps.pop(0)

    def _rc_cb(self, m):
        self._rc = m.channels
        if self._phase in (Phase.SAFE_MANUAL, Phase.DONE, Phase.IDLE):
            return
        if self._ch5() >= self._rc_interrupt_high:
            self.get_logger().warn(
                f"⚠ RC INTERRUPT — CH5={self._ch5()}. STABILIZED, "
                "pilot has control.")
            self._request_mode("STABILIZED")
            self._phase = Phase.SAFE_MANUAL

    def _sigint(self, *_):
        # On the ground / already done → just exit cleanly.
        if self._phase in (Phase.IDLE, Phase.DONE, Phase.SAFE_MANUAL,
                           Phase.DISARM) or self._ctrl_c:
            raise KeyboardInterrupt   # second press also forces exit
        self.get_logger().warn("Ctrl+C — emergency AUTO.LAND "
                               "(press again to force exit)")
        self._ctrl_c = True

    # ════════════════════════════════════════════════════════════
    #  LOOP
    # ════════════════════════════════════════════════════════════

    def _loop(self):
        if self._phase != self._last_phase:
            self.get_logger().info(f"══ Phase: {self._phase.name} ══")
            self._last_phase = self._phase

        if self._ctrl_c and self._phase not in (
                Phase.LAND, Phase.DISARM, Phase.DONE, Phase.SAFE_MANUAL):
            self._phase = Phase.LAND
            return

        self._handlers[self._phase]()

    # ── IDLE: preflight health gate ───────────────────────────────

    def _preflight_failures(self):
        now_ns = self.get_clock().now().nanoseconds
        fails = []
        if not self._state.connected:
            fails.append("FCU not connected")
        if len(self._pose_stamps) / 2.0 < self._pose_hz_min:
            fails.append(
                f"pose rate {len(self._pose_stamps)/2.0:.0f} Hz "
                f"< {self._pose_hz_min:.0f}")
        if len(self._rc) <= self._rc_ch5_idx:
            fails.append("no RC frames")
        if now_ns - self._last_rtab_ns > 2_000_000_000:
            fails.append("RTAB odom silent (camera/rtabmap not alive?)")
        if self._vio_state == 255:
            fails.append("vio_gate not reporting")
        if not self._seed_cli.service_is_ready():
            fails.append("/viman/seed not ready")
        if not self._arm_cli.service_is_ready() \
                or not self._mode_cli.service_is_ready():
            fails.append("MAVROS services not ready")
        return fails

    def _do_idle(self):
        self._pub_sp(0.0, 0.0, 0.3)   # pre-stream for OFFBOARD

        fails = self._preflight_failures()
        if fails:
            self.get_logger().warn(
                "PREFLIGHT BLOCKED: " + "; ".join(fails),
                throttle_duration_sec=5.0)
            return
        self.get_logger().info("Preflight PASSED — waiting for CH5 LOW",
                               throttle_duration_sec=5.0)
        if self._ch5() <= self._rc_start_low:
            self.get_logger().info(f"CH5={self._ch5()} — start trigger")
            self._phase = Phase.ARM

    # ── ARM ───────────────────────────────────────────────────────

    def _do_arm(self):
        self._pub_sp(0.0, 0.0, 0.3)
        if not self._offboard_requested:
            self._request_mode("OFFBOARD")
            self._offboard_requested = True
            return
        if self._state.mode != "OFFBOARD":
            self._request_mode("OFFBOARD")
            return
        if not self._arm_requested:
            self._request_arm(True)
            self._arm_requested = True
            return
        if self._state.armed and self._state.mode == "OFFBOARD":
            self._hold_x = self._pose.pose.position.x
            self._hold_y = self._pose.pose.position.y
            self._home_x, self._home_y = self._hold_x, self._hold_y
            self._hold_heading_q = self._pose.pose.orientation
            self.get_logger().info(
                f"Armed. Home x={self._hold_x:.3f} y={self._hold_y:.3f} "
                f"yaw={yaw_deg_from_quaternion(self._hold_heading_q):.1f}°")
            self._phase = Phase.TAKEOFF

    # ── TAKEOFF (flow only) ───────────────────────────────────────

    def _do_takeoff(self):
        self._pub_sp(self._hold_x, self._hold_y, self._target_alt)
        alt = self._pose.pose.position.z
        if abs(alt - self._target_alt) <= self._alt_tol:
            if self._at_alt_since is None:
                self._at_alt_since = self.get_clock().now()
            elif self._secs(self._at_alt_since) >= self._at_alt_confirm_s:
                self.get_logger().info(f"Reached {alt:.2f} m on flow")
                self._stable_since = self.get_clock().now()
                self._phase = Phase.STABLE_OF
        else:
            self._at_alt_since = None
        self._tele(f"TAKEOFF alt={alt:.2f}/{self._target_alt:.1f}")

    # ── STABLE_OF ─────────────────────────────────────────────────

    def _do_stable_of(self):
        self._pub_vel_hold()
        el = self._secs(self._stable_since)
        self._tele(f"STABLE_OF {el:.1f}/{self._stable_of_secs:.0f}s")
        if el >= self._stable_of_secs:
            self._seed_sent = False
            self._phase = Phase.SEED

    # ── SEED ──────────────────────────────────────────────────────

    def _do_seed(self):
        self._pub_vel_hold()
        if not self._seed_sent:
            if not self._seed_cli.service_is_ready():
                self.get_logger().error("/viman/seed not ready",
                                        throttle_duration_sec=2.0)
                return
            self._seed_cli.call_async(Trigger.Request())
            self._seed_sent  = True
            self._seed_start = self.get_clock().now()
            self.get_logger().info("Seed requested (RTAB reset + alignment)")
            return
        if self._vio_state == GS_VALIDATING:
            self._validate_start = self.get_clock().now()
            self._if_good_since  = None
            self._validate_anchor = (self._pose.pose.position.x,
                                     self._pose.pose.position.y)
            self._phase = Phase.VALIDATE
        elif self._secs(self._seed_start) > self._seed_timeout_s:
            self.get_logger().error("Seed timeout — retrying via FLOW_HOLD")
            self._phase = Phase.FLOW_HOLD

    # ── VALIDATE ──────────────────────────────────────────────────

    def _motion_offset(self):
        """Slow square on flow setpoints. A still drone validates nothing —
        frame errors only show under motion."""
        if not self._motion_test:
            return 0.0, 0.0
        corners = ((0.0, 0.0), (self._motion_amp, 0.0),
                   (self._motion_amp, self._motion_amp),
                   (0.0, self._motion_amp))
        leg = int(self._secs(self._validate_start) / self._motion_leg_s)
        return corners[leg % 4]

    def _do_validate(self):
        ox, oy = self._motion_offset()
        ax, ay = self._validate_anchor
        self._pub_sp(ax + ox, ay + oy, self._target_alt)

        el = self._secs(self._validate_start)
        if self._vio_state >= GS_FAULT_MIN:
            self.get_logger().warn("Gate fault during validation — re-seeding")
            self._phase = Phase.FLOW_HOLD
            return
        # Dip-grace: a brief flicker below threshold doesn't reset the
        # confirmation clock — only a SUSTAINED drop does. Flight noise
        # makes single-sample dips routine; without grace, 5 continuous
        # seconds is nearly impossible airborne.
        if self._init_factor >= self._if_min:
            self._if_low_since = None
            if self._if_good_since is None:
                self._if_good_since = self.get_clock().now()
            if self._secs(self._if_good_since) >= self._validate_hold_s:
                self.get_logger().info(
                    f"VALIDATED: IF={self._init_factor:.2f} held "
                    f"{self._validate_hold_s:.0f}s — handing over")
                self._handover_sent = False
                self._phase = Phase.HANDOVER
                return
        elif self._if_good_since is not None:
            if getattr(self, "_if_low_since", None) is None:
                self._if_low_since = self.get_clock().now()
            elif self._secs(self._if_low_since) > self._dip_grace_s:
                self._if_good_since = None
                self._if_low_since = None

        self._tele(f"VALIDATE IF={self._init_factor:.2f} "
                   f"(need ≥{self._if_min:.2f} for "
                   f"{self._validate_hold_s:.0f}s)  t={el:.0f}s")

        if el > self._validate_timeout_s:
            self.get_logger().error(
                f"Validation timeout ({self._validate_timeout_s:.0f}s) — "
                "RTAB never agreed with flow. Landing on flow.")
            self._phase = Phase.LAND

    # ── HANDOVER ──────────────────────────────────────────────────

    def _do_handover(self):
        self._pub_sp(self._validate_anchor[0], self._validate_anchor[1],
                     self._target_alt)
        if not self._handover_sent:
            req = SetBool.Request()
            req.data = True
            self._gate_cli.call_async(req)
            self._handover_sent  = True
            self._handover_start = self.get_clock().now()
            return
        if self._vio_state == GS_OPEN:
            if self._secs(self._handover_start) >= self._handover_settle_s:
                # Re-lock hold position from current EKF estimate
                self._hold_x = self._pose.pose.position.x
                self._hold_y = self._pose.pose.position.y
                self._hover_start = self.get_clock().now()
                self.get_logger().info("Gate OPEN, settled — hover begins")
                self._phase = Phase.HOVER
        elif self._secs(self._handover_start) > 5.0:
            self.get_logger().error("Gate did not open — recovering")
            self._phase = Phase.FLOW_HOLD

    # ── HOVER (fused VIO) ─────────────────────────────────────────

    def _do_hover(self):
        self._pub_sp(self._hold_x, self._hold_y, self._target_alt)

        if self._vio_state >= GS_FAULT_MIN:
            self.get_logger().error(
                f"VIO fault (gate state {self._vio_state}) — flow hold")
            self._phase = Phase.FLOW_HOLD
            return

        el = self._secs(self._hover_start)
        self._tele(f"HOVER {self._hover_duration - el:.0f}s left  "
                   f"alt={self._pose.pose.position.z:.2f}  "
                   f"IF={self._init_factor:.2f}")
        if el >= self._hover_duration:
            if self._return_enabled:
                self.get_logger().info(
                    f"Hover complete — returning to home "
                    f"({self._home_x:.2f}, {self._home_y:.2f})")
                self._ret_sp = [self._pose.pose.position.x,
                                self._pose.pose.position.y]
                self._ret_start = self.get_clock().now()
                self._ret_arrived_since = None
                self._phase = Phase.RETURN
            else:
                self.get_logger().info("Hover complete — landing")
                self._phase = Phase.LAND

    # ── RETURN: crawl back to the launch pad on fused VIO ─────────

    def _do_return(self):
        # Vision fault mid-return → don't fight it, land where we are
        # (we were about to land anyway; flow handles the descent).
        if self._vio_state >= GS_FAULT_MIN:
            self.get_logger().warn("VIO fault during return — landing here")
            self._phase = Phase.LAND
            return
        if self._secs(self._ret_start) > self._return_timeout:
            self.get_logger().warn("Return timeout — landing here")
            self._phase = Phase.LAND
            return

        # Crawl the setpoint toward home at return_speed (no jumps —
        # PX4 follows a moving target gently instead of lunging).
        step = self._return_speed / self._sp_rate_hz
        dx = self._home_x - self._ret_sp[0]
        dy = self._home_y - self._ret_sp[1]
        d = math.hypot(dx, dy)
        if d > 1e-6:
            f = min(1.0, step / d)
            self._ret_sp[0] += dx * f
            self._ret_sp[1] += dy * f
        self._pub_sp(self._ret_sp[0], self._ret_sp[1], self._target_alt)

        # Arrival: actual position (not setpoint) inside the radius
        ex = self._pose.pose.position.x - self._home_x
        ey = self._pose.pose.position.y - self._home_y
        err = math.hypot(ex, ey)
        self._tele(f"RETURN dist={err:.2f} m → home "
                   f"(radius {self._return_radius:.2f})")
        if err <= self._return_radius:
            if self._ret_arrived_since is None:
                self._ret_arrived_since = self.get_clock().now()
            elif self._secs(self._ret_arrived_since) >= 1.0:
                # Cut the camera off NOW, at altitude — the EKF gets its
                # whole flow transition done before the descent begins,
                # instead of mid-air with a degrading low-altitude camera.
                if self._gate_cli.service_is_ready():
                    req = SetBool.Request()
                    req.data = False
                    self._gate_cli.call_async(req)
                self._gate_close_sent = True
                self._settle_start = self.get_clock().now()
                self.get_logger().info(
                    f"Home reached (err={err:.2f} m) — camera OFF, "
                    f"settling on flow {self._land_settle_s:.1f}s")
                self._phase = Phase.FLOW_SETTLE
        else:
            self._ret_arrived_since = None

    def _do_flow_settle(self):
        """Hold over home on PURE optical flow while the EKF finishes
        its vision→flow transition, then descend. The camera plays no
        part in anything below this altitude."""
        self._pub_sp(self._home_x, self._home_y, self._target_alt)
        el = self._secs(self._settle_start)
        self._tele(f"FLOW_SETTLE {el:.1f}/{self._land_settle_s:.1f}s "
                   "(camera off, flow only)")
        if el >= self._land_settle_s:
            if self._descend_enabled:
                self._desc_z = self._pose.pose.position.z
                self._desc_start = self.get_clock().now()
                self.get_logger().info(
                    f"OFFBOARD precision descent: X/Y locked on home, "
                    f"sinking at {self._descend_speed:.2f} m/s to "
                    f"{self._descend_handoff_alt:.2f} m")
                self._phase = Phase.DESCEND
            else:
                self._phase = Phase.LAND

    def _do_descend(self):
        """OFFBOARD descent: X/Y actively held on home the whole way
        down — this is where AUTO.LAND would drift laterally. Below the
        handoff altitude, AUTO.LAND takes over for touchdown + disarm
        (PX4's land detector; OFFBOARD cannot detect ground safely)."""
        self._desc_z = max(0.0,
                           self._desc_z - self._descend_speed / self._sp_rate_hz)
        self._pub_sp(self._home_x, self._home_y, self._desc_z)

        alt = self._pose.pose.position.z
        ex = self._pose.pose.position.x - self._home_x
        ey = self._pose.pose.position.y - self._home_y
        self._tele(f"DESCEND alt={alt:.2f} → {self._descend_handoff_alt:.2f}"
                   f"  off=({ex:+.2f},{ey:+.2f})")

        if alt <= self._descend_handoff_alt:
            self.get_logger().info(
                f"Handoff altitude — AUTO.LAND for final touchdown "
                f"(lateral offset {math.hypot(ex, ey):.2f} m)")
            self._phase = Phase.LAND
        elif self._secs(self._desc_start) > self._descend_timeout:
            self.get_logger().warn("Descent timeout — AUTO.LAND from here")
            self._phase = Phase.LAND

    # ── FLOW_HOLD: vision fault recovery ──────────────────────────

    def _do_flow_hold(self):
        self._pub_vel_hold()
        if self._revalidations >= int(self._max_revalidations):
            self.get_logger().error(
                f"Max re-validations ({self._revalidations}) reached — "
                "landing on flow.")
            self._phase = Phase.LAND
            return
        self._revalidations += 1
        self.get_logger().warn(
            f"Recovery attempt {self._revalidations}/"
            f"{int(self._max_revalidations)} — re-seeding")
        self._stable_since = self.get_clock().now()
        self._phase = Phase.STABLE_OF   # settle on flow, then SEED again

    # ── LAND / DISARM ─────────────────────────────────────────────

    def _do_land(self):
        # Gate closes BEFORE descent: large altitude change = downward-
        # camera scene upheaval = exactly when vision lies. Land on flow.
        if not self._gate_close_sent:
            if self._gate_cli.service_is_ready():
                req = SetBool.Request()
                req.data = False
                self._gate_cli.call_async(req)
            self._gate_close_sent = True
            return
        if not self._land_requested:
            self._request_mode("AUTO.LAND")
            self._land_requested = True
            self.get_logger().info("Gate closed, AUTO.LAND requested")
            self._phase = Phase.DISARM

    def _do_disarm(self):
        if not self._state.armed:
            self.get_logger().info(
                "Disarmed — mission complete ✓ "
                "(.db saved when launch shuts rtabmap down; "
                "reprocess offline for max quality)")
            self._phase = Phase.DONE

    def _do_safe_manual(self):
        self.get_logger().info(
            f"SAFE MANUAL — CH5={self._ch5()}. Restart to fly again.",
            throttle_duration_sec=5.0)

    # ════════════════════════════════════════════════════════════
    #  HELPERS
    # ════════════════════════════════════════════════════════════

    def _pub_sp(self, x, y, z):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "local_origin"
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        m.pose.orientation = (self._hold_heading_q
                              if self._hold_heading_q is not None
                              else self._pose.pose.orientation)
        self._sp_pub.publish(m)

    def _pub_vel_hold(self):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "local_origin"
        self._vel_pub.publish(m)

    def _request_mode(self, mode):
        if not self._mode_cli.service_is_ready():
            self.get_logger().error(f"set_mode unavailable ({mode})",
                                    throttle_duration_sec=2.0)
            return
        r = SetMode.Request()
        r.custom_mode = mode
        self._mode_cli.call_async(r)

    def _request_arm(self, v):
        if not self._arm_cli.service_is_ready():
            self.get_logger().error("arming unavailable",
                                    throttle_duration_sec=2.0)
            return
        r = CommandBool.Request()
        r.value = v
        self._arm_cli.call_async(r)

    def _secs(self, t):
        if t is None:
            return 0.0
        return (self.get_clock().now() - t).nanoseconds * 1e-9

    def _ch5(self):
        if len(self._rc) > self._rc_ch5_idx:
            return int(self._rc[self._rc_ch5_idx])
        return 1500

    def _tele(self, line):
        now = time.monotonic()
        if now - self._last_print >= 1.0:
            self._last_print = now
            self.get_logger().info(line)


def main():
    rclpy.init()
    node = MissionDirector()
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
