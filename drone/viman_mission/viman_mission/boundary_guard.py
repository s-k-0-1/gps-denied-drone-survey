#!/usr/bin/env python3
"""
boundary_guard — yellow-line STICK FILTER for MANUAL (POSCTL) flight.
Team Viman Rakshak / IRoC-U 2026.

You fly the drone yourself. The guard NEVER moves the drone by itself
and NEVER freezes your transmitter. When the yellow line is nearby and
you push toward it, the guard seizes OFFBOARD and becomes a thin
FILTER between your sticks and PX4:

  · Your roll / pitch / throttle / yaw sticks are read directly from
    /mavros/rc/in and converted to velocity commands — you keep FULL
    control in every direction, exactly like POSITION mode.
  · ONLY the velocity component TOWARD the line is limited:
        allowed_toward = approach_gain × (nearest − hold_dist)
    capped at approach_vmax, ZERO at hold_dist (default 0.5 m).
    So you can approach the line slowly; it tapers to a stop ~0.5 m
    BEFORE the line, while sideways / backward / up / down / yaw all
    keep working normally — you can always pull straight back out.
  · Sticks centred → the drone just brakes to a stop (zero velocity).
    NOTHING is automatic: no pushback, no auto altitude, no position
    hold, no motion the pilot didn't command. The guard can only SLOW
    or STOP motion toward a line — it never drives the drone anywhere.

Release: the guard hands POSCTL back as soon as the line is far again
(release_dist) or lost from view — or instantly on CH5 HIGH.

CORNERS: the detector now reports each line separately, so the clamp
holds you off BOTH arms at once — push into the corner and you stop
softly, clear of each line; pull back and every direction stays free,
so you can rest in the corner or leave it entirely on the sticks. Two-
line frames are flagged (2L); L-corner detections are flagged (C).

═══════════════ RC STICK MAPPING — VERIFY ON THE GROUND ═══════════════
The filter reads RAW RC channels. Defaults assume a standard Mode-2
AETR radio (ch1 roll, ch2 pitch, ch3 throttle, ch4 yaw) with pitch
reversed (push forward = lower PWM), centre 1500 µs.

BENCH CHECK BEFORE FIRST FLIGHT (props off or drone disarmed):
  run the guard, watch the STICK column in the table while moving
  sticks:  pitch forward → STICK x positive;  roll right → STICK y
  negative;  wrong direction → flip rc_pitch_sign / rc_roll_sign in
  the yaml. DO NOT FLY until this reads correctly.
═══════════════════════════════════════════════════════════════════════

State machine:
  DISABLED   not armed / below guard_min_alt / CH5 HIGH / detector dead
  WATCH      monitoring; engages when line close AND sticks push toward
  FILTER     OFFBOARD; sticks passed through with toward-line clamp
  COOLDOWN   brief pause after release (PANIC distance bypasses it)

Safety: CH5 HIGH → instant POSCTL + guard off. RC frames stale while
filtering → instant POSCTL (PX4 handles RC failsafe). External mode
change → guard yields. Engages only from POSCTL.

Run:
  ros2 launch viman_mission boundary_guard.launch.py
"""

import math
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import RCIn, State
from mavros_msgs.srv import SetMode
from std_msgs.msg import Float32, Float32MultiArray

from viman_mission.common import qos_best_effort, qos_reliable


class GuardState:
    DISABLED = "DISABLED"
    WATCH    = "WATCH"
    FILTER   = "FILTER"
    COOLDOWN = "COOLDOWN"


class BoundaryGuard(Node):

    def __init__(self):
        super().__init__("boundary_guard")

        self.declare_parameters("", [
            # --- toward-line clamp ---
            ("govern_dist_m",      1.5),    # engage when line inside this
                                            # AND sticks push toward it
            ("hold_dist_m",        0.5),    # toward-speed reaches ZERO here
                                            # — the drone stops ~this far
                                            # BEFORE the line (un-crossable)
            ("approach_gain",      0.5),    # allowed = gain·(near−hold)
            ("approach_vmax_ms",   0.4),    # toward-speed cap while engaged
            ("engage_cmd_ms",      0.05),   # min STICK toward-command to
                                            # engage (centred sticks never
                                            # trigger)
            ("panic_dist_m",       0.15),   # engage instantly below this,
                                            # even during cooldown
            # --- release ---
            ("release_dist_m",     1.6),    # line farther than this → hand
                                            # back POSCTL
            ("release_hold_s",     0.8),    # line clear/lost this long
            ("cooldown_s",         0.5),    # pause between engagements
            ("offboard_grant_s",   3.0),    # max wait for PX4 to grant OFFBOARD
                                            # after engage (was a hard 1.5 s —
                                            # too tight, caused abort + mode
                                            # thrash). Velocity priming below
                                            # makes the grant near-instant.
            ("max_intervene_s",    0.0),    # hard cap; 0 = disabled (the
                                            # filter passes sticks through,
                                            # so staying engaged is safe)
            # --- manual-control mapping (VERIFY ON GROUND — see header) ---
            ("manual_vmax_ms",     1.0),    # full XY stick → this speed
            ("manual_vmax_z_ms",   0.6),    # full throttle stick → climb
            ("yaw_rate_max_dps",   45.0),   # full yaw stick → this rate
            ("rc_roll_index",      0),      # AETR: ch1 = roll  (0-based)
            ("rc_pitch_index",     1),      # ch2 = pitch
            ("rc_throttle_index",  2),      # ch3 = throttle
            ("rc_yaw_index",       3),      # ch4 = yaw
            ("rc_center_us",       1500),
            ("rc_halfspan_us",     450),
            ("rc_deadzone_us",     40),     # sticks inside this = zero
            ("rc_roll_sign",       1.0),    # flip if roll goes wrong way
            ("rc_pitch_sign",      1.0),    # FLIPPED (was -1.0): this radio
                                            # reads fwd as HIGHER PWM (proven in
                                            # flight). BENCH-VERIFY: push fwd →
                                            # STICK x must read POSITIVE.
            ("rc_yaw_sign",        1.0),
            ("rc_stale_s",         0.5),    # RC frames older → release
            # --- infra ---
            ("guard_min_alt_m",    1.0),
            ("position_mode",      "POSCTL"),
            ("boundary_stale_s",   1.0),
            ("corner_stale_s",     1.0),
            ("sp_rate_hz",         20.0),
            ("rc_ch5_index",       4),
            ("rc_interrupt_high",  1700),
            ("repulsion_topic",    "/viman/boundary/repulsion"),
            ("nearest_topic",      "/viman/boundary/nearest_m"),
            ("coverage_topic",     "/viman/boundary/coverage_pct"),
            ("corner_topic",       "/viman/boundary/corner"),
            ("lines_topic",        "/viman/boundary/lines"),
        ])
        gp = lambda n: self.get_parameter(n).value
        self._govern_dist  = float(gp("govern_dist_m"))
        self._hold_dist    = float(gp("hold_dist_m"))
        self._gain         = float(gp("approach_gain"))
        self._appr_vmax    = float(gp("approach_vmax_ms"))
        self._engage_cmd   = float(gp("engage_cmd_ms"))
        self._panic_dist   = float(gp("panic_dist_m"))
        self._release_dist = float(gp("release_dist_m"))
        self._release_hold = float(gp("release_hold_s"))
        self._cooldown_s   = float(gp("cooldown_s"))
        self._offboard_grant_s = float(gp("offboard_grant_s"))
        self._max_intervene = float(gp("max_intervene_s"))
        self._vmax_xy      = float(gp("manual_vmax_ms"))
        self._vmax_z       = float(gp("manual_vmax_z_ms"))
        self._yawrate_max  = math.radians(float(gp("yaw_rate_max_dps")))
        self._i_roll       = int(gp("rc_roll_index"))
        self._i_pitch      = int(gp("rc_pitch_index"))
        self._i_thr        = int(gp("rc_throttle_index"))
        self._i_yaw        = int(gp("rc_yaw_index"))
        self._rc_center    = float(gp("rc_center_us"))
        self._rc_half      = float(gp("rc_halfspan_us"))
        self._rc_dz        = float(gp("rc_deadzone_us"))
        self._s_roll       = float(gp("rc_roll_sign"))
        self._s_pitch      = float(gp("rc_pitch_sign"))
        self._s_yaw        = float(gp("rc_yaw_sign"))
        self._rc_stale_s   = float(gp("rc_stale_s"))
        self._min_alt      = float(gp("guard_min_alt_m"))
        self._pos_mode     = str(gp("position_mode"))
        self._stale_s      = float(gp("boundary_stale_s"))
        self._corner_stale_s = float(gp("corner_stale_s"))
        self._sp_rate      = float(gp("sp_rate_hz"))
        self._rc_ch5       = int(gp("rc_ch5_index"))
        self._rc_high      = int(gp("rc_interrupt_high"))

        # ── State ────────────────────────────────────────────────
        self._gstate = GuardState.DISABLED
        self._state = State()
        self._pose = PoseStamped()
        self._pose.pose.orientation.w = 1.0
        self._rc = ()
        self._last_rc_ns = 0

        self._rep_body = (0.0, 0.0)
        self._nearest = -1.0
        self._coverage = 0.0
        self._lines = []                # per-line [{dist,nx,ny,strength}] body-FLU
        self._last_boundary_ns = 0
        self._corner = None
        self._last_corner_ns = 0
        self._corner_logged = False
        self._rows = 0

        self._hold_alt = 0.0            # altitude ref while filtering
        self._intervene_start = None
        self._offboard_seen = False
        self._clear_since = None
        self._cooldown_until = None
        self._interventions = 0
        self._clamped = False           # toward-component limited this tick
        self._last_print = 0.0

        # ── ROS wiring ───────────────────────────────────────────
        cb = ReentrantCallbackGroup()
        self.create_subscription(State, "/mavros/state",
                                 self._cb_state, qos_reliable(),
                                 callback_group=cb)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._cb_pose, qos_best_effort(),
                                 callback_group=cb)
        self.create_subscription(RCIn, "/mavros/rc/in",
                                 self._cb_rc, qos_best_effort(),
                                 callback_group=cb)
        self.create_subscription(Vector3Stamped, gp("repulsion_topic"),
                                 self._cb_rep, 10, callback_group=cb)
        self.create_subscription(Float32, gp("nearest_topic"),
                                 self._cb_near, 10, callback_group=cb)
        self.create_subscription(Float32, gp("coverage_topic"),
                                 self._cb_cov, 10, callback_group=cb)
        self.create_subscription(Vector3Stamped, gp("corner_topic"),
                                 self._cb_corner, 10, callback_group=cb)
        self.create_subscription(Float32MultiArray, gp("lines_topic"),
                                 self._cb_lines, 10, callback_group=cb)

        self._sp_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)
        self._vel_pub = self.create_publisher(
            TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10)
        self._mode_cli = self.create_client(SetMode, "/mavros/set_mode",
                                            callback_group=cb)

        self.create_timer(1.0 / self._sp_rate, self._tick, callback_group=cb)

        self.get_logger().info(
            "BoundaryGuard up — STICK FILTER: your sticks pass straight "
            "through; ONLY the toward-line component is clamped "
            f"(zero at {self._hold_dist:.2f} m). No pushback, no auto "
            "motion, no frozen transmitter.\n"
            "GROUND CHECK FIRST: move sticks and verify the STICK column "
            "(fwd+, left+) — flip rc_pitch_sign / rc_roll_sign if wrong.\n"
            "CH5 HIGH disables the guard. Approach the line ≤0.5 m/s.")
        self.get_logger().warn(
            ">>> GUARD BUILD = WALL-ONLY + distance smoothing. If you do NOT "
            "see THIS line at startup, the new code is NOT running — rebuild "
            "& re-source on the Jetson.")

    # ── Callbacks ────────────────────────────────────────────────
    def _cb_state(self, m): self._state = m
    def _cb_pose(self, m):  self._pose = m

    def _cb_rc(self, m):
        self._rc = m.channels
        self._last_rc_ns = self.get_clock().now().nanoseconds

    def _cb_rep(self, m: Vector3Stamped):
        self._rep_body = (m.vector.x, m.vector.y)
        self._last_boundary_ns = self.get_clock().now().nanoseconds

    def _cb_near(self, m: Float32):
        v = float(m.data)
        # Light low-pass. The FRONT line (head-on + downward camera) reads
        # jumpy; the raw value made the toward-clamp flicker so the drone
        # couldn't sit still at the wall. Smoothing steadies the hold. ~1-frame
        # lag only; a genuinely closer line still shows within a couple frames.
        if v >= 0.0 and self._nearest >= 0.0:
            self._nearest = 0.6 * v + 0.4 * self._nearest
        else:
            self._nearest = v

    def _cb_cov(self, m: Float32):
        self._coverage = m.data

    def _cb_lines(self, m: Float32MultiArray):
        """Decode [n, (dist, nx, ny, strength) × n] — body-FLU per-line
        vectors. Also refreshes the boundary-alive timestamp."""
        d = list(m.data)
        lines = []
        if d:
            n = int(d[0])
            for i in range(n):
                b = 1 + 4 * i
                if b + 3 < len(d):
                    lines.append({'dist': d[b], 'nx': d[b + 1],
                                  'ny': d[b + 2], 'strength': d[b + 3]})
        # Light low-pass on each line's distance (same reason as _cb_near) —
        # only while the line count is stable, so a steady hold reads steady
        # and the per-line clamp doesn't chatter on the jumpy front line.
        if len(lines) == len(self._lines):
            for i, L in enumerate(lines):
                L['dist'] = 0.6 * L['dist'] + 0.4 * self._lines[i]['dist']
        self._lines = lines
        self._last_boundary_ns = self.get_clock().now().nanoseconds

    def _cb_corner(self, m: Vector3Stamped):
        if m.vector.z > 0.5:
            self._corner = (m.vector.x, m.vector.y)
            self._last_corner_ns = self.get_clock().now().nanoseconds
            if not self._corner_logged:
                self.get_logger().warn(
                    f"L-CORNER at body ({m.vector.x:+.2f}, "
                    f"{m.vector.y:+.2f}) m — BOTH arms clamped by the "
                    "potential field; all other directions stay free.")
                self._corner_logged = True

    # ── RC stick reading ─────────────────────────────────────────
    def _stick(self, idx, sign=1.0):
        """One channel → normalised [-1, 1] with deadzone."""
        if len(self._rc) <= idx:
            return 0.0
        d = float(self._rc[idx]) - self._rc_center
        if abs(d) < self._rc_dz:
            return 0.0
        return sign * max(-1.0, min(1.0, d / self._rc_half))

    def _stick_body_cmd(self):
        """Sticks → desired BODY-frame velocities (like POSCTL).
        Returns (vx_fwd, vy_left, vz_up, yaw_rate)."""
        vx = self._stick(self._i_pitch, self._s_pitch) * self._vmax_xy
        vy = -self._stick(self._i_roll, self._s_roll) * self._vmax_xy
        vz = self._stick(self._i_thr) * self._vmax_z
        yr = -self._stick(self._i_yaw, self._s_yaw) * self._yawrate_max
        return vx, vy, vz, yr

    def _rc_fresh(self):
        if self._last_rc_ns == 0:
            return False
        return (self.get_clock().now().nanoseconds
                - self._last_rc_ns) / 1e9 <= self._rc_stale_s

    # ── Helpers ──────────────────────────────────────────────────
    def _ch5(self):
        return int(self._rc[self._rc_ch5]) \
            if len(self._rc) > self._rc_ch5 else 1500

    def _boundary_age(self):
        if self._last_boundary_ns == 0:
            return float("inf")
        return (self.get_clock().now().nanoseconds
                - self._last_boundary_ns) / 1e9

    def _corner_fresh(self):
        fresh = (self._last_corner_ns != 0 and
                 (self.get_clock().now().nanoseconds
                  - self._last_corner_ns) / 1e9 <= self._corner_stale_s)
        if not fresh:
            self._corner_logged = False
        return fresh

    def _yaw(self):
        q = self._pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _rep_enu(self):
        rx, ry = self._rep_body
        yaw = self._yaw()
        return (rx * math.cos(yaw) - ry * math.sin(yaw),
                rx * math.sin(yaw) + ry * math.cos(yaw))

    def _lines_enu(self):
        """Per-line list as (dist_m, ux, uy) with (ux, uy) a UNIT push-away
        vector in ENU (away from that line). At a corner this returns BOTH
        lines, so the clamp can hold the drone clear of each independently.
        Falls back to a single line from the blended repulsion when the
        detector isn't publishing per-line data (older detector)."""
        yaw = self._yaw()
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

    def _set_mode(self, mode):
        if self._mode_cli.service_is_ready():
            r = SetMode.Request()
            r.custom_mode = mode
            self._mode_cli.call_async(r)
        else:
            self.get_logger().error(f"set_mode not ready ({mode})")

    def _pub_pose_sp(self):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "local_origin"
        m.pose = self._pose.pose
        self._sp_pub.publish(m)

    def _pub_vel(self, vx, vy, vz, yr):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "local_origin"
        m.twist.linear.x = float(vx)
        m.twist.linear.y = float(vy)
        m.twist.linear.z = float(vz)
        m.twist.angular.z = float(yr)
        self._vel_pub.publish(m)

    def _set_gstate(self, s):
        if s != self._gstate:
            self.get_logger().info(f"── Guard: {self._gstate} → {s} ──")
            self._gstate = s

    def _table(self, flag=""):
        now = time.monotonic()
        if now - self._last_print < 1.0:
            return
        self._last_print = now
        if self._rows % 15 == 0:
            print(f"\n  {'GUARD':<9} {'MODE':<10} {'ALT':>5}  "
                  f"{'NEAR(m)':>7}  {'YEL%':>5}  {'WEIGHT':>6}  "
                  f"{'STICK x,y':>12}  {'INT#':>4}  FLAG")
            print("  " + "─" * 84)
        self._rows += 1
        near_txt = (f"{self._nearest:>7.2f}" if self._nearest >= 0
                    else f"{'—':>7}")
        wt = math.hypot(self._rep_body[0], self._rep_body[1])
        sx, sy, _, _ = self._stick_body_cmd()
        if len(self._lines) >= 2:
            flag = "2L " + flag
        if self._corner_fresh():
            flag = "C " + flag
        print(f"  {self._gstate:<9} {self._state.mode:<10} "
              f"{self._pose.pose.position.z:>5.2f}  {near_txt}  "
              f"{self._coverage:>4.1f}%  {wt:>6.2f}  "
              f"{sx:>+5.2f},{sy:>+5.2f}   {self._interventions:>4d}  {flag}")

    # ── Main loop ────────────────────────────────────────────────
    def _tick(self):
        alt = self._pose.pose.position.z
        ch5_kill = self._ch5() >= self._rc_high

        # Prime the OFFBOARD *velocity* stream BEFORE we ever engage.
        # Root cause of the "OFFBOARD not granted in 1.5 s" abort + mode
        # thrash seen in flight: WATCH pre-streamed POSITION setpoints, then
        # FILTER switched to VELOCITY setpoints — that setpoint-TYPE flip
        # makes PX4 refuse OFFBOARD. Fix: while armed near a line (but not
        # yet filtering) publish zero-velocity setpoints of the SAME type
        # we'll use when engaged. PX4 IGNORES them in POSCTL (pilot keeps
        # full control, drone does NOT move), but they give PX4 a consistent
        # setpoint history so OFFBOARD is granted the instant we engage.
        if self._state.armed and self._gstate != GuardState.FILTER:
            if 0.0 <= self._nearest <= self._govern_dist * 1.5:
                self._pub_vel(0.0, 0.0, 0.0, 0.0)   # prime (no-op in POSCTL)
            else:
                self._pub_pose_sp()                 # far from any line

        if self._gstate == GuardState.FILTER:
            self._do_filter(ch5_kill)
            return

        if (not self._state.armed) or alt < self._min_alt or ch5_kill:
            reason = ("CH5 HIGH" if ch5_kill else
                      "disarmed" if not self._state.armed else
                      f"alt<{self._min_alt:.1f}m")
            self._table(f"OFF ({reason})")
            self._set_gstate(GuardState.DISABLED)
            return

        if self._boundary_age() > self._stale_s:
            self.get_logger().error(
                "⚠⚠ GUARD IS BLIND — no boundary data. NO PROTECTION. "
                "DO NOT FLY TOWARD THE LINE.", throttle_duration_sec=2.0)
            self._table("BLIND — detector silent, NO protection!")
            self._set_gstate(GuardState.DISABLED)
            return

        # ── Cooldown (PANIC bypasses) ────────────────────────────
        if self._gstate == GuardState.COOLDOWN:
            panic = 0.0 <= self._nearest <= self._panic_dist
            if not panic and \
                    self.get_clock().now().nanoseconds < self._cooldown_until:
                return
            if panic:
                self.get_logger().warn(
                    f"PANIC — line at {self._nearest:.2f} m in cooldown")
            self._set_gstate(GuardState.WATCH)

        if self._gstate in (GuardState.DISABLED, GuardState.COOLDOWN):
            self._set_gstate(GuardState.WATCH)

        # ── WATCH: engage only when pilot pushes toward a line ─
        self._table(f"watching (clamp<{self._govern_dist:.1f}m, "
                    f"wall@{self._hold_dist:.2f}m)")

        if self._nearest < 0.0 or self._state.mode != self._pos_mode:
            return

        vx, vy, _, _ = self._stick_body_cmd()
        yaw = self._yaw()
        cvx = vx * math.cos(yaw) - vy * math.sin(yaw)   # cmd ENU
        cvy = vx * math.sin(yaw) + vy * math.cos(yaw)

        # Engage if the pilot pushes toward ANY nearby line (corner-aware),
        # or any line is already within the hold distance.
        pushing = too_close = False
        for (dist, ux, uy) in self._lines_enu():
            cmd_toward = -(cvx * ux + cvy * uy)         # cmd speed toward line
            if dist <= self._govern_dist and cmd_toward > self._engage_cmd:
                pushing = True
            if dist <= self._hold_dist + 0.05:
                too_close = True
        if pushing or too_close:
            if not self._rc_fresh():
                return                      # never filter without sticks
            self._interventions += 1
            self._hold_alt = alt
            self._intervene_start = self.get_clock().now()
            self._offboard_seen = False
            self._clear_since = None
            self.get_logger().warn(
                f"⚠ YELLOW LINE {self._nearest:.2f} m — STICK FILTER ON "
                f"(#{self._interventions}). You keep full control; only "
                f"motion toward the line is limited (wall at "
                f"{self._hold_dist:.2f} m).")
            self._set_mode("OFFBOARD")
            self._set_gstate(GuardState.FILTER)

    # ── FILTER: sticks pass through, toward-line clamped ─────────
    def _do_filter(self, ch5_kill):
        now = self.get_clock().now()
        el = (now - self._intervene_start).nanoseconds / 1e9

        if ch5_kill or not self._rc_fresh():
            why = "CH5 HIGH" if ch5_kill else "RC frames stale"
            self.get_logger().warn(
                f"{why} — releasing to {self._pos_mode}, guard "
                f"{'OFF' if ch5_kill else 'cooldown'}")
            self._set_mode(self._pos_mode)
            self._begin_cooldown()
            return

        if self._state.mode == "OFFBOARD":
            self._offboard_seen = True
        elif self._offboard_seen:
            self.get_logger().warn(
                f"Mode changed to {self._state.mode} externally — yields")
            self._begin_cooldown()
            return
        elif el > self._offboard_grant_s:
            self.get_logger().error(
                f"OFFBOARD not granted in {self._offboard_grant_s:.1f} s "
                "— aborting to POSCTL (pilot has control)")
            self._set_mode(self._pos_mode)
            self._begin_cooldown()
            return
        else:
            self._set_mode("OFFBOARD")

        # ── Sticks → ENU velocity command ────────────────────────
        vx, vy, vz, yr = self._stick_body_cmd()
        yaw = self._yaw()
        cvx = vx * math.cos(yaw) - vy * math.sin(yaw)
        cvy = vx * math.sin(yaw) + vy * math.cos(yaw)

        # NO automatic altitude motion. The throttle stick passes straight
        # through as vertical velocity; centred throttle → vz = 0, which is
        # a zero-velocity command (the drone simply holds its height, it does
        # not climb or descend on its own). The old P-term that actively flew
        # the drone back to a saved altitude is removed — that was autonomous
        # motion the pilot did not command.

        # ── THE CLAMP: limit the toward-line speed of EACH line ──
        # Every detected line clamps its OWN toward-component to a limit
        # that tapers smoothly to ZERO at hold_dist. At a corner BOTH lines
        # clamp independently, so pushing into the corner stops you off both
        # — you can rest in the corner. Sideways / backward / up / down / yaw
        # stay free, so you always pull back out easily on the sticks.
        self._clamped = False
        for (dist, ux, uy) in self._lines_enu():
            toward = -(cvx * ux + cvy * uy)          # cmd speed toward line
            allowed = max(0.0, min(
                self._appr_vmax,
                self._gain * (dist - self._hold_dist)))
            if toward > allowed:
                excess = toward - allowed
                cvx += ux * excess                   # strip the excess
                cvy += uy * excess
                self._clamped = True

        self._pub_vel(cvx, cvy, vz, yr)

        self._table("FILTER "
                    + ("CLAMPED at wall" if self._clamped else "free")
                    + f"  t={el:.0f}s")

        # ── Release: line far away or lost ───────────────────────
        clear = (self._nearest < 0.0) or (self._nearest > self._release_dist)
        if clear:
            if self._clear_since is None:
                self._clear_since = now
            elif (now - self._clear_since).nanoseconds / 1e9 \
                    >= self._release_hold:
                self.get_logger().info(
                    f"Line clear — {self._pos_mode} returned ✓")
                self._set_mode(self._pos_mode)
                self._begin_cooldown()
                return
        else:
            self._clear_since = None

        if self._max_intervene > 0.0 and el > self._max_intervene:
            self.get_logger().warn(
                f"Filter timeout ({self._max_intervene:.0f} s) — "
                f"returning {self._pos_mode}")
            self._set_mode(self._pos_mode)
            self._begin_cooldown()

    def _begin_cooldown(self):
        self._cooldown_until = self.get_clock().now().nanoseconds \
            + int(self._cooldown_s * 1e9)
        self._set_gstate(GuardState.COOLDOWN)


def main():
    rclpy.init()
    node = BoundaryGuard()
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
