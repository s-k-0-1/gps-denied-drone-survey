#!/usr/bin/env python3
"""
whycode_mission — Autonomous WhyCon/WhyCode centre-align-probe-land.
Team Viman Rakshak / IRoC-U 2026.

Target: a WhyCon/WhyCode fiducial (41 cm outer diameter) lying flat on the
ground.  The downward-facing D455 sees it; a separate **whycon ROS node**
detects it and publishes its pose.  This node consumes that pose.

What it does
────────────
  IDLE     wait for RC CH5 LOW to arm
  ARM      OFFBOARD + arm; lock home X/Y and arm yaw
  TAKEOFF  climb to target_alt holding home
  SEARCH   hover (slow yaw) until the marker is seen for detect_frames
  CENTER   fly laterally until directly above the marker
  YAW_ALIGN rotate so the drone faces "forward" per the marker's whycon
           orientation (drives the marker's in-image angle to ~0)
  FORWARD  move 1 m along the drone's (now marker-aligned) heading
  BACK     return 1 m to the marker centre
  DESCEND  re-centre on the marker live, sink, hand off to AUTO.LAND
  LAND     AUTO.LAND owns touchdown + disarm
  DONE

Detection input (from the whycon node)
──────────────────────────────────────
  Subscribes to `whycon_topic` (default /whycon/poses) of type
  geometry_msgs/PoseArray.  Each pose is the marker in the CAMERA OPTICAL
  frame: position x=right, y=down, z=depth(toward ground); orientation =
  marker rotation (WhyCode).  The whycon node MUST be configured with the
  41 cm marker diameter and a calibrated camera so its poses are metric.

  If your whycon build publishes on a different topic/type, set
  `whycon_topic`.  If it publishes NO orientation (identity quaternion),
  YAW_ALIGN is skipped automatically and the drone keeps its takeoff
  heading (a warning is logged).

Tuning (first flight)
─────────────────────
  cam_x_sign / cam_y_sign  flip if the drone CENTERs the wrong way.
  whycon_yaw_sign          flip if the drone yaws AWAY from aligned.
  whycon_yaw_offset_deg    constant bias between marker "forward" and
                           the drone's nose (camera mounting).

Safety
──────
  RC CH5 >= rc_interrupt_high at any time → STABILIZED (pilot takeover).
  Ctrl+C / marker lost handling per phase (see code).

Run (whycon node must already be publishing):
  ros2 run viman_mission whycode_mission
"""

import math

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import RCIn, State
from mavros_msgs.srv import CommandBool, SetMode

from whycode_interfaces.msg import MarkerArray

from viman_mission.common import qos_best_effort, qos_reliable


class Phase:
    IDLE      = "IDLE"
    ARM       = "ARM"
    TAKEOFF   = "TAKEOFF"
    SEARCH    = "SEARCH"
    CENTER    = "CENTER"
    YAW_ALIGN = "YAW_ALIGN"
    FORWARD   = "FORWARD"
    BACK      = "BACK"
    DESCEND   = "DESCEND"
    LAND      = "LAND"
    SAFE      = "SAFE"
    DONE      = "DONE"


def _quat_yaw(qx, qy, qz, qw) -> float:
    """Yaw (rad) about Z from a quaternion (ZYX convention)."""
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def _wrap(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class WhycodeMission(Node):

    def __init__(self):
        super().__init__("whycode_mission")

        self.declare_parameters("", [
            # Flight profile
            ("target_alt",          2.0),    # work altitude over the marker (m)
            ("alt_tolerance",       0.12),
            ("at_alt_confirm_s",    1.5),
            ("approach_speed_ms",   0.3),    # lateral speed, CENTER/BACK
            ("forward_speed_ms",    0.3),    # speed for the 1 m probe leg
            ("descend_speed_ms",    0.15),
            ("land_handoff_alt_m",  0.3),
            ("center_thr_m",        0.12),   # within this radius = "centred"
            ("leg_radius_m",        0.15),   # arrival radius for FORWARD/BACK
            ("leg_settle_s",        1.5),    # hold at forward point / back
            ("sp_rate_hz",          20.0),
            # RC
            ("rc_ch5_index",        4),
            ("rc_start_low",        1200),
            ("rc_interrupt_high",   1700),
            # Detection (whycode node)
            ("markers_topic",       "/whycode_node/markers"),
            ("marker_diameter_m",   0.41),   # informational; set in the whycode node too
            ("detect_frames",       5),      # consecutive fresh detections to confirm
            ("marker_timeout_s",    2.0),    # pose older than this = "lost" (bridges gaps)
            ("alt_gate_m",          1.5),    # reject a marker whose camera-depth differs
                                             # from drone altitude by more than this
                                             # (kills false detections); <=0 disables
            ("outlier_jump_m",      1.0),    # reject a lone detection that jumps more than
                                             # this (camera-frame m) from the last accepted
            # Geometry / mounting signs
            ("cam_x_sign",          1.0),
            ("cam_y_sign",         -1.0),
            ("forward_dist_m",      1.0),    # length of the probe leg
            # Yaw alignment
            ("yaw_align_tol_deg",   5.0),
            ("yaw_align_hold_s",    1.0),
            ("yaw_slew_dps",        20.0),
            ("yaw_align_timeout_s", 25.0),
            ("whycon_yaw_sign",     1.0),
            ("whycon_yaw_offset_deg", 0.0),
        ])

        def gp(n): return self.get_parameter(n).value

        self._target_alt    = gp("target_alt")
        self._alt_tol       = gp("alt_tolerance")
        self._at_alt_conf   = gp("at_alt_confirm_s")
        self._appr_speed    = gp("approach_speed_ms")
        self._fwd_speed     = gp("forward_speed_ms")
        self._desc_speed    = gp("descend_speed_ms")
        self._land_alt      = gp("land_handoff_alt_m")
        self._center_thr    = gp("center_thr_m")
        self._leg_radius    = gp("leg_radius_m")
        self._leg_settle    = gp("leg_settle_s")
        self._sp_rate       = gp("sp_rate_hz")
        self._rc_ch5        = int(gp("rc_ch5_index"))
        self._rc_low        = int(gp("rc_start_low"))
        self._rc_high       = int(gp("rc_interrupt_high"))
        self._markers_topic = gp("markers_topic")
        self._detect_frames = int(gp("detect_frames"))
        self._marker_tmo    = gp("marker_timeout_s")
        self._alt_gate      = float(gp("alt_gate_m"))
        self._outlier_jump  = float(gp("outlier_jump_m"))
        self._cam_x_sign    = float(gp("cam_x_sign"))
        self._cam_y_sign    = float(gp("cam_y_sign"))
        self._fwd_dist      = float(gp("forward_dist_m"))
        self._yaw_tol       = math.radians(gp("yaw_align_tol_deg"))
        self._yaw_hold      = gp("yaw_align_hold_s")
        self._yaw_slew      = math.radians(gp("yaw_slew_dps"))
        self._yaw_tmo       = gp("yaw_align_timeout_s")
        self._yaw_sign      = float(gp("whycon_yaw_sign"))
        self._yaw_offset    = math.radians(gp("whycon_yaw_offset_deg"))

        # ── State ────────────────────────────────────────────────
        self._phase   = Phase.IDLE
        self._last_ph = None
        self._state   = State()
        self._pose    = PoseStamped()
        self._pose.pose.orientation.w = 1.0
        self._rc      = ()

        self._home_x = 0.0
        self._home_y = 0.0
        self._arm_yaw = 0.0
        self._cmd_yaw = 0.0           # slewed commanded yaw (rad)

        # Latest marker, remapped to camera convention (x=right, y=down, z=depth)
        self._mk_x = 0.0             # right  (= -whycode.position.y)
        self._mk_y = 0.0             # down   (= -whycode.position.z)
        self._mk_z = 0.0             # depth  (=  whycode.position.x) ≈ altitude
        self._mk_cam_yaw = 0.0       # marker yaw from WhyCode (rotation.z, rad)
        self._mk_has_orient = False
        self._mk_stamp_ns = 0
        self._mk_last_xy = None      # last accepted (x,y) for outlier rejection

        # Targets locked over the marker
        self._marker_ex = 0.0        # ENU position directly above marker
        self._marker_ey = 0.0
        self._fwd_ex = 0.0
        self._fwd_ey = 0.0

        # Timers / counters
        self._arm_req      = False
        self._detect_count = 0
        self._at_alt_since = None
        self._yaw_ok_since = None
        self._yaw_start    = None
        self._leg_since    = None
        self._desc_z       = None

        # ── ROS wiring ────────────────────────────────────────────
        cbg = ReentrantCallbackGroup()
        self.create_subscription(State, "/mavros/state",
            self._cb_state, qos_reliable(), callback_group=cbg)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
            self._cb_pose, qos_best_effort(), callback_group=cbg)
        self.create_subscription(RCIn, "/mavros/rc/in",
            self._cb_rc, qos_best_effort(), callback_group=cbg)
        self.create_subscription(MarkerArray, self._markers_topic,
            self._cb_markers, qos_best_effort(), callback_group=cbg)

        self._sp_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", qos_reliable())
        self._arm_cli  = self.create_client(
            CommandBool, "/mavros/cmd/arming", callback_group=cbg)
        self._mode_cli = self.create_client(
            SetMode, "/mavros/set_mode", callback_group=cbg)

        self.create_timer(1.0 / self._sp_rate, self._tick, callback_group=cbg)
        self.get_logger().info(
            f"WhycodeMission ready — marker {gp('marker_diameter_m'):.2f} m | "
            f"markers topic '{self._markers_topic}' | "
            f"alt {self._target_alt:.1f} m | probe {self._fwd_dist:.1f} m\n"
            "Lower RC CH5 to arm and begin.")

    # ── Callbacks ────────────────────────────────────────────────

    def _cb_state(self, m): self._state = m
    def _cb_pose(self, m):  self._pose = m

    def _cb_rc(self, m):
        self._rc = m.channels
        if len(self._rc) <= self._rc_ch5:
            return
        ch5 = self._rc[self._rc_ch5]
        if ch5 <= self._rc_low and self._phase == Phase.IDLE:
            self._arm_req = True
        if ch5 >= self._rc_high and self._phase not in (
                Phase.LAND, Phase.DONE, Phase.SAFE, Phase.IDLE):
            self.get_logger().warn(f"RC INTERRUPT CH5={ch5} → STABILIZED")
            self._set_mode("STABILIZED")
            self._phase = Phase.SAFE

    def _cb_markers(self, m: MarkerArray):
        if not m.markers:
            return
        # WhyCode camera frame: position.x = depth (along optical axis),
        # y = left, z = up. Remap to the mission's camera convention
        # (x = right, y = down, z = depth) so the rest of the node is unchanged.
        # Pick the marker nearest the image centre (smallest lateral offset).
        best = min(m.markers,
                   key=lambda mk: abs(mk.position.position.y)
                                + abs(mk.position.position.z))
        depth =  best.position.position.x
        mx    = -best.position.position.y    # camera right
        my    = -best.position.position.z    # camera down

        # ── Safety gate 1: depth must match the drone's altitude ──────
        # For a downward camera over a ground marker, camera-depth ≈ the
        # drone's height. A detection far off that is a false positive.
        alt = self._pose.pose.position.z
        if self._alt_gate > 0.0 and alt > 0.3 \
                and abs(depth - alt) > self._alt_gate:
            self.get_logger().warn(
                f"marker depth {depth:.2f} m vs altitude {alt:.2f} m — "
                "rejected (alt gate)", throttle_duration_sec=2.0)
            return

        # ── Safety gate 2: reject a lone outlier jump ─────────────────
        # (skipped after the marker has gone stale, so re-acquisition at a
        #  new position is allowed.)
        if self._mk_last_xy is not None and self._marker_fresh():
            jump = math.hypot(mx - self._mk_last_xy[0], my - self._mk_last_xy[1])
            if jump > self._outlier_jump:
                self.get_logger().warn(
                    f"marker jumped {jump:.2f} m in one frame — rejected (outlier)",
                    throttle_duration_sec=2.0)
                return

        self._mk_x = mx
        self._mk_y = my
        self._mk_z = depth
        self._mk_cam_yaw = best.rotation.z      # real WhyCode yaw [rad]
        self._mk_has_orient = True
        self._mk_last_xy = (mx, my)
        st = m.header.stamp
        self._mk_stamp_ns = (st.sec * 1_000_000_000 + st.nanosec) \
            if (st.sec or st.nanosec) else self.get_clock().now().nanoseconds

    # ── Helpers ──────────────────────────────────────────────────

    def _set_mode(self, mode):
        r = SetMode.Request()
        r.custom_mode = mode
        self._mode_cli.call_async(r)

    def _yaw(self) -> float:
        q = self._pose.pose.orientation
        return _quat_yaw(q.x, q.y, q.z, q.w)

    def _marker_fresh(self) -> bool:
        if self._mk_stamp_ns == 0:
            return False
        age = (self.get_clock().now().nanoseconds - self._mk_stamp_ns) / 1e9
        return age <= self._marker_tmo

    def _marker_enu_offset(self):
        """ENU offset FROM drone TO the marker (dx, dy), or None if stale."""
        if not self._marker_fresh():
            return None
        tx = self._mk_x * self._cam_x_sign
        ty = self._mk_y * self._cam_y_sign
        yaw = self._yaw()
        dx = math.cos(yaw) * tx - math.sin(yaw) * ty
        dy = math.sin(yaw) * tx + math.cos(yaw) * ty
        return dx, dy

    def _pub_sp(self, x, y, z, yaw_rad):
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.x = float(x)
        sp.pose.position.y = float(y)
        sp.pose.position.z = float(z)
        sp.pose.orientation.z = math.sin(yaw_rad / 2.0)
        sp.pose.orientation.w = math.cos(yaw_rad / 2.0)
        self._sp_pub.publish(sp)

    def _slew_yaw(self, target_yaw):
        """Step _cmd_yaw toward target at yaw_slew rate; return new _cmd_yaw."""
        err = _wrap(target_yaw - self._cmd_yaw)
        step = self._yaw_slew / self._sp_rate
        self._cmd_yaw = _wrap(self._cmd_yaw + max(-step, min(step, err)))
        return self._cmd_yaw

    def _step_to(self, tx, ty, z, yaw, speed):
        """One speed-limited step toward (tx,ty,z). Returns distance to target."""
        px, py = self._pose.pose.position.x, self._pose.pose.position.y
        dx, dy = tx - px, ty - py
        dist = math.hypot(dx, dy)
        step = speed / self._sp_rate
        if dist > step:
            nx, ny = px + step * dx / dist, py + step * dy / dist
        else:
            nx, ny = tx, ty
        self._pub_sp(nx, ny, z, yaw)
        return dist

    # ── State machine ────────────────────────────────────────────

    def _tick(self):
        if self._phase != self._last_ph:
            self.get_logger().info(f"══ Phase: {self._phase} ══")
            self._last_ph = self._phase
        getattr(self, "_do_" + self._phase.lower())()

    def _do_idle(self):
        self._pub_sp(self._home_x, self._home_y, self._target_alt, 0.0)
        if self._arm_req:
            self._arm_req = False
            self._set_mode("OFFBOARD")
            self._phase = Phase.ARM

    def _do_arm(self):
        self._pub_sp(self._home_x, self._home_y, self._target_alt, 0.0)
        if self._state.mode != "OFFBOARD":
            self._set_mode("OFFBOARD")
            return
        if not self._state.armed:
            r = CommandBool.Request(); r.value = True
            self._arm_cli.call_async(r)
            return
        self._home_x = self._pose.pose.position.x
        self._home_y = self._pose.pose.position.y
        self._arm_yaw = self._yaw()
        self._cmd_yaw = self._arm_yaw
        self.get_logger().info(
            f"Armed  home=({self._home_x:.2f},{self._home_y:.2f})  "
            f"yaw={math.degrees(self._arm_yaw):.1f}°")
        self._at_alt_since = None
        self._phase = Phase.TAKEOFF

    def _do_takeoff(self):
        self._pub_sp(self._home_x, self._home_y, self._target_alt, self._arm_yaw)
        alt = self._pose.pose.position.z
        if abs(alt - self._target_alt) <= self._alt_tol:
            if self._at_alt_since is None:
                self._at_alt_since = self.get_clock().now()
            elif self._secs(self._at_alt_since) >= self._at_alt_conf:
                self._detect_count = 0
                self._phase = Phase.SEARCH
        else:
            self._at_alt_since = None

    def _do_search(self):
        # Hold over home; do NOT yaw-spin (marker is large, camera sees it from
        # straight above). Just wait for stable detections.
        self._pub_sp(self._home_x, self._home_y, self._target_alt, self._cmd_yaw)
        off = self._marker_enu_offset()
        if off is None:
            self._detect_count = 0
            self.get_logger().info("SEARCH — no marker", throttle_duration_sec=1.0)
            return
        self._detect_count += 1
        dx, dy = off
        self.get_logger().info(
            f"SEARCH — marker offset=({dx:+.2f},{dy:+.2f})m "
            f"frame {self._detect_count}/{self._detect_frames}",
            throttle_duration_sec=0.5)
        if self._detect_count >= self._detect_frames:
            self._phase = Phase.CENTER

    def _do_center(self):
        off = self._marker_enu_offset()
        if off is not None:
            px, py = self._pose.pose.position.x, self._pose.pose.position.y
            self._marker_ex = px + off[0]
            self._marker_ey = py + off[1]
        dist = self._step_to(self._marker_ex, self._marker_ey,
                             self._target_alt, self._cmd_yaw, self._appr_speed)
        self.get_logger().info(f"CENTER dist={dist:.2f} m", throttle_duration_sec=0.5)
        if dist <= self._center_thr:
            self.get_logger().info("Centred — YAW_ALIGN")
            self._yaw_start = self.get_clock().now()
            self._yaw_ok_since = None
            self._phase = Phase.YAW_ALIGN

    def _do_yaw_align(self):
        # Keep re-centring over the marker while rotating.
        off = self._marker_enu_offset()
        if off is not None:
            px, py = self._pose.pose.position.x, self._pose.pose.position.y
            self._marker_ex = px + off[0]
            self._marker_ey = py + off[1]

        # No orientation from whycon → cannot align; keep heading, move on.
        if not self._mk_has_orient:
            self.get_logger().warn(
                "whycon publishes no orientation — skipping YAW_ALIGN, "
                "keeping current heading.")
            self._begin_forward()
            return

        # Drive the marker's in-image angle to 0 (drone faces marker-forward).
        # target_yaw = current_drone_yaw - sign*(marker_cam_yaw) + offset
        err = self._yaw_sign * self._mk_cam_yaw - self._yaw_offset
        target_yaw = _wrap(self._yaw() - err)
        cmd = self._slew_yaw(target_yaw)
        self._pub_sp(self._marker_ex, self._marker_ey, self._target_alt, cmd)

        self.get_logger().info(
            f"YAW_ALIGN err={math.degrees(err):+.1f}° "
            f"(tol {math.degrees(self._yaw_tol):.0f}°)",
            throttle_duration_sec=0.5)

        if abs(err) <= self._yaw_tol:
            if self._yaw_ok_since is None:
                self._yaw_ok_since = self.get_clock().now()
            elif self._secs(self._yaw_ok_since) >= self._yaw_hold:
                self.get_logger().info("Yaw aligned — FORWARD")
                self._begin_forward()
        else:
            self._yaw_ok_since = None
            if self._secs(self._yaw_start) > self._yaw_tmo:
                self.get_logger().warn(
                    "YAW_ALIGN timeout — proceeding with current heading")
                self._begin_forward()

    def _begin_forward(self):
        # Lock the marker-centre ENU and compute the 1 m forward target along
        # the (now aligned) commanded heading.
        self._marker_ex = self._pose.pose.position.x \
            if self._marker_enu_offset() is None else self._marker_ex
        self._marker_ey = self._pose.pose.position.y \
            if self._marker_enu_offset() is None else self._marker_ey
        yaw = self._cmd_yaw
        self._fwd_ex = self._marker_ex + self._fwd_dist * math.cos(yaw)
        self._fwd_ey = self._marker_ey + self._fwd_dist * math.sin(yaw)
        self._leg_since = None
        self.get_logger().info(
            f"Probe leg: marker=({self._marker_ex:.2f},{self._marker_ey:.2f}) "
            f"→ fwd=({self._fwd_ex:.2f},{self._fwd_ey:.2f})")
        self._phase = Phase.FORWARD

    def _do_forward(self):
        dist = self._step_to(self._fwd_ex, self._fwd_ey,
                             self._target_alt, self._cmd_yaw, self._fwd_speed)
        self.get_logger().info(f"FORWARD dist={dist:.2f} m", throttle_duration_sec=0.5)
        if dist <= self._leg_radius:
            if self._leg_since is None:
                self._leg_since = self.get_clock().now()
            elif self._secs(self._leg_since) >= self._leg_settle:
                self._leg_since = None
                self._phase = Phase.BACK

    def _do_back(self):
        dist = self._step_to(self._marker_ex, self._marker_ey,
                             self._target_alt, self._cmd_yaw, self._fwd_speed)
        self.get_logger().info(f"BACK dist={dist:.2f} m", throttle_duration_sec=0.5)
        if dist <= self._leg_radius:
            if self._leg_since is None:
                self._leg_since = self.get_clock().now()
            elif self._secs(self._leg_since) >= self._leg_settle:
                self._leg_since = None
                self._desc_z = self._target_alt
                self._phase = Phase.DESCEND

    def _do_descend(self):
        # Re-centre on the marker live (if visible), sink, handoff near ground.
        off = self._marker_enu_offset()
        if off is not None:
            px, py = self._pose.pose.position.x, self._pose.pose.position.y
            self._marker_ex = px + off[0]
            self._marker_ey = py + off[1]
        self._desc_z = max(0.0, self._desc_z - self._desc_speed / self._sp_rate)

        px, py = self._pose.pose.position.x, self._pose.pose.position.y
        dx, dy = self._marker_ex - px, self._marker_ey - py
        dist = math.hypot(dx, dy)
        step = self._appr_speed / self._sp_rate
        if dist > step:
            nx, ny = px + step * dx / dist, py + step * dy / dist
        else:
            nx, ny = self._marker_ex, self._marker_ey
        self._pub_sp(nx, ny, self._desc_z, self._cmd_yaw)

        alt = self._pose.pose.position.z
        self.get_logger().info(
            f"DESCEND alt={alt:.2f} m  lat_err={dist:.2f} m",
            throttle_duration_sec=0.3)
        if alt <= self._land_alt:
            self.get_logger().info(f"Handoff to AUTO.LAND at {alt:.2f} m")
            self._set_mode("AUTO.LAND")
            self._phase = Phase.LAND

    def _do_land(self):
        if not self._state.armed:
            self.get_logger().info("Disarmed — mission complete ✓")
            self._phase = Phase.DONE

    def _do_safe(self):
        self.get_logger().info("SAFE — pilot has control. Restart to fly again.",
                               throttle_duration_sec=5.0)

    def _do_done(self):
        pass

    def _secs(self, t):
        return 0.0 if t is None else (self.get_clock().now() - t).nanoseconds * 1e-9


def main():
    rclpy.init()
    node = WhycodeMission()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
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
