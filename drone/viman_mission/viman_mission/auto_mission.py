#!/usr/bin/env python3
"""
auto_mission — Autonomous RTAB-Map VIO Mission node
Viman Rakshak / IRoC-U 2026

Mission phases (all in OFFBOARD — no X/Y drift):
  IDLE       : Wait for CH5 LOW (PWM ≤ rc_start_low) to start
  ARM        : Stream setpoints → OFFBOARD → Arm
               Home X/Y locked from MAVROS local ENU frame (same origin as QGC home)
  TAKEOFF    : Climb to target_alt holding home X/Y
  STABLE_OF  : Hold still on optical flow — camera warm-up window
  INIT_CAM   : Launch RTAB-Map + vision_bridge subprocess
  WAIT_VIO   : Poll /rtabmap/rtabmap/odom covariance until stable
               PX4 fuses optical-flow + vision automatically (no param change)
  HOVER      : Hold home X/Y at altitude for hover_duration; watch RTAB-Map health
  LAND       : AUTO.LAND
  DISARM     : Wait for PX4 to self-disarm after touchdown

Safety:
  CH5 ≥ rc_interrupt_high → STABILIZED, SAFE_MANUAL phase (pilot full control)
  Ctrl+C                  → AUTO.LAND immediately, then clean shutdown
  RTAB-Map lost in HOVER  → AUTO.LAND immediately

All tunables are ROS parameters (see config/mission_params.yaml).
Coordinate frame notes: see viman_mission/common.py.
Optical flow + camera fusion is handled by PX4's EKF2 automatically;
this node does NOT touch any EKF2 parameters.
"""

import os
import signal
import subprocess
import time
from datetime import datetime
from enum import Enum, auto

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import RCIn, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry

from viman_mission.common import (qos_best_effort, qos_reliable,
                                  rtab_cov_good, yaw_deg_from_quaternion)
from viman_mission.rtabmap_config import (build_rtabmap_cmd,
                                          build_vision_bridge_cmd)


class Phase(Enum):
    IDLE        = auto()
    ARM         = auto()
    TAKEOFF     = auto()
    STABLE_OF   = auto()
    INIT_CAM    = auto()
    WAIT_VIO    = auto()
    HOVER       = auto()
    LAND        = auto()
    DISARM      = auto()
    SAFE_MANUAL = auto()
    DONE        = auto()


class AutoMission(Node):

    def __init__(self):
        super().__init__("auto_mission")

        # ── Parameters (defaults = previous hard-coded constants) ──
        p = self.declare_parameters("", [
            ("target_alt",        2.0),    # hover altitude, ENU +Z (m)
            ("alt_tolerance",     0.12),   # ±m band for "at altitude"
            ("stable_of_secs",    3.0),    # hold-still time before RTAB-Map
            ("cam_good_secs",     5.0),    # continuous good tracking needed
            ("cam_timeout_secs",  30.0),   # abort to land if VIO not ready
            ("hover_duration",    30.0),   # hover time on RTAB-Map (s)
            ("sp_rate_hz",        20.0),   # setpoint publish rate
            ("rc_ch5_index",      4),      # channel 5 = index 4 (0-based)
            ("rc_start_low",      1200),   # CH5 PWM ≤ this → start mission
            ("rc_interrupt_high", 1700),   # CH5 PWM ≥ this → pilot takeover
            ("map_dir",           "/media/jetson/ROS2_SSD/maps"),
        ])
        (self._target_alt, self._alt_tol, self._stable_of_secs,
         self._cam_good_secs, self._cam_timeout_secs, self._hover_duration,
         self._sp_rate_hz, self._rc_ch5_idx, self._rc_start_low,
         self._rc_interrupt_high, self._map_dir) = (x.value for x in p)

        self._phase      = Phase.IDLE
        self._last_phase = None

        # Home position locked from MAVROS ENU frame on arm.
        # ENU origin = QGC home/arming point. X=East, Y=North, Z=Up.
        self._hold_x = 0.0
        self._hold_y = 0.0
        # Heading (yaw) locked at arm time — held for all setpoints so PX4
        # never receives a "rotate to face East" command. None = live pose.
        self._hold_heading_q = None

        # Live sensor data
        self._pose = PoseStamped()
        self._pose.pose.orientation.w = 1.0   # identity, not zero-quat
        self._state = State()
        self._vel   = TwistStamped()
        self._rc    = []
        self._rtab_cov = float("inf")   # /rtabmap/rtabmap/odom covariance[0]

        # Ctrl+C → AUTO.LAND
        self._ctrl_c = False
        signal.signal(signal.SIGINT, self._sigint_handler)

        # Telemetry print throttle
        self._last_print     = 0.0
        self._header_printed = False

        # Phase timestamps (rclpy.Time)
        self._at_alt_since    = None
        self._stable_since    = None
        self._cam_init_start  = None
        self._rtab_good_since = None
        self._hover_start     = None

        # Arm sequence flags
        self._offboard_requested = False
        self._arm_requested      = False

        # Subprocesses
        self._rtabmap_proc = None
        self._bridge_proc  = None

        cb = ReentrantCallbackGroup()
        qos_be, qos_rel = qos_best_effort(), qos_reliable()

        # ── Subscribers ──────────────────────────────────────────
        self.create_subscription(
            State, "/mavros/state",
            self._state_cb, qos_rel, callback_group=cb)
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose",
            self._pose_cb, qos_be, callback_group=cb)
        self.create_subscription(
            TwistStamped, "/mavros/local_position/velocity_local",
            self._vel_cb, qos_be, callback_group=cb)
        self.create_subscription(
            RCIn, "/mavros/rc/in",
            self._rc_cb, qos_be, callback_group=cb)
        self.create_subscription(
            Odometry, "/rtabmap/rtabmap/odom",
            self._rtab_cb, qos_rel, callback_group=cb)

        # ── Publishers ───────────────────────────────────────────
        self._sp_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)
        # Velocity publisher — used during optical-flow-only phases
        # (STABLE_OF, INIT_CAM, WAIT_VIO) so the drone holds via flow
        # velocity directly instead of fighting a drifting EKF position.
        self._vel_pub = self.create_publisher(
            TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10)

        # ── Service clients ──────────────────────────────────────
        self._arm_cli  = self.create_client(
            CommandBool, "/mavros/cmd/arming", callback_group=cb)
        self._mode_cli = self.create_client(
            SetMode, "/mavros/set_mode", callback_group=cb)

        # ── Phase dispatch table ─────────────────────────────────
        self._handlers = {
            Phase.IDLE:        self._do_idle,
            Phase.ARM:         self._do_arm,
            Phase.TAKEOFF:     self._do_takeoff,
            Phase.STABLE_OF:   self._do_stable_of,
            Phase.INIT_CAM:    self._do_init_cam,
            Phase.WAIT_VIO:    self._do_wait_vio,
            Phase.HOVER:       self._do_hover,
            Phase.LAND:        self._do_land,
            Phase.DISARM:      self._do_disarm,
            Phase.SAFE_MANUAL: self._do_safe_manual,
            Phase.DONE:        lambda: None,
        }

        # ── Main loop timer ──────────────────────────────────────
        self.create_timer(1.0 / self._sp_rate_hz, self._loop,
                          callback_group=cb)

        self.get_logger().info(
            "AutoMission ready. "
            f"Set CH5 ≤ {self._rc_start_low} PWM to start. "
            f"CH5 ≥ {self._rc_interrupt_high} PWM = safety takeover.")

    # ════════════════════════════════════════════════════════════
    #  CALLBACKS
    # ════════════════════════════════════════════════════════════

    def _state_cb(self, msg: State):
        self._state = msg

    def _pose_cb(self, msg: PoseStamped):
        self._pose = msg

    def _vel_cb(self, msg: TwistStamped):
        self._vel = msg

    def _rtab_cb(self, msg: Odometry):
        # cov[0] = x-x diagonal; small positive = good; 99999 = lost
        self._rtab_cov = msg.pose.covariance[0]

    def _sigint_handler(self, sig, frame):
        self.get_logger().warn("Ctrl+C — scheduling emergency AUTO.LAND")
        self._ctrl_c = True

    def _rc_cb(self, msg: RCIn):
        self._rc = list(msg.channels)

        # Safety interrupt — ignored where pilot already has control
        if self._phase in (Phase.SAFE_MANUAL, Phase.DONE, Phase.IDLE):
            return

        if self._ch5_pwm() >= self._rc_interrupt_high:
            self.get_logger().warn(
                f"⚠  RC INTERRUPT — CH5={self._ch5_pwm()} PWM. "
                "Switching to STABILIZED. Pilot has full control.")
            self._request_mode("STABILIZED")
            self._phase = Phase.SAFE_MANUAL

    # ════════════════════════════════════════════════════════════
    #  MAIN CONTROL LOOP  (sp_rate_hz)
    # ════════════════════════════════════════════════════════════

    def _loop(self):
        # Phase transition bookkeeping
        if self._phase != self._last_phase:
            self.get_logger().info(f"══ Phase: {self._phase.name} ══")
            self._last_phase     = self._phase
            self._header_printed = False

        # Ctrl+C guard — fires before any phase logic
        if self._ctrl_c and self._phase not in (
                Phase.LAND, Phase.DISARM, Phase.DONE, Phase.SAFE_MANUAL):
            self.get_logger().warn("Ctrl+C — emergency AUTO.LAND")
            self._phase = Phase.LAND
            return

        self._handlers[self._phase]()

    # ── Phase handlers ────────────────────────────────────────────

    def _do_idle(self):
        self._pub_sp(0.0, 0.0, 0.3)   # pre-stream so OFFBOARD is ready

        if not self._state.connected:
            self.get_logger().info(
                "Waiting for FCU...", throttle_duration_sec=5.0)
            return

        if self._ch5_pwm() <= self._rc_start_low:
            self.get_logger().info(
                f"CH5={self._ch5_pwm()} — start trigger received")
            self._phase = Phase.ARM

    def _do_arm(self):
        self._pub_sp(0.0, 0.0, 0.3)

        if not self._offboard_requested:
            self._request_mode("OFFBOARD")
            self._offboard_requested = True
            return

        if self._state.mode != "OFFBOARD":
            self._request_mode("OFFBOARD")   # keep requesting
            return

        if not self._arm_requested:
            self._request_arm(True)
            self._arm_requested = True
            return

        if self._state.armed and self._state.mode == "OFFBOARD":
            # Lock home X/Y and heading from current pose.
            # Heading MUST be locked so setpoints never command PX4 to
            # rotate to 0° (East) — that causes aggressive yaw spin.
            self._hold_x = self._pose.pose.position.x
            self._hold_y = self._pose.pose.position.y
            self._hold_heading_q = self._pose.pose.orientation
            yaw_deg = yaw_deg_from_quaternion(self._hold_heading_q)
            self.get_logger().info(
                f"Armed. Home locked — "
                f"x={self._hold_x:.3f} m (East)  "
                f"y={self._hold_y:.3f} m (North)  "
                f"yaw={yaw_deg:.1f}°")
            self._phase = Phase.TAKEOFF

    def _do_takeoff(self):
        now = self.get_clock().now()
        self._pub_sp(self._hold_x, self._hold_y, self._target_alt)

        alt = self._pose.pose.position.z
        dx  = self._pose.pose.position.x - self._hold_x
        dy  = self._pose.pose.position.y - self._hold_y

        if not self._header_printed:
            print(f"\n  {'Alt(m)':>8}  {'Tgt(m)':>8}  {'ENU-z':>8}  "
                  f"{'ΔX(m)':>8}  {'ΔY(m)':>8}")
            print(f"  {'─'*8}  {'─'*8}  {'─'*8}  {'─'*8}  {'─'*8}")
            self._header_printed = True

        self._print_1hz(
            f"  {alt:>8.3f}  {self._target_alt:>8.3f}  "
            f"{alt:>8.3f}  {dx:>8.3f}  {dy:>8.3f}")

        if abs(alt - self._target_alt) <= self._alt_tol:
            if self._at_alt_since is None:
                self._at_alt_since = now
            elif self._secs(self._at_alt_since) >= 1.5:
                self.get_logger().info(
                    f"Reached {alt:.3f} m  ΔX={dx:.3f}  ΔY={dy:.3f}")
                self._stable_since = now
                self._phase = Phase.STABLE_OF
        else:
            self._at_alt_since = None

    def _do_stable_of(self):
        # Velocity hold: command zero velocity so optical flow directly
        # counteracts drift — more stable than fighting a drifting EKF
        # position estimate with a fixed position setpoint.
        self._pub_vel_hold()
        elapsed = self._secs(self._stable_since)
        self.get_logger().info(
            f"Holding still on optical flow: "
            f"{elapsed:.1f}/{self._stable_of_secs:.0f}s",
            throttle_duration_sec=1.0)
        if elapsed >= self._stable_of_secs:
            self._phase = Phase.INIT_CAM

    def _do_init_cam(self):
        self._pub_vel_hold()   # vel hold until vision confirmed

        # Capture ACTUAL ENU position right now — used for both RTAB-Map
        # initial_pose and vision_bridge offset. More accurate than
        # target_alt since drone may be at 1.97 or 2.03 m at this moment.
        actual_x = self._pose.pose.position.x
        actual_y = self._pose.pose.position.y
        actual_z = self._pose.pose.position.z

        if self._rtabmap_proc is None:
            db_path = (f"{self._map_dir}/flight_"
                       f"{datetime.now().strftime('%Y%m%d_%H%M%S')}.db")
            os.makedirs(self._map_dir, exist_ok=True)
            # Tell RTAB-Map its true starting pose in the world frame
            # (ENU, origin = home/arm point) — see rtabmap_config.py.
            init_pose = (f"{actual_x:.4f} {actual_y:.4f} "
                         f"{actual_z:.4f} 0 0 0")
            self.get_logger().info(
                f"Launching RTAB-Map → {db_path}  "
                f"initial_pose=[{init_pose}]")
            self._rtabmap_proc = subprocess.Popen(
                build_rtabmap_cmd(db_path, init_pose))

        if self._bridge_proc is None:
            # Pass ACTUAL PX4 ENU position at RTAB-Map init time as offset.
            # vision_bridge adds this to every odom pose before publishing
            # to /mavros/vision_pose/pose, so PX4 EKF gets ground-relative
            # positions and no Z-shift occurs.
            self.get_logger().info(
                f"Launching vision_bridge — offset "
                f"({actual_x:.3f}, {actual_y:.3f}, {actual_z:.3f})…")
            self._bridge_proc = subprocess.Popen(
                build_vision_bridge_cmd(actual_x, actual_y, actual_z))

        self._cam_init_start  = self.get_clock().now()
        self._rtab_good_since = None
        self._phase = Phase.WAIT_VIO

    def _do_wait_vio(self):
        # PX4 fuses optical flow + vision poses automatically. This phase
        # just waits until the camera is initialised before the timed
        # hover — no EKF2 params changed. Velocity hold: zero vx/vy lets
        # optical flow counteract horizontal drift directly.
        self._pub_vel_hold()
        now = self.get_clock().now()

        waiting = self._secs(self._cam_init_start)

        if waiting > self._cam_timeout_secs:
            self.get_logger().error(
                f"RTAB-Map not ready after "
                f"{self._cam_timeout_secs:.0f}s — landing")
            self._phase = Phase.LAND
            return

        if rtab_cov_good(self._rtab_cov):
            if self._rtab_good_since is None:
                self._rtab_good_since = now
                self.get_logger().info(
                    f"RTAB-Map tracking detected (cov={self._rtab_cov:.2f}), "
                    f"confirming {self._cam_good_secs:.0f}s…")
            good_dur = self._secs(self._rtab_good_since)
            self.get_logger().info(
                f"RTAB-Map good {good_dur:.1f}/{self._cam_good_secs:.0f}s  "
                f"cov={self._rtab_cov:.2f}",
                throttle_duration_sec=1.0)
            if good_dur >= self._cam_good_secs:
                # Re-lock home from CURRENT position before switching to
                # position setpoints. The drone may have drifted during
                # vel-hold; capturing now avoids a sudden position jump.
                self._hold_x = self._pose.pose.position.x
                self._hold_y = self._pose.pose.position.y
                self._hold_heading_q = self._pose.pose.orientation
                self.get_logger().info(
                    f"RTAB-Map confirmed — re-locked home at "
                    f"x={self._hold_x:.3f}  y={self._hold_y:.3f}  "
                    f"z={self._pose.pose.position.z:.3f}. Starting hover.")
                self._hover_start = now
                self._phase = Phase.HOVER
        else:
            if self._rtab_good_since is not None:
                self.get_logger().warn(
                    f"RTAB-Map tracking lost "
                    f"(cov={self._rtab_cov:.1f}) — resetting")
                self._rtab_good_since = None

        self.get_logger().info(
            f"Waiting RTAB-Map: {waiting:.1f}/{self._cam_timeout_secs:.0f}s  "
            f"cov={self._rtab_cov:.2f}",
            throttle_duration_sec=3.0)

    def _do_hover(self):
        self._pub_sp(self._hold_x, self._hold_y, self._target_alt)

        # RTAB-Map health watchdog
        if not rtab_cov_good(self._rtab_cov):
            self.get_logger().error(
                f"RTAB-Map LOST during hover "
                f"(cov={self._rtab_cov:.1f}) — landing")
            self._phase = Phase.LAND
            return

        elapsed   = self._secs(self._hover_start)
        remaining = self._hover_duration - elapsed

        if not self._header_printed:
            print(f"\n  {'TimeLeft':>10}  {'Alt(m)':>8}  {'ENU-z':>8}  "
                  f"{'ΔX(m)':>8}  {'ΔY(m)':>8}  "
                  f"{'vx':>6}  {'vy':>6}  {'vz':>6}  {'RTAB-cov':>10}")
            print(f"  {'─'*10}  {'─'*8}  {'─'*8}  {'─'*8}  {'─'*8}  "
                  f"{'─'*6}  {'─'*6}  {'─'*6}  {'─'*10}")
            self._header_printed = True

        alt = self._pose.pose.position.z
        dx  = self._pose.pose.position.x - self._hold_x
        dy  = self._pose.pose.position.y - self._hold_y
        v   = self._vel.twist.linear
        self._print_1hz(
            f"  {remaining:>9.1f}s  {alt:>8.3f}  {alt:>8.3f}  "
            f"{dx:>8.3f}  {dy:>8.3f}  "
            f"{v.x:>6.2f}  {v.y:>6.2f}  {v.z:>6.2f}  "
            f"{self._rtab_cov:>10.2f}")

        if elapsed >= self._hover_duration:
            self.get_logger().info("Hover complete — landing")
            self._phase = Phase.LAND

    def _do_land(self):
        self._request_mode("AUTO.LAND")
        self._phase = Phase.DISARM
        self.get_logger().info("AUTO.LAND requested.")

    def _do_disarm(self):
        # Don't publish OFFBOARD setpoints after AUTO.LAND — PX4 is no
        # longer in OFFBOARD mode and spurious setpoints just add noise.
        if not self._state.armed:
            self.get_logger().info("Disarmed — mission complete ✓")
            self._kill_subprocs()
            self._phase = Phase.DONE

    def _do_safe_manual(self):
        self.get_logger().info(
            f"SAFE MANUAL — CH5={self._ch5_pwm()}. "
            "Restart node with CH5 LOW to fly again.",
            throttle_duration_sec=5.0)

    # ════════════════════════════════════════════════════════════
    #  HELPERS
    # ════════════════════════════════════════════════════════════

    def _pub_sp(self, x: float, y: float, z: float):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "local_origin"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        # Never command 0°-yaw (identity). Use locked heading after arming,
        # or the drone's live heading before arming — prevents PX4 from
        # spinning to face East the moment OFFBOARD activates.
        if self._hold_heading_q is not None:
            msg.pose.orientation = self._hold_heading_q
        else:
            msg.pose.orientation = self._pose.pose.orientation
        self._sp_pub.publish(msg)

    def _pub_vel_hold(self):
        """Zero velocity setpoint for optical-flow-only hold phases.
        Commands vx=vy=vz=0 in ENU so optical flow directly counteracts
        horizontal drift without relying on the EKF position estimate
        (which drifts on optical flow alone)."""
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "local_origin"   # ENU world frame
        # twist defaults to all-zero: zero velocity, no yaw rate
        self._vel_pub.publish(msg)

    def _request_mode(self, mode: str):
        if not self._mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f"set_mode unavailable (wanted {mode})")
            return
        req = SetMode.Request()
        req.custom_mode = mode
        self._mode_cli.call_async(req)

    def _request_arm(self, value: bool):
        if not self._arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("arming service unavailable")
            return
        req = CommandBool.Request()
        req.value = value
        self._arm_cli.call_async(req)

    def _secs(self, t) -> float:
        if t is None:
            return 0.0
        return (self.get_clock().now() - t).nanoseconds * 1e-9

    def _ch5_pwm(self) -> int:
        if len(self._rc) > self._rc_ch5_idx:
            return int(self._rc[self._rc_ch5_idx])
        return 1500   # neutral default

    def _print_1hz(self, line: str):
        now = time.monotonic()
        if now - self._last_print >= 1.0:
            print(line)
            self._last_print = now

    def _kill_subprocs(self):
        for proc in (self._rtabmap_proc, self._bridge_proc):
            if proc and proc.poll() is None:
                proc.terminate()
                self.get_logger().info(f"Terminated PID {proc.pid}")


def main():
    rclpy.init()
    node = AutoMission()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down")
        node._kill_subprocs()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
