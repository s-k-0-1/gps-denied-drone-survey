#!/usr/bin/env python3
"""
auto_land_mission.py  —  Autonomous RTAB-Map VIO Mission (AUTO.LAND)
Viman Rakshak / IRoC-U 2026

Mission phases (all in OFFBOARD — no X/Y drift):
  IDLE       : Wait for CH5 LOW (PWM ≤ 1200) to start
  ARM        : Stream setpoints → OFFBOARD → Arm
               Home X/Y locked from MAVROS local ENU frame (same origin as QGC home)
  TAKEOFF    : Climb to TARGET_ALT (2 m) holding home X/Y
  STABLE_OF  : Hold 3 s still on optical flow — camera warm-up window
  INIT_CAM   : Launch RTAB-Map + vision_bridge subprocess
  WAIT_VIO   : Poll /rtabmap/rtabmap/odom covariance until stable
               PX4 fuses optical-flow + vision automatically (no param change)
  HOVER      : Hold home X/Y at TARGET_ALT for 30 s; watch RTAB-Map health
  LAND       : Switch to AUTO.LAND mode — PX4 handles descent fully
  DISARM     : Wait for PX4 to self-disarm (NO OFFBOARD setpoints during landing)

Key difference from auto_mission.py:
  During DISARM, OFFBOARD setpoints are NOT published.
  AUTO.LAND mode owns the drone completely from LAND phase onwards.

Safety:
  CH5 ≥ 1700  → STABILIZED, SAFE_MANUAL phase (pilot full control)
  Ctrl+C      → AUTO.LAND immediately, then clean shutdown
  RTAB-Map lost during HOVER → AUTO.LAND immediately

Coordinate frame:
  MAVROS publishes /mavros/local_position/pose in ENU (East-North-Up).
  PX4 internally uses NED, but MAVROS auto-converts — we never touch NED directly.
    ENU: X=East  Y=North  Z=Up(+altitude)   ← what this script uses
    NED: X=North Y=East   Z=Down(-altitude)  ← PX4 internal / QGC display
  Origin is the same for both: home/arming point.
  QGC shows NED values so X/Y look swapped vs script — but physics is identical.
  The OFFBOARD setpoint locks the drone to that origin throughout.
  Optical flow + camera fusion is handled by PX4's EKF2 automatically;
  this script does NOT touch any EKF2 parameters.

Map output:
  /media/jetson/ROS2_SSD/maps/flight_<timestamp>.db

Telemetry (1 Hz):
  TAKEOFF  → Alt | Tgt | ENU-z | ΔX | ΔY
  HOVER    → TimeLeft | Alt | ENU-z | ΔX | ΔY | vx | vy | vz | RTAB-cov
"""

import subprocess
import os
import signal
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                       HistoryPolicy, DurabilityPolicy)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, RCIn
from mavros_msgs.srv import CommandBool, SetMode

# ═══════════════════════════════════════════════════════════════════
#  MISSION PARAMETERS  (edit here)
# ═══════════════════════════════════════════════════════════════════

TARGET_ALT        = 2.0    # hover altitude in metres (ENU +Z)
ALT_TOLERANCE     = 0.12   # ±m band to consider "at altitude"

STABLE_OF_SECS    = 3.0    # seconds to hold still on optical flow
                           # before launching RTAB-Map

# RTAB-Map covariance thresholds
# 0 < cov < 100  → tracking good  (matches rtabmap_trigger.py logic)
# cov ≥ 100      → lost (99999 when RTAB-Map reports tracking failure)
CAM_BAD_COV       = 100.0  # covariance ≥ this → tracking lost
CAM_GOOD_SECS     = 5.0    # continuous seconds of good tracking needed
CAM_TIMEOUT_SECS  = 30.0   # abort to land if not ready in this time

HOVER_DURATION    = 30.0   # hover time on RTAB-Map (seconds)

SP_RATE_HZ        = 20.0   # setpoint publish rate (Hz)

# ── RC channel config (0-based index) ─────────────────────────────
RC_CH5_IDX        = 4      # channel 5 = index 4
RC_START_LOW      = 1200   # CH5 PWM ≤ this  → start mission
RC_INTERRUPT_HIGH = 1700   # CH5 PWM ≥ this  → pilot safety takeover

# ── Subprocess paths ──────────────────────────────────────────────
VISION_BRIDGE     = "/home/jetson/drone_ws/vision_bridge.py"
MAP_DIR           = "/media/jetson/ROS2_SSD/maps"

# ═══════════════════════════════════════════════════════════════════


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


class AutoLandMission(Node):

    def __init__(self):
        super().__init__("auto_land_mission")

        self._phase      = Phase.IDLE
        self._last_phase = None

        # Home position locked from MAVROS ENU frame on arm.
        # ENU origin = QGC home/arming point (same reference, different axis convention).
        # X=East, Y=North, Z=Up — all setpoints in this same frame.
        self._hold_x = 0.0
        self._hold_y = 0.0

        # Live sensor data
        self._pose     = PoseStamped()
        self._state    = State()
        self._vel      = TwistStamped()
        self._rc       = []
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

        # ── QoS ──────────────────────────────────────────────────
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        cb = ReentrantCallbackGroup()

        # ── Subscribers ──────────────────────────────────────────
        self.create_subscription(
            State,        "/mavros/state",
            self._state_cb, qos_rel, callback_group=cb)
        self.create_subscription(
            PoseStamped,  "/mavros/local_position/pose",
            self._pose_cb, qos_be, callback_group=cb)
        self.create_subscription(
            TwistStamped, "/mavros/local_position/velocity_local",
            self._vel_cb, qos_be, callback_group=cb)
        self.create_subscription(
            RCIn,         "/mavros/rc/in",
            self._rc_cb, qos_be, callback_group=cb)
        self.create_subscription(
            Odometry,     "/rtabmap/rtabmap/odom",
            self._rtab_cb, qos_rel, callback_group=cb)

        # ── Publishers ───────────────────────────────────────────
        self._sp_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)

        # ── Service clients ──────────────────────────────────────
        self._arm_cli  = self.create_client(CommandBool, "/mavros/cmd/arming",  callback_group=cb)
        self._mode_cli = self.create_client(SetMode,     "/mavros/set_mode",     callback_group=cb)

        # ── Main loop timer ───────────────────────────────────────
        self.create_timer(1.0 / SP_RATE_HZ, self._loop, callback_group=cb)

        self.get_logger().info(
            "AutoLandMission ready. "
            f"Set CH5 ≤ {RC_START_LOW} PWM to start. "
            f"CH5 ≥ {RC_INTERRUPT_HIGH} PWM = safety takeover."
        )

    # ════════════════════════════════════════════════════════════════
    #  CALLBACKS
    # ════════════════════════════════════════════════════════════════

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

        # Safety interrupt — ignored in phases where pilot already has control
        if self._phase in (Phase.SAFE_MANUAL, Phase.DONE, Phase.IDLE):
            return

        if self._ch5_pwm() >= RC_INTERRUPT_HIGH:
            self.get_logger().warn(
                f"⚠  RC INTERRUPT — CH5={self._ch5_pwm()} PWM. "
                "Switching to STABILIZED. Pilot has full control.")
            self._request_mode("STABILIZED")
            self._phase = Phase.SAFE_MANUAL

    # ════════════════════════════════════════════════════════════════
    #  MAIN CONTROL LOOP  (20 Hz)
    # ════════════════════════════════════════════════════════════════

    def _loop(self):
        now = self.get_clock().now()

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

        # ── IDLE ─────────────────────────────────────────────────
        if self._phase == Phase.IDLE:
            self._pub_sp(0.0, 0.0, 0.3)   # pre-stream so OFFBOARD is ready

            if not self._state.connected:
                self.get_logger().info(
                    "Waiting for FCU...", throttle_duration_sec=5.0)
                return

            if self._ch5_pwm() <= RC_START_LOW:
                self.get_logger().info(
                    f"CH5={self._ch5_pwm()} — start trigger received")
                self._phase = Phase.ARM

        # ── ARM ──────────────────────────────────────────────────
        elif self._phase == Phase.ARM:
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
                self.get_logger().info(
                    f"Armed. Home locked from MAVROS ENU: "
                    f"x={self._hold_x:.3f} m (East)  "
                    f"y={self._hold_y:.3f} m (North)"
                )
                self._phase = Phase.TAKEOFF

        # ── TAKEOFF ──────────────────────────────────────────────
        elif self._phase == Phase.TAKEOFF:
            self._pub_sp(self._hold_x, self._hold_y, TARGET_ALT)

            alt = self._pose.pose.position.z
            dx  = self._pose.pose.position.x - self._hold_x
            dy  = self._pose.pose.position.y - self._hold_y

            if not self._header_printed:
                print(f"\n  {'Alt(m)':>8}  {'Tgt(m)':>8}  {'ENU-z':>8}  "
                      f"{'ΔX(m)':>8}  {'ΔY(m)':>8}")
                print(f"  {'─'*8}  {'─'*8}  {'─'*8}  {'─'*8}  {'─'*8}")
                self._header_printed = True

            self._print_1hz(
                f"  {alt:>8.3f}  {TARGET_ALT:>8.3f}  "
                f"{alt:>8.3f}  {dx:>8.3f}  {dy:>8.3f}")

            if abs(alt - TARGET_ALT) <= ALT_TOLERANCE:
                if self._at_alt_since is None:
                    self._at_alt_since = now
                elif self._secs(self._at_alt_since) >= 1.5:
                    self.get_logger().info(
                        f"Reached {alt:.3f} m  ΔX={dx:.3f}  ΔY={dy:.3f}")
                    self._stable_since = now
                    self._phase = Phase.STABLE_OF
            else:
                self._at_alt_since = None

        # ── STABLE on optical flow ────────────────────────────────
        elif self._phase == Phase.STABLE_OF:
            self._pub_sp(self._hold_x, self._hold_y, TARGET_ALT)
            elapsed = self._secs(self._stable_since)
            self.get_logger().info(
                f"Holding still on optical flow: "
                f"{elapsed:.1f}/{STABLE_OF_SECS:.0f}s",
                throttle_duration_sec=1.0)
            if elapsed >= STABLE_OF_SECS:
                self._phase = Phase.INIT_CAM

        # ── INIT_CAM: launch RTAB-Map + vision_bridge ────────────
        elif self._phase == Phase.INIT_CAM:
            self._pub_sp(self._hold_x, self._hold_y, TARGET_ALT)

            # Capture ACTUAL ENU position right now — used for both RTAB-Map
            # initial_pose and vision_bridge offset. More accurate than
            # TARGET_ALT since drone may be at 1.97 or 2.03 m at this moment.
            actual_x = self._pose.pose.position.x
            actual_y = self._pose.pose.position.y
            actual_z = self._pose.pose.position.z

            if self._rtabmap_proc is None:
                from datetime import datetime
                db_path = f"{MAP_DIR}/flight_{datetime.now().strftime('%Y%m%d_%H%M%S')}.db"
                os.makedirs(MAP_DIR, exist_ok=True)
                # ── CRITICAL: tell RTAB-Map its true starting pose in the
                # world frame (ENU, origin = home/arm point).
                # Without this, RTAB-Map sets (0,0,0) at the drone's current
                # position, causing a Z offset in all published vision poses
                # → PX4 EKF shifts → drone overshoots altitude → crash on land.
                # initial_pose format: "x y z roll pitch yaw"
                init_pose = (f"{actual_x:.4f} {actual_y:.4f} "
                             f"{actual_z:.4f} 0 0 0")
                self.get_logger().info(
                    f"Launching RTAB-Map → {db_path}  "
                    f"initial_pose=[{init_pose}]")
                self._rtabmap_proc = subprocess.Popen([
                    "ros2", "launch", "rtabmap_launch", "rtabmap.launch.py",
                    f"database_path:={db_path}",
                    f"initial_pose:={init_pose}",
                    "rgb_topic:=/camera/camera/color/image_raw",
                    "depth_topic:=/camera/camera/depth/image_rect_raw",
                    "camera_info_topic:=/camera/camera/color/camera_info",
                    "frame_id:=camera_link",
                    "approx_sync:=false",
                    "odom_topic:=rtabmap/odom",
                    "visual_odometry:=true",
                    "publish_tf:=true",
                    "tf_delay:=0.05",
                    "tf_tolerance:=0.2",
                    "rviz:=false",
                    "rtabmap_viz:=false",
                    "odom_frame_id:=rtabmap/odom",
                    "Odom/Strategy:=0",
                    "Odom/FilteringStrategy:=1",
                    "Odom/ParticleSize:=400",
                    "Odom/MinInliers:=10",
                    "Odom/ResetCountdown:=50",
                    "Odom/GuessMotion:=true",
                    "Odom/ImageDecimation:=1",
                    "Odom/KeyFrameThr:=0.3",
                    "OdomF2M/MaxSize:=3000",
                    "OdomF2M/MaxNewFeatures:=300",
                    "OdomF2M/BundleAdjustment:=0",
                    "OdomF2M/BundleAdjustmentMaxFrames:=0",
                    "OdomF2M/ValidDepthRatio:=0.1",
                    "Vis/FeatureType:=9",
                    "Vis/MaxFeatures:=800",
                    "Vis/MinInliers:=10",
                    "Vis/InlierDistance:=0.1",
                    "Vis/CorNNDRRatio:=0.80",
                    "Vis/PnPFlags:=0",
                    "Vis/DepthAsMask:=false",
                    "GFTT/MinDistance:=5",
                    "GFTT/QualityLevel:=0.01",
                    "Kp/MaxFeatures:=500",
                    "Kp/MaxDepth:=8.0",
                    "Kp/MinDepth:=0.3",
                    "Kp/NNStrategy:=1",
                    "Kp/BadSignRatio:=0.3",
                    "LccBow/MinInliers:=6",
                    "LccBow/InlierDistance:=0.15",
                    "Reg/VarianceFromInliersCount:=false",
                    "Rtabmap/DetectionRate:=1.0",
                    "RGBD/OptimizeMaxError:=1.5",
                    "RGBD/ProximityBySpace:=true",
                    "RGBD/ProximityMaxGraphDepth:=0",
                    "RGBD/ProximityPathMaxNeighbors:=20",
                    "Mem/STMSize:=50",
                    "Mem/RehearsalSimilarity:=0.20",
                    "Grid/FromDepth:=false",
                    "RGBD/OptimizeFromGraphEnd:=false",
                    "Optimizer/Slam2D:=false",
                ])

            if self._bridge_proc is None:
                # Pass ACTUAL PX4 ENU position at RTAB-Map init time as offset.
                # vision_bridge adds this to every odom pose before publishing
                # to /mavros/vision_pose/pose, so PX4 EKF gets ground-relative
                # positions and no Z-shift occurs.
                self.get_logger().info(
                    f"Launching vision_bridge — offset "
                    f"({actual_x:.3f}, {actual_y:.3f}, {actual_z:.3f})…")
                self._bridge_proc = subprocess.Popen([
                    "python3", VISION_BRIDGE,
                    "--ros-args",
                    "-p", f"offset_x:={actual_x:.4f}",
                    "-p", f"offset_y:={actual_y:.4f}",
                    "-p", f"offset_z:={actual_z:.4f}",
                ])

            self._cam_init_start  = now
            self._rtab_good_since = None
            self._phase = Phase.WAIT_VIO

        # ── WAIT_VIO: wait for RTAB-Map to track reliably ─────────
        # PX4 fuses optical flow + vision poses automatically.
        # This phase just waits until the camera is initialised
        # before we start the timed hover — no EKF2 params changed.
        elif self._phase == Phase.WAIT_VIO:
            self._pub_sp(self._hold_x, self._hold_y, TARGET_ALT)

            waiting = self._secs(self._cam_init_start)

            if waiting > CAM_TIMEOUT_SECS:
                self.get_logger().error(
                    f"RTAB-Map not ready after {CAM_TIMEOUT_SECS:.0f}s — landing")
                self._phase = Phase.LAND
                return

            # Good: 0 < cov < 100  (99999 = lost, 0 = not yet publishing)
            is_good = 0.0 < self._rtab_cov < CAM_BAD_COV

            if is_good:
                if self._rtab_good_since is None:
                    self._rtab_good_since = now
                    self.get_logger().info(
                        f"RTAB-Map tracking detected (cov={self._rtab_cov:.2f}), "
                        f"confirming {CAM_GOOD_SECS:.0f}s…")
                good_dur = self._secs(self._rtab_good_since)
                self.get_logger().info(
                    f"RTAB-Map good {good_dur:.1f}/{CAM_GOOD_SECS:.0f}s  "
                    f"cov={self._rtab_cov:.2f}",
                    throttle_duration_sec=1.0)
                if good_dur >= CAM_GOOD_SECS:
                    self.get_logger().info(
                        "RTAB-Map confirmed. Starting hover.")
                    self._hover_start = now
                    self._phase = Phase.HOVER
            else:
                if self._rtab_good_since is not None:
                    self.get_logger().warn(
                        f"RTAB-Map tracking lost (cov={self._rtab_cov:.1f}) — resetting")
                    self._rtab_good_since = None

            self.get_logger().info(
                f"Waiting RTAB-Map: {waiting:.1f}/{CAM_TIMEOUT_SECS:.0f}s  "
                f"cov={self._rtab_cov:.2f}",
                throttle_duration_sec=3.0)

        # ── HOVER ─────────────────────────────────────────────────
        elif self._phase == Phase.HOVER:
            self._pub_sp(self._hold_x, self._hold_y, TARGET_ALT)

            # RTAB-Map health watchdog
            if self._rtab_cov >= CAM_BAD_COV:
                self.get_logger().error(
                    f"RTAB-Map LOST during hover (cov={self._rtab_cov:.1f}) — landing")
                self._phase = Phase.LAND
                return

            elapsed   = self._secs(self._hover_start)
            remaining = HOVER_DURATION - elapsed

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
            vx  = self._vel.twist.linear.x
            vy  = self._vel.twist.linear.y
            vz  = self._vel.twist.linear.z
            self._print_1hz(
                f"  {remaining:>9.1f}s  {alt:>8.3f}  {alt:>8.3f}  "
                f"{dx:>8.3f}  {dy:>8.3f}  "
                f"{vx:>6.2f}  {vy:>6.2f}  {vz:>6.2f}  "
                f"{self._rtab_cov:>10.2f}")

            if elapsed >= HOVER_DURATION:
                self.get_logger().info("Hover complete — switching to AUTO.LAND")
                self._phase = Phase.LAND

        # ── LAND ─────────────────────────────────────────────────
        # Switch to AUTO.LAND mode. PX4 takes full control of descent.
        # OFFBOARD setpoints are stopped from this point onwards.
        elif self._phase == Phase.LAND:
            self._request_mode("AUTO.LAND")
            self.get_logger().info(
                "AUTO.LAND mode requested — PX4 handling descent.")
            self._phase = Phase.DISARM

        # ── DISARM ───────────────────────────────────────────────
        # AUTO.LAND is active — do NOT publish OFFBOARD setpoints.
        # PX4 lands and auto-disarms on its own. Just wait.
        elif self._phase == Phase.DISARM:
            if not self._state.armed:
                self.get_logger().info("Disarmed — mission complete ✓")
                self._kill_subprocs()
                self._phase = Phase.DONE

        # ── SAFE_MANUAL ──────────────────────────────────────────
        elif self._phase == Phase.SAFE_MANUAL:
            self.get_logger().info(
                f"SAFE MANUAL — CH5={self._ch5_pwm()}. "
                "Restart node with CH5 LOW to fly again.",
                throttle_duration_sec=5.0)

        # ── DONE ─────────────────────────────────────────────────
        elif self._phase == Phase.DONE:
            pass

    # ════════════════════════════════════════════════════════════════
    #  HELPERS
    # ════════════════════════════════════════════════════════════════

    def _pub_sp(self, x: float, y: float, z: float):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        self._sp_pub.publish(msg)

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
        if len(self._rc) > RC_CH5_IDX:
            return int(self._rc[RC_CH5_IDX])
        return 1500   # neutral default

    def _print_1hz(self, line: str):
        import time as _t
        now = _t.monotonic()
        if now - self._last_print >= 1.0:
            print(line)
            self._last_print = now

    def _kill_subprocs(self):
        for proc in (self._rtabmap_proc, self._bridge_proc):
            if proc and proc.poll() is None:
                proc.terminate()
                self.get_logger().info(f"Terminated PID {proc.pid}")


# ═══════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = AutoLandMission()
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
