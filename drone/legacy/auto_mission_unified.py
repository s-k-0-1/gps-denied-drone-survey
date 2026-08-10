#!/usr/bin/env python3
"""
auto_mission_unified.py — Single-process autonomous RTAB-Map VIO mission
Viman Rakshak / IRoC-U 2026

Run directly:  python3 auto_mission_unified.py

ONE script, ONE node, ONE process. Replaces:
  auto_mission.py     → mission state machine        (merged: this file)
  vision_bridge.py    → RTAB-odom → MAVROS vision    (merged: in-process,
                        no subprocess, no extra serialization hop,
                        offset handoff is a variable assignment)
  rtabmap_trigger.py  → quality-gated bridge start   (merged: WAIT_VIO phase)
Only RTAB-Map itself is still a subprocess (C++ launch stack).

Optimizations vs the 3-script version:
  • vision_bridge in-process: /rtabmap/rtabmap/odom is subscribed ONCE and
    serves both covariance monitoring and vision-pose republishing.
    Old chain: rtabmap → bridge proc → DDS → mavros  (2 hops, 3 processes)
    New chain: rtabmap → this node    → mavros       (1 hop,  1 process)
  • Offset handoff: previously CLI params to a spawned process at INIT_CAM;
    now three float assignments — zero latency, zero spawn cost.
  • Non-blocking service calls: service_is_ready() instead of
    wait_for_service(1.0). The old blocking wait inside the 20 Hz loop could
    stall setpoint streaming long enough to trip the PX4 OFFBOARD failsafe.
  • Hot paths allocate nothing avoidable: remap constants precomputed,
    throttle uses integer nanoseconds, telemetry strings built only at 1 Hz.

Mission phases (all in OFFBOARD — no X/Y drift):
  IDLE       : Wait for CH5 LOW (PWM ≤ 1200) to start
  ARM        : Stream setpoints → OFFBOARD → Arm; home X/Y + heading locked
  TAKEOFF    : Climb to TARGET_ALT holding home X/Y
  STABLE_OF  : Hold 3 s still on optical flow — camera warm-up window
  INIT_CAM   : Launch RTAB-Map subprocess; arm the in-process vision bridge
  WAIT_VIO   : Gate on /rtabmap/rtabmap/odom covariance until stable
  HOVER      : Hold home X/Y at TARGET_ALT for 30 s; RTAB-Map watchdog
  LAND       : AUTO.LAND
  DISARM     : Wait for PX4 to self-disarm after touchdown

Safety:
  CH5 ≥ 1700      → STABILIZED, SAFE_MANUAL (pilot full control)
  Ctrl+C          → AUTO.LAND immediately, then clean shutdown
  RTAB-Map lost during HOVER → AUTO.LAND immediately

Coordinate frame:
  MAVROS publishes /mavros/local_position/pose in ENU (East-North-Up).
  PX4 internally uses NED, but MAVROS auto-converts — we never touch NED.
    ENU: X=East  Y=North  Z=Up(+altitude)   ← what this script uses
    NED: X=North Y=East   Z=Down(-altitude) ← PX4 internal / QGC display
  Origin is the same for both: home/arming point.
  Optical flow + camera fusion is handled by PX4's EKF2 automatically;
  this script does NOT touch any EKF2 parameters.

Map output: /media/jetson/ROS2_SSD/maps/flight_<timestamp>.db
"""

import math
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
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import RCIn, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry

# ═══════════════════════════════════════════════════════════════════
#  MISSION PARAMETERS  (edit here)
# ═══════════════════════════════════════════════════════════════════

TARGET_ALT        = 1.0    # hover altitude in metres (ENU +Z)
ALT_TOLERANCE     = 0.12   # ±m band to consider "at altitude"
AT_ALT_CONFIRM_S  = 1.5    # seconds inside band before TAKEOFF → STABLE_OF

STABLE_OF_SECS    = 3.0    # hold-still time on optical flow before RTAB-Map

# RTAB-Map covariance thresholds (pose.covariance[0], x-x diagonal)
# 0 < cov < 100 → tracking good;  ≥ 100 → lost (99999 on failure);  0 → silent
CAM_BAD_COV       = 100.0
CAM_GOOD_SECS     = 5.0    # continuous seconds of good tracking needed
CAM_TIMEOUT_SECS  = 30.0   # abort to land if VIO not ready in this time

HOVER_DURATION    = 30.0   # hover time on RTAB-Map (seconds)
SP_RATE_HZ        = 20.0   # setpoint publish rate (Hz)
VISION_RATE_HZ    = 30.0   # vision-pose republish cap to MAVROS (Hz)

# ── RC channel config (0-based index) ─────────────────────────────
RC_CH5_IDX        = 4      # channel 5 = index 4
RC_START_LOW      = 1200   # CH5 PWM ≤ this → start mission
RC_INTERRUPT_HIGH = 1700   # CH5 PWM ≥ this → pilot safety takeover

MAP_DIR           = "/media/jetson/ROS2_SSD/maps"

# ═══════════════════════════════════════════════════════════════════
#  RTAB-MAP LAUNCH COMMAND
# ═══════════════════════════════════════════════════════════════════
# CLI override strategy (rtabmap.launch.py):
#   `args`         → BOTH rgbd_odometry AND rtabmap nodes
#   `odom_args`    → ONLY rgbd_odometry
#   `rtabmap_args` → ONLY rtabmap (merged with args)
# CRITICAL: Odom/* and OdomF2M/* MUST go in odom_args — rtabmap doesn't
# declare them and crashes with ParameterNotDeclaredException otherwise.
# OdomF2M/BundleAdjustment MUST be in odom_args — the := form is overridden
# by the param flood back to 1 ("Too low inliers after bundle adjustment").

# ── MAX-QUALITY TUNING ─────────────────────────────────────────────
# Strategy: the rgbd_odometry node is flight-critical and must stay
# real-time (≥15 Hz on the Jetson) — tuned up only moderately. The
# rtabmap (SLAM/mapping) node runs asynchronously and can lag without
# endangering the drone — tuned up aggressively.
# Previous flight-tested values shown as  (was X)  — revert if the
# Jetson can't hold odometry rate (check: ros2 topic hz /rtabmap/odom).

_SHARED_CLI = (
    "--Vis/MinInliers 8 --Vis/InlierDistance 0.1 --Vis/CorNNDRRatio 0.80 "
    "--GFTT/MinDistance 5 "      # (was 7)  denser feature grid
    "--GFTT/QualityLevel 0.005 " # (was 0.01) accept weaker corners → more features
    "--Kp/MaxFeatures 750 "      # (was 500) richer loop-closure vocabulary
    "--Kp/MaxDepth 8.0 --Kp/MinDepth 0.3"
)
_ODOM_ONLY_CLI = (
    "--Odom/ResetCountdown 5 --Odom/Strategy 0 --Odom/FilteringStrategy 1 "
    "--Odom/GuessMotion true --Odom/KeyFrameThr 0.5 "
    "--OdomF2M/BundleAdjustment 0 "  # KEEP 0 — BA over-filters inliers (flight-tested fix)
    "--OdomF2M/MaxSize 4000 "        # (was 3000) bigger local feature map → steadier poses
    "--OdomF2M/MaxNewFeatures 400 "  # (was 300)
    "--OdomF2M/ValidDepthRatio 0.3"
)
_RTABMAP_ONLY_CLI = (
    "--Vis/MinInliers 8 "
    "--Rtabmap/DetectionRate 2.0 "   # (was 1.0) 2 Hz keyframes → denser map graph
    "--RGBD/OptimizeMaxError 1.5 "
    "--RGBD/LinearUpdate 0.05 "      # add nodes after 5 cm motion (default 0.1 m)
    "--RGBD/AngularUpdate 0.05 "     # ...or ~3° rotation — captures slow hover drift
    "--Mem/ImagePreDecimation 1 "    # store full-resolution images in the .db
    "--Mem/ImagePostDecimation 1 "   # → best possible offline reprocessing/export
    "--Mem/NotLinkedNodesKept true"  # keep every captured frame in the database
)


def build_rtabmap_cmd(db_path: str, init_pose: str):
    """initial_pose ("x y z roll pitch yaw") = true starting pose in the
    world frame (ENU, origin = home/arm point). Without it RTAB-Map zeroes
    at the drone's current position → Z offset in vision poses → PX4 EKF
    shifts → altitude overshoot → crash on land."""
    return [
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
        # `:=` form for params not subject to flood override:
        "Odom/ImageDecimation:=1",      # full-resolution odometry
        "Vis/FeatureType:=9",
        "Vis/MaxFeatures:=1200",        # (was 800) more odom features — watch CPU
        "Vis/PnPFlags:=0",
        "Vis/DepthAsMask:=false",
        "Kp/NNStrategy:=1",
        "Kp/BadSignRatio:=0.3",
        "LccBow/MinInliers:=6",
        "LccBow/InlierDistance:=0.15",
        "Reg/VarianceFromInliersCount:=false",
        "RGBD/ProximityBySpace:=true",
        "RGBD/ProximityMaxGraphDepth:=0",
        "RGBD/ProximityPathMaxNeighbors:=20",
        "Mem/STMSize:=50",
        "Mem/RehearsalSimilarity:=0.5",
        "Grid/FromDepth:=false",
        "RGBD/OptimizeFromGraphEnd:=false",
        "Optimizer/Slam2D:=false",
        # CLI strings — applied by RTAB-Map's own parser, highest priority:
        f"args:={_SHARED_CLI}",
        f"odom_args:={_ODOM_ONLY_CLI}",
        f"rtabmap_args:={_RTABMAP_ONLY_CLI}",
    ]


# ═══════════════════════════════════════════════════════════════════
#  VISION BRIDGE MATH  (precomputed, allocation-free)
# ═══════════════════════════════════════════════════════════════════

# Rotation of -90° around Z (camera frame → ENU heading)
_RZ_Z = -0.7071067811865476
_RZ_W = 0.7071067811865476


def _odom_valid(p, q) -> bool:
    """NaN/Inf + quaternion-norm sanity check on an incoming odom pose."""
    vals = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    for v in vals:
        if math.isnan(v) or math.isinf(v):
            return False
    norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
    # |norm - 1| ≤ 0.01  ⇔  0.9801 ≤ norm² ≤ 1.0201 (skips the sqrt)
    return 0.9801 <= norm_sq <= 1.0201


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


class UnifiedMission(Node):
    """Mission state machine + in-process vision bridge, one node."""

    def __init__(self):
        super().__init__("auto_mission")

        self._phase      = Phase.IDLE
        self._last_phase = None

        # Home position + heading locked from MAVROS ENU frame on arm.
        # Heading lock prevents PX4 "rotate to face East" yaw spin.
        self._hold_x = 0.0
        self._hold_y = 0.0
        self._hold_heading_q = None   # None = use live pose

        # Live sensor data
        self._pose = PoseStamped()
        self._pose.pose.orientation.w = 1.0   # identity, not zero-quat
        self._state = State()
        self._vel   = TwistStamped()
        self._rc    = ()
        self._rtab_cov = float("inf")

        # ── In-process vision bridge state ───────────────────────
        # Armed at INIT_CAM with the drone's exact ENU position at RTAB-Map
        # start (RTAB-odom zeroes wherever it inits; adding this offset
        # makes poses ground-relative so PX4 EKF gets no Z-shift).
        self._bridge_enabled = False
        self._off_x = 0.0
        self._off_y = 0.0
        self._off_z = 0.0
        self._vision_interval_ns = int(1e9 / VISION_RATE_HZ)
        self._last_vision_ns = 0
        self._valid_count = 0
        self._nan_count   = 0

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

        # Subprocess (RTAB-Map only — bridge is in-process now)
        self._rtabmap_proc = None

        # ── QoS ──────────────────────────────────────────────────
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10,
            durability=DurabilityPolicy.VOLATILE)
        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10,
            durability=DurabilityPolicy.VOLATILE)

        cb = ReentrantCallbackGroup()

        # ── Subscribers (every external topic, one node) ─────────
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
        # ONE odom subscription feeds BOTH the covariance watchdog and the
        # vision bridge (was two subscriptions in two processes).
        self.create_subscription(
            Odometry, "/rtabmap/rtabmap/odom",
            self._rtab_cb, qos_rel, callback_group=cb)

        # ── Publishers ───────────────────────────────────────────
        self._sp_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)
        # Velocity setpoints for optical-flow-only phases (STABLE_OF,
        # INIT_CAM, WAIT_VIO): zero-velocity hold lets flow counteract
        # drift directly instead of fighting a drifting EKF position.
        self._vel_pub = self.create_publisher(
            TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10)
        # Vision bridge output (was vision_bridge.py's publisher):
        self._vision_pub = self.create_publisher(
            PoseStamped, "/mavros/vision_pose/pose", qos_rel)

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

        self.create_timer(1.0 / SP_RATE_HZ, self._loop, callback_group=cb)

        self.get_logger().info(
            "UnifiedMission ready (mission + vision bridge in-process). "
            f"Set CH5 ≤ {RC_START_LOW} PWM to start. "
            f"CH5 ≥ {RC_INTERRUPT_HIGH} PWM = safety takeover.")

    # ════════════════════════════════════════════════════════════
    #  CALLBACKS
    # ════════════════════════════════════════════════════════════

    def _state_cb(self, msg: State):
        self._state = msg

    def _pose_cb(self, msg: PoseStamped):
        self._pose = msg

    def _vel_cb(self, msg: TwistStamped):
        self._vel = msg

    def _sigint_handler(self, sig, frame):
        self.get_logger().warn("Ctrl+C — scheduling emergency AUTO.LAND")
        self._ctrl_c = True

    def _rc_cb(self, msg: RCIn):
        self._rc = msg.channels

        # Safety interrupt — ignored where pilot already has control
        if self._phase in (Phase.SAFE_MANUAL, Phase.DONE, Phase.IDLE):
            return

        if self._ch5_pwm() >= RC_INTERRUPT_HIGH:
            self.get_logger().warn(
                f"⚠  RC INTERRUPT — CH5={self._ch5_pwm()} PWM. "
                "Switching to STABILIZED. Pilot has full control.")
            self._request_mode("STABILIZED")
            self._phase = Phase.SAFE_MANUAL

    def _rtab_cb(self, msg: Odometry):
        """Single hot path for ALL RTAB-Map odometry:
        1. covariance → watchdog/state machine,
        2. valid poses → remap + offset → /mavros/vision_pose/pose.
        Runs at full odom rate — kept allocation-light deliberately."""
        # cov[0] = x-x diagonal; small positive = good; 99999 = lost
        self._rtab_cov = msg.pose.covariance[0]

        if not self._bridge_enabled:
            return

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        if not _odom_valid(p, q):
            self._nan_count += 1
            if self._nan_count % 100 == 1:
                self.get_logger().warn(
                    f"Bridge: dropping invalid pose #{self._nan_count}")
            return
        self._nan_count = 0

        # 30 Hz throttle — integer ns compare, no object construction
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_vision_ns < self._vision_interval_ns:
            return
        self._last_vision_ns = now_ns

        # Position remap (camera → ENU) + world-frame offset
        out = PoseStamped()
        out.header.stamp    = msg.header.stamp
        out.header.frame_id = "map"
        out.pose.position.x =  -p.x + self._off_x
        out.pose.position.y =  p.y + self._off_y
        out.pose.position.z =  p.z + self._off_z
        # Quaternion remap: -90° about Z (precomputed constants)
        out.pose.orientation.x = _RZ_W * q.x - _RZ_Z * q.y
        out.pose.orientation.y = _RZ_W * q.y + _RZ_Z * q.x
        out.pose.orientation.z = _RZ_W * q.z + _RZ_Z * q.w
        out.pose.orientation.w = _RZ_W * q.w - _RZ_Z * q.z
        self._vision_pub.publish(out)

        self._valid_count += 1
        if self._valid_count % 50 == 1:
            self.get_logger().info(
                f"Bridge pose #{self._valid_count}: "
                f"x={out.pose.position.x:.3f} "
                f"y={out.pose.position.y:.3f} "
                f"z={out.pose.position.z:.3f}")

    # ════════════════════════════════════════════════════════════
    #  MAIN CONTROL LOOP  (20 Hz)
    # ════════════════════════════════════════════════════════════

    def _loop(self):
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

        if self._ch5_pwm() <= RC_START_LOW:
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
            self._hold_x = self._pose.pose.position.x
            self._hold_y = self._pose.pose.position.y
            self._hold_heading_q = self._pose.pose.orientation
            q = self._hold_heading_q
            yaw_deg = math.degrees(
                math.atan2(2 * (q.w * q.z + q.x * q.y),
                           1 - 2 * (q.y * q.y + q.z * q.z)))
            self.get_logger().info(
                f"Armed. Home locked — "
                f"x={self._hold_x:.3f} m (East)  "
                f"y={self._hold_y:.3f} m (North)  "
                f"yaw={yaw_deg:.1f}°")
            self._phase = Phase.TAKEOFF

    def _do_takeoff(self):
        self._pub_sp(self._hold_x, self._hold_y, TARGET_ALT)

        alt = self._pose.pose.position.z

        if abs(alt - TARGET_ALT) <= ALT_TOLERANCE:
            if self._at_alt_since is None:
                self._at_alt_since = self.get_clock().now()
            elif self._secs(self._at_alt_since) >= AT_ALT_CONFIRM_S:
                dx = self._pose.pose.position.x - self._hold_x
                dy = self._pose.pose.position.y - self._hold_y
                self.get_logger().info(
                    f"Reached {alt:.3f} m  ΔX={dx:.3f}  ΔY={dy:.3f}")
                self._stable_since = self.get_clock().now()
                self._phase = Phase.STABLE_OF
                return
        else:
            self._at_alt_since = None

        # Telemetry — strings only built at 1 Hz, not 20 Hz
        if self._print_due():
            if not self._header_printed:
                print(f"\n  {'Alt(m)':>8}  {'Tgt(m)':>8}  {'ENU-z':>8}  "
                      f"{'ΔX(m)':>8}  {'ΔY(m)':>8}")
                print(f"  {'─'*8}  {'─'*8}  {'─'*8}  {'─'*8}  {'─'*8}")
                self._header_printed = True
            dx = self._pose.pose.position.x - self._hold_x
            dy = self._pose.pose.position.y - self._hold_y
            print(f"  {alt:>8.3f}  {TARGET_ALT:>8.3f}  "
                  f"{alt:>8.3f}  {dx:>8.3f}  {dy:>8.3f}")

    def _do_stable_of(self):
        self._pub_vel_hold()
        elapsed = self._secs(self._stable_since)
        self.get_logger().info(
            f"Holding still on optical flow: "
            f"{elapsed:.1f}/{STABLE_OF_SECS:.0f}s",
            throttle_duration_sec=1.0)
        if elapsed >= STABLE_OF_SECS:
            self._phase = Phase.INIT_CAM

    def _do_init_cam(self):
        self._pub_vel_hold()   # vel hold until vision confirmed

        # Capture ACTUAL ENU position right now — used for both RTAB-Map
        # initial_pose and the bridge offset. More accurate than TARGET_ALT
        # since the drone may be at 1.97 or 2.03 m at this moment.
        actual_x = self._pose.pose.position.x
        actual_y = self._pose.pose.position.y
        actual_z = self._pose.pose.position.z

        if self._rtabmap_proc is None:
            db_path = (f"{MAP_DIR}/flight_"
                       f"{datetime.now().strftime('%Y%m%d_%H%M%S')}.db")
            os.makedirs(MAP_DIR, exist_ok=True)
            init_pose = (f"{actual_x:.4f} {actual_y:.4f} "
                         f"{actual_z:.4f} 0 0 0")
            self.get_logger().info(
                f"Launching RTAB-Map → {db_path}  "
                f"initial_pose=[{init_pose}]")
            self._rtabmap_proc = subprocess.Popen(
                build_rtabmap_cmd(db_path, init_pose))

        # Arm the in-process bridge (was: spawn vision_bridge.py with CLI
        # params). Three assignments — instant, no process spawn.
        self._off_x = actual_x
        self._off_y = actual_y
        self._off_z = actual_z
        self._bridge_enabled = True
        self.get_logger().info(
            f"Vision bridge armed — offset "
            f"({actual_x:.3f}, {actual_y:.3f}, {actual_z:.3f})")

        self._cam_init_start  = self.get_clock().now()
        self._rtab_good_since = None
        self._phase = Phase.WAIT_VIO

    def _do_wait_vio(self):
        # PX4 fuses optical flow + vision poses automatically; this phase
        # just gates the timed hover on camera health. No EKF2 params touched.
        self._pub_vel_hold()
        now = self.get_clock().now()

        waiting = self._secs(self._cam_init_start)
        if waiting > CAM_TIMEOUT_SECS:
            self.get_logger().error(
                f"RTAB-Map not ready after {CAM_TIMEOUT_SECS:.0f}s — landing")
            self._phase = Phase.LAND
            return

        # Good: 0 < cov < 100  (99999 = lost, 0 = not yet publishing)
        if 0.0 < self._rtab_cov < CAM_BAD_COV:
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
                # Re-lock home from CURRENT position before returning to
                # position setpoints — avoids a jump after vel-hold drift.
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
            f"Waiting RTAB-Map: {waiting:.1f}/{CAM_TIMEOUT_SECS:.0f}s  "
            f"cov={self._rtab_cov:.2f}",
            throttle_duration_sec=3.0)

    def _do_hover(self):
        self._pub_sp(self._hold_x, self._hold_y, TARGET_ALT)

        # RTAB-Map health watchdog
        if self._rtab_cov >= CAM_BAD_COV:
            self.get_logger().error(
                f"RTAB-Map LOST during hover "
                f"(cov={self._rtab_cov:.1f}) — landing")
            self._phase = Phase.LAND
            return

        elapsed = self._secs(self._hover_start)
        if elapsed >= HOVER_DURATION:
            self.get_logger().info("Hover complete — landing")
            self._phase = Phase.LAND
            return

        # Telemetry — strings only built at 1 Hz, not 20 Hz
        if self._print_due():
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
            print(f"  {HOVER_DURATION - elapsed:>9.1f}s  "
                  f"{alt:>8.3f}  {alt:>8.3f}  "
                  f"{dx:>8.3f}  {dy:>8.3f}  "
                  f"{v.x:>6.2f}  {v.y:>6.2f}  {v.z:>6.2f}  "
                  f"{self._rtab_cov:>10.2f}")

    def _do_land(self):
        self._request_mode("AUTO.LAND")
        self._phase = Phase.DISARM
        self.get_logger().info("AUTO.LAND requested.")

    def _do_disarm(self):
        # No OFFBOARD setpoints after AUTO.LAND — PX4 left OFFBOARD and
        # spurious setpoints just add noise. Bridge keeps publishing vision
        # poses so the EKF stays healthy through touchdown.
        if not self._state.armed:
            self.get_logger().info("Disarmed — mission complete ✓")
            self._shutdown_pipeline()
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
        # Never command 0°-yaw (identity): locked heading after arming,
        # live heading before — prevents the "spin to face East" on OFFBOARD.
        if self._hold_heading_q is not None:
            msg.pose.orientation = self._hold_heading_q
        else:
            msg.pose.orientation = self._pose.pose.orientation
        self._sp_pub.publish(msg)

    def _pub_vel_hold(self):
        """Zero-velocity setpoint for optical-flow-only hold phases."""
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "local_origin"   # ENU world frame
        # twist defaults to all-zero: vx=vy=vz=0, no yaw rate
        self._vel_pub.publish(msg)

    def _request_mode(self, mode: str):
        # Non-blocking: service_is_ready() instead of wait_for_service(1.0).
        # A blocking wait here runs inside the 20 Hz loop and can stall
        # setpoint streaming → PX4 OFFBOARD failsafe. The loop retries
        # every tick anyway, so skipping one cycle is safe.
        if not self._mode_cli.service_is_ready():
            self.get_logger().error(
                f"set_mode unavailable (wanted {mode})",
                throttle_duration_sec=2.0)
            return
        req = SetMode.Request()
        req.custom_mode = mode
        self._mode_cli.call_async(req)

    def _request_arm(self, value: bool):
        if not self._arm_cli.service_is_ready():
            self.get_logger().error(
                "arming service unavailable", throttle_duration_sec=2.0)
            return
        req = CommandBool.Request()
        req.value = value
        self._arm_cli.call_async(req)

    def _secs(self, t) -> float:
        if t is None:
            return 0.0
        return (self.get_clock().now() - t).nanoseconds * 1e-9

    def _ch5_pwm(self) -> int:
        rc = self._rc
        if len(rc) > RC_CH5_IDX:
            return int(rc[RC_CH5_IDX])
        return 1500   # neutral default

    def _print_due(self) -> bool:
        now = time.monotonic()
        if now - self._last_print >= 1.0:
            self._last_print = now
            return True
        return False

    def _shutdown_pipeline(self):
        self._bridge_enabled = False
        if self._rtabmap_proc and self._rtabmap_proc.poll() is None:
            self._rtabmap_proc.terminate()
            self.get_logger().info(
                f"Terminated RTAB-Map PID {self._rtabmap_proc.pid}")


def main():
    rclpy.init()
    node = UnifiedMission()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down")
        node._shutdown_pipeline()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
