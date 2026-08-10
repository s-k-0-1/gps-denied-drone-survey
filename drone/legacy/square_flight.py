#!/usr/bin/env python3
"""
Autonomous Square Flight Script
IRoC-U 2026 — Team Viman Rakshak

Flight plan:
  1. Arm + takeoff to 1m (optical flow only)
  2. Wait for RTAB-Map vision fusion
  3. Climb to 2m
  4. Hold 20s at origin
  5. Forward 1m → hold 20s
  6. Right 1m → hold 20s
  7. Back 1m → hold 20s
  8. Left 1m → hold 20s
  9. Land

Safety:
  - Ctrl+C → force land
  - RC mode change → exit offboard
"""

import math
import signal
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode


# ── Mission parameters ────────────────────────────────────────────────────────
INIT_ALT        = 1.5    # metres — hover here while RTAB-Map initializes
TAKEOFF_ALT     = 2    # metres — mission altitude
HOLD_SECS       = 10.0   # seconds at each waypoint
STEP_SIZE       = 1.0    # metres per leg
POSITION_TOL    = 0.15   # metres — waypoint acceptance radius
OFFBOARD_HZ     = 20.0   # setpoint publish rate
VISION_TIMEOUT  = 60.0   # seconds to wait for vision fusion before aborting


class SquareFlight(Node):

    def __init__(self):
        super().__init__("square_flight")

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.sp_pub = self.create_publisher(
            PoseStamped,
            "/mavros/setpoint_position/local",
            qos_reliable
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        self.state          = State()
        self.extended_state = ExtendedState()
        self.local_pose     = PoseStamped()
        self._vision_active = False

        self.create_subscription(State,       "/mavros/state",
                                 self._state_cb, qos_sensor)
        self.create_subscription(ExtendedState, "/mavros/extended_state",
                                 self._ext_state_cb, qos_sensor)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._pose_cb, qos_sensor)
        self.create_subscription(PoseStamped, "/mavros/vision_pose/pose",
                                 self._vision_cb, qos_reliable)

        # ── Service clients ───────────────────────────────────────────────────
        self.arming_client   = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode,     "/mavros/set_mode")

        # ── Internal state ────────────────────────────────────────────────────
        self._force_land = False
        self._current_yaw = 0.0
        self._origin_x = 0.0
        self._origin_y = 0.0

        # ── Signal handlers ───────────────────────────────────────────────────
        signal.signal(signal.SIGINT,  self._sigint_handler)
        signal.signal(signal.SIGTERM, self._sigint_handler)

        self.get_logger().info("Square flight node ready")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _state_cb(self, msg: State):
        prev = self.state.mode
        self.state = msg
        if prev and prev != msg.mode and prev == "OFFBOARD":
            self.get_logger().warn(
                f"Mode changed OFFBOARD → {msg.mode} — pilot took over"
            )
            self._force_land = True

    def _ext_state_cb(self, msg: ExtendedState):
        self.extended_state = msg

    def _pose_cb(self, msg: PoseStamped):
        self.local_pose = msg

    def _vision_cb(self, msg: PoseStamped):
        self._vision_active = True

    def _sigint_handler(self, sig, frame):
        self.get_logger().warn("Ctrl+C — force landing!")
        self._force_land = True

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _make_sp(self, x, y, z, yaw=0.0) -> PoseStamped:
        sp = PoseStamped()
        sp.header.frame_id = "map"
        sp.pose.position.x = float(x)
        sp.pose.position.y = float(y)
        sp.pose.position.z = float(z)
        sp.pose.orientation.x = 0.0
        sp.pose.orientation.y = 0.0
        sp.pose.orientation.z = math.sin(yaw / 2.0)
        sp.pose.orientation.w = math.cos(yaw / 2.0)
        return sp

    def _pub_sp(self, sp: PoseStamped):
        sp.header.stamp = self.get_clock().now().to_msg()
        self.sp_pub.publish(sp)

    def _distance_to(self, x, y, z) -> float:
        p = self.local_pose.pose.position
        return math.sqrt((p.x-x)**2 + (p.y-y)**2 + (p.z-z)**2)

    def _at_position(self, x, y, z) -> bool:
        return self._distance_to(x, y, z) < POSITION_TOL

    def _set_mode(self, mode: str) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        return future.result() and future.result().mode_sent

    def _arm(self) -> bool:
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        return future.result() and future.result().success

    def _land(self):
        self.get_logger().info("Landing...")
        self._set_mode("AUTO.LAND")

    def _spin_rate(self, secs: float):
        """Spin ROS for given seconds at OFFBOARD_HZ."""
        rate = 1.0 / OFFBOARD_HZ
        end  = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=rate)
            if self._force_land:
                return

    def _publish_for_secs(self, sp: PoseStamped, secs: float):
        rate = 1.0 / OFFBOARD_HZ
        end  = time.time() + secs
        while time.time() < end:
            if self._force_land:
                return
            if self.state.mode != "OFFBOARD" and self.state.armed:
                self.get_logger().warn("Not in OFFBOARD — stopping")
                self._force_land = True
                return
            self._pub_sp(sp)
            rclpy.spin_once(self, timeout_sec=rate)

    def _go_to(self, x, y, z, label="waypoint", timeout=30.0):
        sp = self._make_sp(x, y, z)
        self.get_logger().info(f"→ {label}: ({x:.1f}, {y:.1f}, {z:.1f})")
        rate    = 1.0 / OFFBOARD_HZ
        deadline = time.time() + timeout
        while not self._at_position(x, y, z):
            if self._force_land:
                return
            if time.time() > deadline:
                self.get_logger().warn(f"Timeout reaching {label} — continuing")
                break
            self._pub_sp(sp)
            rclpy.spin_once(self, timeout_sec=rate)
        self.get_logger().info(f"✓ Reached {label}")

    def _hold(self, x, y, z, secs=None):
        sp   = self._make_sp(x, y, z)
        secs = secs or HOLD_SECS
        self.get_logger().info(f"Holding ({x:.1f},{y:.1f},{z:.1f}) for {secs:.0f}s")
        self._publish_for_secs(sp, secs)

    # ── Mission ───────────────────────────────────────────────────────────────

    def run(self):
        rate = 1.0 / OFFBOARD_HZ

        # Step 1 — Wait for MAVROS
        self.get_logger().info("Waiting for MAVROS...")
        while not self.state.connected:
            rclpy.spin_once(self, timeout_sec=rate)
            if self._force_land:
                return
        self.get_logger().info("MAVROS connected")

        # Step 2 — Pre-arm: stream setpoints for 2s
        self.get_logger().info("Streaming setpoints pre-arm...")
        sp_init = self._make_sp(0.0, 0.0, INIT_ALT)
        for _ in range(int(OFFBOARD_HZ * 2)):
            self._pub_sp(sp_init)
            rclpy.spin_once(self, timeout_sec=rate)
            if self._force_land:
                return

        # Step 3 — Switch to OFFBOARD and arm
        self.get_logger().info("Switching to OFFBOARD...")
        if not self._set_mode("OFFBOARD"):
            self.get_logger().error("OFFBOARD switch failed — aborting")
            return

        self.get_logger().info("Arming...")
        if not self._arm():
            self.get_logger().error("Arm failed — aborting")
            return
        self.get_logger().info("Armed!")

        # Step 4 — Climb to INIT_ALT (1m) using optical flow
        self.get_logger().info(f"Climbing to {INIT_ALT}m for RTAB-Map init...")
        self._go_to(0.0, 0.0, INIT_ALT, f"init alt {INIT_ALT}m", timeout=20.0)
        if self._force_land:
            self._land(); return

        # Step 5 — Wait for RTAB-Map vision fusion
        self.get_logger().info(
            f"Holding at {INIT_ALT}m — waiting for vision fusion (max {VISION_TIMEOUT:.0f}s)..."
        )
        sp_init = self._make_sp(0.0, 0.0, INIT_ALT)
        vision_deadline = time.time() + VISION_TIMEOUT
        while not self._vision_active:
            if self._force_land:
                self._land(); return
            if time.time() > vision_deadline:
                self.get_logger().error(
                    "Vision not fused in time — landing for safety"
                )
                self._land()
                return
            self._pub_sp(sp_init)
            rclpy.spin_once(self, timeout_sec=rate)
            remaining = vision_deadline - time.time()
            if int(remaining) % 10 == 0:
                self.get_logger().info(
                    f"Still waiting for vision... {remaining:.0f}s remaining"
                )

        self.get_logger().info("Vision fused! Letting EKF2 settle for 10s...")

        # Let EKF2 stabilize after vision fusion — position will jump then settle
        sp_init = self._make_sp(0.0, 0.0, INIT_ALT)
        settle_end = time.time() + 10.0
        while time.time() < settle_end:
            if self._force_land:
                self._land(); return
            self._pub_sp(sp_init)
            rclpy.spin_once(self, timeout_sec=rate)

        # Capture the current position as mission origin AFTER vision settled
        self._origin_x = self.local_pose.pose.position.x
        self._origin_y = self.local_pose.pose.position.y
        self.get_logger().info(
            f"Mission origin captured: ({self._origin_x:.2f}, {self._origin_y:.2f})"
        )

        # Step 6 — Climb to mission altitude
        self._go_to(self._origin_x, self._origin_y, TAKEOFF_ALT,
                    f"mission alt {TAKEOFF_ALT}m", timeout=20.0)
        if self._force_land:
            self._land(); return

        ox, oy = self._origin_x, self._origin_y

        # Step 7 — Hold at origin
        self.get_logger().info("=== MISSION START ===")
        self._hold(ox, oy, TAKEOFF_ALT)
        if self._force_land:
            self._land(); return

        # Step 8 — Forward 1m
        self._go_to(ox + STEP_SIZE, oy, TAKEOFF_ALT, "WP1 Forward")
        if self._force_land:
            self._land(); return
        self._hold(ox + STEP_SIZE, oy, TAKEOFF_ALT)
        if self._force_land:
            self._land(); return

        # Step 9 — Right 1m
        self._go_to(ox + STEP_SIZE, oy - STEP_SIZE, TAKEOFF_ALT, "WP2 Right")
        if self._force_land:
            self._land(); return
        self._hold(ox + STEP_SIZE, oy - STEP_SIZE, TAKEOFF_ALT)
        if self._force_land:
            self._land(); return

        # Step 10 — Back 1m
        self._go_to(ox, oy - STEP_SIZE, TAKEOFF_ALT, "WP3 Back")
        if self._force_land:
            self._land(); return
        self._hold(ox, oy - STEP_SIZE, TAKEOFF_ALT)
        if self._force_land:
            self._land(); return

        # Step 11 — Left 1m back to origin
        self._go_to(ox, oy, TAKEOFF_ALT, "WP4 Origin", timeout=60.0)
        if self._force_land:
            self._land(); return
        self._hold(ox, oy, TAKEOFF_ALT)
        if self._force_land:
            self._land(); return

        # Step 12 — Land
        self.get_logger().info("=== MISSION COMPLETE — LANDING ===")
        self._land()

        # Wait for disarm
        self.get_logger().info("Waiting for disarm...")
        while self.state.armed:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self._force_land:
                break

        self.get_logger().info("Disarmed — done!")

    def force_land_and_exit(self):
        self.get_logger().warn("Force landing...")
        self._land()
        time.sleep(3.0)
        sys.exit(0)


def main():
    rclpy.init()
    node = SquareFlight()
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f"Exception: {e}")
        node._land()
    finally:
        if node._force_land:
            node.force_land_and_exit()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
