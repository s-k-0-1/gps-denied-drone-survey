#!/usr/bin/env python3
"""
vision_bridge — RTAB-Map odometry → MAVROS vision pose.

Subscribes /rtabmap/rtabmap/odom, validates each pose (NaN/Inf and
quaternion-norm checks), remaps the RTAB-Map camera frame into PX4's
ENU world frame, adds the world-frame offset captured at RTAB-Map init
time, throttles to 30 Hz, and publishes /mavros/vision_pose/pose.

Offset rationale: RTAB-Map odom always starts at (0,0,0) wherever it
inits. If the drone was at (ox, oy, oz) in MAVROS ENU when RTAB-Map
started, every published odom pose must be shifted by that amount so
PX4 EKF gets ground-relative poses (no Z-shift → no altitude overshoot).
Note: initial_pose on the SLAM node does NOT affect the odom node, so
this offset in vision_bridge is the real fix.

Parameters: offset_x, offset_y, offset_z (ENU metres),
            publish_rate_hz (default 30.0).
Coordinate frame notes: see viman_mission/common.py.
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from viman_mission.common import qos_reliable

# Rotation of -90° around Z, applied to every incoming quaternion.
_RZ_Z = -0.7071067811865476
_RZ_W = 0.7071067811865476


def _is_valid(msg: Odometry) -> bool:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    vals = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    if any(math.isnan(v) or math.isinf(v) for v in vals):
        return False

    norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    return abs(norm - 1.0) <= 0.01


def _remap_quaternion(qx, qy, qz, qw):
    """Rotate quaternion by -90° around Z (camera → ENU heading)."""
    nx = _RZ_W * qx - _RZ_Z * qy
    ny = _RZ_W * qy + _RZ_Z * qx
    nz = _RZ_W * qz + _RZ_Z * qw
    nw = _RZ_W * qw - _RZ_Z * qz
    return nx, ny, nz, nw


class VisionBridge(Node):
    def __init__(self):
        super().__init__("vision_bridge")

        self.declare_parameter("offset_x", 0.0)
        self.declare_parameter("offset_y", 0.0)
        self.declare_parameter("offset_z", 0.0)
        self.declare_parameter("publish_rate_hz", 30.0)
        self._offset_x = self.get_parameter("offset_x").value
        self._offset_y = self.get_parameter("offset_y").value
        self._offset_z = self.get_parameter("offset_z").value
        rate = self.get_parameter("publish_rate_hz").value

        qos = qos_reliable()

        self.pose_pub = self.create_publisher(
            PoseStamped, "/mavros/vision_pose/pose", qos)

        self.create_subscription(
            Odometry, "/rtabmap/rtabmap/odom", self.odom_callback, qos)

        self._last_pub    = self.get_clock().now()
        self._interval_ns = int(1e9 / rate)

        self._valid_count = 0
        self._nan_count   = 0

        self.get_logger().info(
            f"Vision bridge started — RTAB-Map → MAVROS  |  "
            f"offset=({self._offset_x:.3f}, {self._offset_y:.3f}, "
            f"{self._offset_z:.3f})")

    def odom_callback(self, msg: Odometry):
        if not _is_valid(msg):
            self._nan_count += 1
            if self._nan_count % 100 == 1:
                self.get_logger().warn(
                    f"Dropping invalid pose #{self._nan_count}")
            return

        self._nan_count = 0

        now = self.get_clock().now()
        if (now - self._last_pub).nanoseconds < self._interval_ns:
            return
        self._last_pub = now

        # Position remapping + world-frame offset. RTAB-Map odom starts at
        # (0,0,0) at init point; add the known PX4 ENU position at init
        # time so PX4 EKF gets ground-relative poses.
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        pose = PoseStamped()
        pose.header.stamp    = msg.header.stamp
        pose.header.frame_id = "map"
        pose.pose.position.x = -p.x + self._offset_x
        pose.pose.position.y =  p.y + self._offset_y
        pose.pose.position.z =  p.z + self._offset_z
        (pose.pose.orientation.x,
         pose.pose.orientation.y,
         pose.pose.orientation.z,
         pose.pose.orientation.w) = _remap_quaternion(q.x, q.y, q.z, q.w)

        self.pose_pub.publish(pose)

        self._valid_count += 1
        if self._valid_count % 50 == 1:
            self.get_logger().info(
                f"Pose #{self._valid_count}: "
                f"x={pose.pose.position.x:.3f} "
                f"y={pose.pose.position.y:.3f} "
                f"z={pose.pose.position.z:.3f}")


def main():
    rclpy.init()
    node = VisionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
