#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


def _is_valid(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation

    vals = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]

    if any(math.isnan(v) or math.isinf(v) for v in vals):
        return False

    norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    if abs(norm - 1.0) > 0.01:
        return False

    return True


def _remap_quaternion(qx, qy, qz, qw):
    # Rotation of -90° around Z
    rz_x = 0.0
    rz_y = 0.0
    rz_z = -0.7071067811865476
    rz_w = 0.7071067811865476

    nx = rz_w * qx + rz_x * qw + rz_y * qz - rz_z * qy
    ny = rz_w * qy - rz_x * qz + rz_y * qw + rz_z * qx
    nz = rz_w * qz + rz_x * qy - rz_y * qx + rz_z * qw
    nw = rz_w * qw - rz_x * qx - rz_y * qy - rz_z * qz

    return nx, ny, nz, nw


class VisionBridge(Node):
    def __init__(self):
        super().__init__("vision_bridge")

        # ── Offset: RTAB-Map odom always starts at (0,0,0) wherever it inits.
        # If the drone was at (ox, oy, oz) in MAVROS ENU frame when RTAB-Map
        # started, every published odom pose must be shifted by that amount
        # before sending to PX4 via /mavros/vision_pose/pose.
        #
        # Coordinate frame: ENU  (X=East, Y=North, Z=Up)
        # Origin: same as QGC home/arming point
        # MAVROS handles NED↔ENU conversion internally — we always work in ENU.
        #
        # Pass offsets via:
        #   --ros-args -p offset_x:=X -p offset_y:=Y -p offset_z:=Z
        # Note: initial_pose on the SLAM node does NOT affect the odom node,
        # so this offset in vision_bridge is the real fix.
        self.declare_parameter("offset_x", 0.0)
        self.declare_parameter("offset_y", 0.0)
        self.declare_parameter("offset_z", 0.0)
        self._offset_x = self.get_parameter("offset_x").value
        self._offset_y = self.get_parameter("offset_y").value
        self._offset_z = self.get_parameter("offset_z").value

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            "/mavros/vision_pose/pose",
            qos_reliable,
        )

        self.create_subscription(
            Odometry,
            "/rtabmap/rtabmap/odom",
            self.odom_callback,
            qos_reliable,
        )

        self._last_pub = self.get_clock().now()
        self._interval_ns = int(1e9 / 30.0)  # 30 Hz

        self._valid_count = 0
        self._nan_count = 0

        self.get_logger().info(
            f"Vision bridge started — RTAB-Map → MAVROS  |  "
            f"offset=({self._offset_x:.3f}, {self._offset_y:.3f}, {self._offset_z:.3f})"
        )

    def odom_callback(self, msg):
        if not _is_valid(msg):
            self._nan_count += 1

            if self._nan_count % 100 == 1:
                self.get_logger().warn(
                    f"Dropping invalid pose #{self._nan_count}"
                )
            return

        self._nan_count = 0

        now = self.get_clock().now()
        if (now - self._last_pub).nanoseconds < self._interval_ns:
            return

        self._last_pub = now

        # Position remapping + world-frame offset.
        # RTAB-Map odom starts at (0,0,0) at init point. We add the known
        # PX4 ENU position at init time so PX4 EKF gets ground-relative poses.
        px_x = -msg.pose.pose.position.x + self._offset_x
        px_y =  msg.pose.pose.position.y + self._offset_y
        px_z =  msg.pose.pose.position.z + self._offset_z

        # Quaternion remapping
        qx, qy, qz, qw = _remap_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = "map"

        pose.pose.position.x = px_x
        pose.pose.position.y = px_y
        pose.pose.position.z = px_z

        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.pose_pub.publish(pose)

        self._valid_count += 1

        if self._valid_count % 50 == 1:
            self.get_logger().info(
                f"Pose #{self._valid_count}: "
                f"x={px_x:.3f} "
                f"y={px_y:.3f} "
                f"z={px_z:.3f}"
            )


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
