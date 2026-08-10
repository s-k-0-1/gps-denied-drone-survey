#!/usr/bin/env python3
"""
rtabmap_trigger — standalone altitude-triggered RTAB-Map launcher.

Alternative to the integrated auto_mission flow: watches
/mavros/local_position/pose, and once the drone holds above target_alt
for stable_secs, launches RTAB-Map with the exact ENU pose at that
moment as initial_pose (so RTAB-Map's frame is aligned with PX4's local
origin — without it the drone overshoots altitude by the launch height
and crash-lands). Once RTAB-Map covariance stays good for
good_quality_secs, launches vision_bridge with the same pose as offset.

RTAB-Map launch configuration is shared with auto_mission via
viman_mission/rtabmap_config.py.
"""

import os
import subprocess
from datetime import datetime

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from viman_mission.common import qos_best_effort, qos_reliable, rtab_cov_good
from viman_mission.rtabmap_config import (build_rtabmap_cmd,
                                          build_vision_bridge_cmd)


class RtabmapTrigger(Node):

    def __init__(self):
        super().__init__("rtabmap_trigger")

        # ── Parameters (defaults = previous hard-coded constants) ──
        self.declare_parameter("target_alt", 1.2)        # start near ground
        self.declare_parameter("stable_secs", 2.0)
        self.declare_parameter("good_quality_secs", 5.0)
        self.declare_parameter("map_dir", "/media/jetson/ROS2_SSD/maps")
        self._target_alt        = self.get_parameter("target_alt").value
        self._stable_secs       = self.get_parameter("stable_secs").value
        self._good_quality_secs = self.get_parameter("good_quality_secs").value
        self._map_dir           = self.get_parameter("map_dir").value

        self._above_since        = None
        self._rtabmap_launched   = False
        self._bridge_launched    = False
        self._good_quality_since = None
        self._launch_x = 0.0   # captured from PX4 at launch moment
        self._launch_y = 0.0
        self._launch_z = 0.0

        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose",
            self._pose_cb, qos_best_effort())

        self.create_subscription(
            Odometry, "/rtabmap/rtabmap/odom",
            self._odom_cb, qos_reliable())

        self.get_logger().info(
            f"Waiting for altitude > {self._target_alt}m for "
            f"{self._stable_secs}s to start RTAB-Map")

    def _pose_cb(self, msg: PoseStamped):
        if self._rtabmap_launched:
            return

        alt = msg.pose.position.z
        now = self.get_clock().now()

        if alt >= self._target_alt:
            if self._above_since is None:
                self._above_since = now
                self.get_logger().info(
                    f"Altitude {alt:.2f}m reached, "
                    f"waiting {self._stable_secs}s...")
            elif (now - self._above_since).nanoseconds / 1e9 >= self._stable_secs:
                self._rtabmap_launched = True
                # Capture EXACT position at launch time — used as
                # initial_pose so RTAB-Map's frame is aligned with PX4's
                # local origin.
                self._launch_x = msg.pose.position.x
                self._launch_y = msg.pose.position.y
                self._launch_z = msg.pose.position.z
                self.get_logger().info(
                    f"Launching RTAB-Map at ENU "
                    f"x={self._launch_x:.3f}(East) "
                    f"y={self._launch_y:.3f}(North) "
                    f"z={self._launch_z:.3f}(Up) — "
                    "waiting for good quality...")
                self._launch_rtabmap()
        else:
            if self._above_since is not None:
                self.get_logger().warn(
                    f"Altitude dropped to {alt:.2f}m, resetting timer")
            self._above_since = None

    def _odom_cb(self, msg: Odometry):
        if self._bridge_launched or not self._rtabmap_launched:
            return

        cov = msg.pose.covariance[0]
        now = self.get_clock().now()

        if rtab_cov_good(cov):
            if self._good_quality_since is None:
                self._good_quality_since = now
                self.get_logger().info("Good quality detected, confirming...")
            elif ((now - self._good_quality_since).nanoseconds / 1e9
                  >= self._good_quality_secs):
                self._bridge_launched = True
                self.get_logger().info(
                    "Quality confirmed — launching vision bridge!")
                self._launch_vision_bridge()
        else:
            # Only reset after sustained bad quality — brief dips ignored
            if self._good_quality_since is not None:
                elapsed = (now - self._good_quality_since).nanoseconds / 1e9
                if elapsed >= self._good_quality_secs:
                    self.get_logger().warn(
                        "Quality lost, resetting quality timer")
                    self._good_quality_since = None

    def _launch_rtabmap(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        db_path = f"{self._map_dir}/flight_{timestamp}.db"
        os.makedirs(self._map_dir, exist_ok=True)

        # Pass real-world starting pose so RTAB-Map publishes poses that
        # are ground-relative, not relative to its own init point.
        init_pose = (f"{self._launch_x:.4f} {self._launch_y:.4f} "
                     f"{self._launch_z:.4f} 0 0 0")
        self.get_logger().info(
            f"Saving map to {db_path}  |  initial_pose=[{init_pose}]")
        subprocess.Popen(build_rtabmap_cmd(db_path, init_pose))

    def _launch_vision_bridge(self):
        # PX4 ENU position at RTAB-Map init time → vision_bridge offset,
        # correcting the odom Z-shift before sending to PX4.
        self.get_logger().info(
            f"Launching vision_bridge — offset "
            f"({self._launch_x:.3f}, {self._launch_y:.3f}, "
            f"{self._launch_z:.3f})")
        subprocess.Popen(build_vision_bridge_cmd(
            self._launch_x, self._launch_y, self._launch_z))


def main():
    rclpy.init()
    node = RtabmapTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
