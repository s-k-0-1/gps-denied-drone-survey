import subprocess
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

TARGET_ALT        = 1  # start RTAB-Map early near ground
STABLE_SECS       = 2.0
MIN_GOOD_QUALITY  = 50    # minimum quality to trust
GOOD_QUALITY_SECS = 3.0   # seconds of good quality before starting bridge

class RtabmapTrigger(Node):

    def __init__(self):
        super().__init__("rtabmap_trigger")
        self._above_since       = None
        self._rtabmap_launched  = False
        self._bridge_launched   = False
        self._good_quality_since = None

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._pose_cb,
            qos_sensor
        )

        self.create_subscription(
            Odometry,
            "/rtabmap/rtabmap/odom",
            self._odom_cb,
            qos_reliable
        )

        self.get_logger().info(
            f"Waiting for altitude > {TARGET_ALT}m for {STABLE_SECS}s to start RTAB-Map"
        )

    def _pose_cb(self, msg: PoseStamped):
        if self._rtabmap_launched:
            return

        alt = msg.pose.position.z
        now = self.get_clock().now()

        if alt >= TARGET_ALT:
            if self._above_since is None:
                self._above_since = now
                self.get_logger().info(f"Altitude {alt:.2f}m reached, waiting {STABLE_SECS}s...")
            elif (now - self._above_since).nanoseconds / 1e9 >= STABLE_SECS:
                self._rtabmap_launched = True
                self.get_logger().info("Launching RTAB-Map — waiting for good quality...")
                self._launch_rtabmap()
        else:
            if self._above_since is not None:
                self.get_logger().warn(f"Altitude dropped to {alt:.2f}m, resetting timer")
            self._above_since = None

    def _odom_cb(self, msg: Odometry):
        if self._bridge_launched or not self._rtabmap_launched:
            return

        cov = msg.pose.covariance[0]
        now = self.get_clock().now()

        # Valid pose: covariance is small and non-zero
        # Use persistent good quality — only reset if bad for multiple frames
        is_good = (cov > 0.0 and cov < 100.0)  # 99999 = lost, small = good

        if is_good:
            if self._good_quality_since is None:
                self._good_quality_since = now
                self.get_logger().info("Good quality detected, confirming...")
            elif (now - self._good_quality_since).nanoseconds / 1e9 >= GOOD_QUALITY_SECS:
                self._bridge_launched = True
                self.get_logger().info("Quality confirmed — launching vision bridge!")
                self._launch_vision_bridge()
        else:
            # Only reset if bad quality persists for more than 0.5s
            if self._good_quality_since is not None:
                elapsed = (now - self._good_quality_since).nanoseconds / 1e9
                if elapsed < GOOD_QUALITY_SECS:
                    pass  # ignore brief quality dips
                else:
                    self.get_logger().warn("Quality lost, resetting quality timer")
                    self._good_quality_since = None

    def _launch_rtabmap(self):
        subprocess.Popen([
            "ros2", "launch", "rtabmap_launch", "rtabmap.launch.py",
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

    def _launch_vision_bridge(self):
        subprocess.Popen([
            "python3",
            "/home/jetson/drone_ws/vision_bridge.py"
        ])


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
