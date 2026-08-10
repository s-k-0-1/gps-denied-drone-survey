#!/usr/bin/env python3
"""
precision_land — Autonomous precision landing on AprilTag GridBoard.
Team Viman Rakshak / IRoC-U 2026.

Place the board anywhere 10 cm – 2 m from the drone.
The drone takes off to target_alt, slowly rotates to find the board,
flies laterally to centre it under the camera, then descends and hands
off to AUTO.LAND.

Board: 4×4 AprilTag grid, marker 7.65 cm, gap 2 cm.
       Dictionary defaults to DICT_APRILTAG_36H11 — change via param
       if your board uses a different family.

Requires opencv-contrib-python ≥ 4.7 for GridBoard constructor.
Install: pip3 install opencv-contrib-python --break-system-packages

Tuning cam_x_sign / cam_y_sign
───────────────────────────────
After first flight, if the drone moves the WRONG way during APPROACH:
  • Drone moves left  when it should move right  → flip cam_x_sign (-1)
  • Drone moves back  when it should move forward → flip cam_y_sign (-1)
These depend on how the D455 is physically mounted (which edge faces front).

Run:
  ros2 run viman_mission precision_land
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import RCIn, State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import CameraInfo, Image

from viman_mission.common import qos_best_effort, qos_reliable

# ── Dictionary lookup ─────────────────────────────────────────────────
_DICT_MAP = {
    "DICT_APRILTAG_16H5":  cv2.aruco.DICT_APRILTAG_16H5,
    "DICT_APRILTAG_25H9":  cv2.aruco.DICT_APRILTAG_25H9,
    "DICT_APRILTAG_36H10": cv2.aruco.DICT_APRILTAG_36H10,
    "DICT_APRILTAG_36H11": cv2.aruco.DICT_APRILTAG_36H11,
    "DICT_4X4_50":         cv2.aruco.DICT_4X4_50,
    "DICT_5X5_100":        cv2.aruco.DICT_5X5_100,
}


class Phase:
    IDLE     = "IDLE"
    ARM      = "ARM"
    TAKEOFF  = "TAKEOFF"
    SEARCH   = "SEARCH"    # slow yaw rotation looking for board
    APPROACH = "APPROACH"  # lateral fly-to-centre
    DESCEND  = "DESCEND"   # centred descent, continuous correction
    LAND     = "LAND"      # AUTO.LAND handed off
    DONE     = "DONE"


class PrecisionLandMission(Node):

    def __init__(self):
        super().__init__("precision_land")

        # ── Parameters ───────────────────────────────────────────
        self.declare_parameters("", [
            # Flight
            ("target_alt",          2.5),    # takeoff / search / approach altitude (m)
            ("alt_tolerance",       0.12),   # ±m band to consider "at altitude"
            ("at_alt_confirm_s",    1.5),    # must hold altitude this long before search
            ("approach_speed_ms",   0.3),    # lateral speed during approach / descent (m/s)
            ("descend_speed_ms",    0.15),   # sink rate during descent (m/s)
            ("land_handoff_alt_m",  0.3),    # switch to AUTO.LAND below this altitude
            ("centering_thr_m",     0.15),   # board must be within this radius to start descent
            ("sp_rate_hz",          20.0),
            # RC
            ("rc_ch5_index",        4),
            ("rc_start_low",        1200),
            ("rc_interrupt_high",   1700),
            # Search
            ("search_yaw_dps",      15.0),   # yaw rotation rate during search (deg/s)
            ("detect_frames",       5),      # consecutive detections needed before acting
            # Board
            ("marker_size_m",       0.0765),
            ("marker_gap_m",        0.02),
            ("board_cols",          4),
            ("board_rows",          4),
            ("apriltag_dict",       "DICT_APRILTAG_36H11"),
            # Camera mounting signs — flip if approach goes wrong direction
            ("cam_x_sign",          1.0),
            ("cam_y_sign",         -1.0),    # image Y is usually inverted vs body
        ])

        def gp(name): return self.get_parameter(name).value

        self._target_alt      = gp("target_alt")
        self._alt_tol         = gp("alt_tolerance")
        self._at_alt_confirm  = gp("at_alt_confirm_s")
        self._approach_speed  = gp("approach_speed_ms")
        self._descend_speed   = gp("descend_speed_ms")
        self._land_alt        = gp("land_handoff_alt_m")
        self._center_thr      = gp("centering_thr_m")
        self._sp_rate         = gp("sp_rate_hz")
        self._rc_ch5          = int(gp("rc_ch5_index"))
        self._rc_start_low    = int(gp("rc_start_low"))
        self._rc_intr_high    = int(gp("rc_interrupt_high"))
        self._search_yaw_rate = math.radians(gp("search_yaw_dps"))
        self._detect_frames   = int(gp("detect_frames"))
        self._cam_x_sign      = float(gp("cam_x_sign"))
        self._cam_y_sign      = float(gp("cam_y_sign"))

        # ── AprilTag board ───────────────────────────────────────
        adict_id = _DICT_MAP.get(gp("apriltag_dict"), cv2.aruco.DICT_APRILTAG_36H11)
        adict    = cv2.aruco.getPredefinedDictionary(adict_id)
        self._board = cv2.aruco.GridBoard(
            (int(gp("board_cols")), int(gp("board_rows"))),
            float(gp("marker_size_m")),
            float(gp("marker_gap_m")),
            adict,
        )
        det_params = cv2.aruco.DetectorParameters()
        # Loosen corner refinement for small markers at distance
        det_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.ArucoDetector(adict, det_params)

        # ── Camera calibration (from camera_info) ────────────────
        self._K = None   # 3×3 intrinsics matrix
        self._D = None   # distortion coefficients

        # ── Mission state ─────────────────────────────────────────
        self._phase          = Phase.IDLE
        self._state          = State()
        self._pose           = PoseStamped()
        self._home_x         = 0.0
        self._home_y         = 0.0
        self._arm_yaw        = 0.0    # yaw locked at arm time (rad)
        self._search_yaw     = 0.0    # current commanded yaw during SEARCH (rad)
        self._bridge         = CvBridge()
        self._latest_image   = None
        self._arm_req        = False
        self._detect_count   = 0      # consecutive detection frames
        self._board_target_x = 0.0   # ENU target above board
        self._board_target_y = 0.0
        self._desc_z         = None
        self._at_alt_since   = None

        # ── ROS wiring ────────────────────────────────────────────
        cbg = ReentrantCallbackGroup()

        self.create_subscription(State, "/mavros/state",
            self._cb_state, qos_reliable(), callback_group=cbg)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
            self._cb_pose, qos_best_effort(), callback_group=cbg)
        self.create_subscription(RCIn, "/mavros/rc/in",
            self._cb_rc, qos_best_effort(), callback_group=cbg)
        self.create_subscription(Image, "/camera/camera/color/image_raw",
            self._cb_image, qos_best_effort(), callback_group=cbg)
        self.create_subscription(CameraInfo, "/camera/camera/color/camera_info",
            self._cb_caminfo, qos_reliable(), callback_group=cbg)

        self._pub_sp = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", qos_reliable())
        self._arm_cli  = self.create_client(
            CommandBool, "/mavros/cmd/arming", callback_group=cbg)
        self._mode_cli = self.create_client(
            SetMode, "/mavros/set_mode", callback_group=cbg)

        self.create_timer(1.0 / self._sp_rate, self._tick, callback_group=cbg)
        self.get_logger().info(
            "PrecisionLand ready — lower RC CH5 to arm and begin")

    # ── Subscribers ──────────────────────────────────────────────

    def _cb_state(self, msg):
        self._state = msg

    def _cb_pose(self, msg):
        self._pose = msg

    def _cb_caminfo(self, msg):
        if self._K is None:
            self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self._D = np.array(msg.d, dtype=np.float64)
            self.get_logger().info(
                f"Camera intrinsics loaded  fx={self._K[0,0]:.1f}  fy={self._K[1,1]:.1f}")

    def _cb_image(self, msg):
        try:
            self._latest_image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image convert: {e}", throttle_duration_sec=2.0)

    def _cb_rc(self, msg):
        if len(msg.channels) <= self._rc_ch5:
            return
        ch5 = msg.channels[self._rc_ch5]
        if ch5 <= self._rc_start_low and self._phase == Phase.IDLE:
            self._arm_req = True
        if ch5 >= self._rc_intr_high and self._phase not in (Phase.LAND, Phase.DONE):
            self.get_logger().warn("RC interrupt — MANUAL")
            self._set_mode("MANUAL")

    # ── Helpers ──────────────────────────────────────────────────

    def _set_mode(self, mode: str):
        req = SetMode.Request()
        req.custom_mode = mode
        self._mode_cli.call_async(req)

    def _yaw_rad(self) -> float:
        """Current drone yaw from pose quaternion (ENU, rad)."""
        q = self._pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def _pub_pose_sp(self, x: float, y: float, z: float, yaw_rad: float):
        sp = PoseStamped()
        sp.header.stamp    = self.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.x = x
        sp.pose.position.y = y
        sp.pose.position.z = z
        sp.pose.orientation.z = math.sin(yaw_rad / 2.0)
        sp.pose.orientation.w = math.cos(yaw_rad / 2.0)
        self._pub_sp.publish(sp)

    def _detect(self):
        """
        Run AprilTag board detection on the latest image.

        Returns (dx_enu, dy_enu) — the board's ENU offset FROM the drone
        (i.e. how far drone must move to be directly above the board).
        Returns None if board not found.

        Camera-to-ENU rotation:
          Camera is mounted looking DOWN.  Camera X / Y rotate into ENU
          using the drone's current yaw.  cam_x_sign / cam_y_sign correct
          for physical mounting orientation (tune empirically).
        """
        if self._latest_image is None or self._K is None:
            return None

        gray = cv2.cvtColor(self._latest_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)

        if ids is None or len(ids) == 0:
            return None

        # Get 3D↔2D correspondences for the detected subset of board markers
        obj_pts, img_pts = self._board.matchImagePoints(corners, ids)
        if obj_pts is None or len(obj_pts) < 4:
            return None

        ok, _, tvec = cv2.solvePnP(
            obj_pts, img_pts, self._K, self._D,
            flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None

        # tvec: board-centre position in camera optical frame (metres)
        #   X = right in image, Y = down in image, Z = depth (toward ground)
        tx = float(tvec[0]) * self._cam_x_sign
        ty = float(tvec[1]) * self._cam_y_sign

        # Rotate camera-frame lateral offset → ENU using drone yaw
        yaw = self._yaw_rad()
        dx_enu = math.cos(yaw) * tx - math.sin(yaw) * ty
        dy_enu = math.sin(yaw) * tx + math.cos(yaw) * ty
        return dx_enu, dy_enu

    def _step_toward(self, tx: float, ty: float, z: float):
        """
        Publish a setpoint one speed-limited step toward (tx, ty) at z.
        Returns remaining distance (m).
        """
        px = self._pose.pose.position.x
        py = self._pose.pose.position.y
        dx, dy = tx - px, ty - py
        dist   = math.hypot(dx, dy)
        step   = self._approach_speed / self._sp_rate
        if dist > step:
            nx = px + step * dx / dist
            ny = py + step * dy / dist
        else:
            nx, ny = tx, ty
        self._pub_pose_sp(nx, ny, z, self._arm_yaw)
        return dist

    # ── State machine ────────────────────────────────────────────

    def _tick(self):
        p = self._phase
        if   p == Phase.IDLE:     self._do_idle()
        elif p == Phase.ARM:      self._do_arm()
        elif p == Phase.TAKEOFF:  self._do_takeoff()
        elif p == Phase.SEARCH:   self._do_search()
        elif p == Phase.APPROACH: self._do_approach()
        elif p == Phase.DESCEND:  self._do_descend()
        # LAND / DONE: nothing to publish, AUTO.LAND is in control

    def _do_idle(self):
        # Keep publishing so OFFBOARD can be activated
        self._pub_pose_sp(self._home_x, self._home_y, self._target_alt, 0.0)
        if self._arm_req:
            self._arm_req = False
            self._set_mode("OFFBOARD")
            self._phase = Phase.ARM

    def _do_arm(self):
        self._pub_pose_sp(self._home_x, self._home_y, self._target_alt, 0.0)
        if not self._state.armed:
            req = CommandBool.Request()
            req.value = True
            self._arm_cli.call_async(req)
            return
        # Drone just armed — lock home and yaw
        self._home_x      = self._pose.pose.position.x
        self._home_y      = self._pose.pose.position.y
        self._arm_yaw     = self._yaw_rad()
        self._search_yaw  = self._arm_yaw
        self.get_logger().info(
            f"Armed  home=({self._home_x:.2f},{self._home_y:.2f})  "
            f"yaw={math.degrees(self._arm_yaw):.1f}°")
        self._phase = Phase.TAKEOFF

    def _do_takeoff(self):
        self._pub_pose_sp(
            self._home_x, self._home_y, self._target_alt, self._arm_yaw)
        alt = self._pose.pose.position.z
        self.get_logger().info(
            f"TAKEOFF  {alt:.2f}/{self._target_alt:.1f} m",
            throttle_duration_sec=1.0)

        if abs(alt - self._target_alt) <= self._alt_tol:
            if self._at_alt_since is None:
                self._at_alt_since = self.get_clock().now()
            elif (self.get_clock().now() - self._at_alt_since).nanoseconds \
                    >= self._at_alt_confirm * 1e9:
                self.get_logger().info("At altitude — SEARCH start")
                self._at_alt_since = None
                self._phase = Phase.SEARCH
        else:
            self._at_alt_since = None

    def _do_search(self):
        # Increment commanded yaw and hold position
        self._search_yaw += self._search_yaw_rate / self._sp_rate
        px = self._pose.pose.position.x
        py = self._pose.pose.position.y
        self._pub_pose_sp(px, py, self._target_alt, self._search_yaw)

        result = self._detect()
        if result is None:
            self._detect_count = 0
            self.get_logger().info(
                f"SEARCH  yaw={math.degrees(self._search_yaw):.0f}°  no board",
                throttle_duration_sec=0.5)
            return

        self._detect_count += 1
        dx, dy = result
        dist = math.hypot(dx, dy)
        self.get_logger().info(
            f"SEARCH  board seen  offset=({dx:+.2f},{dy:+.2f})m  "
            f"dist={dist:.2f}m  frame {self._detect_count}/{self._detect_frames}")

        if self._detect_count >= self._detect_frames:
            # Board confirmed — lock target ENU position
            self._board_target_x = px + dx
            self._board_target_y = py + dy
            self.get_logger().info(
                f"Board locked → ENU ({self._board_target_x:.2f},{self._board_target_y:.2f}) "
                f"— APPROACH")
            self._detect_count = 0
            self._phase = Phase.APPROACH

    def _do_approach(self):
        # Update target from live detection every frame
        result = self._detect()
        if result is not None:
            px = self._pose.pose.position.x
            py = self._pose.pose.position.y
            dx, dy = result
            self._board_target_x = px + dx
            self._board_target_y = py + dy

        dist = self._step_toward(
            self._board_target_x, self._board_target_y, self._target_alt)

        self.get_logger().info(
            f"APPROACH  dist={dist:.2f} m  "
            f"target=({self._board_target_x:.2f},{self._board_target_y:.2f})",
            throttle_duration_sec=0.5)

        if dist <= self._center_thr:
            self.get_logger().info(
                f"Board centred (dist={dist:.2f} m) — DESCEND")
            self._desc_z = self._target_alt
            self._phase  = Phase.DESCEND

    def _do_descend(self):
        # Re-detect and update lateral target every frame
        result = self._detect()
        if result is not None:
            px = self._pose.pose.position.x
            py = self._pose.pose.position.y
            dx, dy = result
            self._board_target_x = px + dx
            self._board_target_y = py + dy

        # Sink
        self._desc_z = max(0.0, self._desc_z - self._descend_speed / self._sp_rate)

        # Lateral correction step toward board centre
        px   = self._pose.pose.position.x
        py   = self._pose.pose.position.y
        dx   = self._board_target_x - px
        dy   = self._board_target_y - py
        dist = math.hypot(dx, dy)
        step = self._approach_speed / self._sp_rate
        if dist > step:
            nx = px + step * dx / dist
            ny = py + step * dy / dist
        else:
            nx, ny = self._board_target_x, self._board_target_y

        self._pub_pose_sp(nx, ny, self._desc_z, self._arm_yaw)

        alt = self._pose.pose.position.z
        self.get_logger().info(
            f"DESCEND  alt={alt:.2f} m  lat_err={dist:.2f} m",
            throttle_duration_sec=0.3)

        if alt <= self._land_alt:
            self.get_logger().info(
                f"Handoff to AUTO.LAND at alt={alt:.2f} m")
            self._set_mode("AUTO.LAND")
            self._phase = Phase.LAND


def main():
    rclpy.init()
    node = PrecisionLandMission()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
