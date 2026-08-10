"""Autonomous yellow-line CORNER mission — full ground bringup.

    ros2 launch viman_mission boundary_corner.launch.py
    ros2 launch viman_mission boundary_corner.launch.py start_camera:=false

Starts, each as its own parallel process:
  1. rs_pipeline          — RealSense colour+depth driver (camera init at 2 m)
  2. RTAB-Map stack       — robust flight preset (fused VIO after handover)
  3. vio_gate             — validated, gated RTAB→PX4 bridge (gate CLOSED)
  4. yellow_boundary_detector — per-line (front + side) potential-field
                           detector; publishes /viman/boundary/lines
  5. boundary_test_auto   — the corner mission state machine:
       flow takeoff 2 m → return home + lock ARM-TIME heading → seed/validate
       → VIO climb 3 m → forward to the front line (soft stop 0.7 m) → strafe
       LEFT → detect the second line → settle in the corner (0.7 m off both)
       → hover 30 s → return home → land.

MAVROS is started separately, as usual. The mission's preflight gate verifies
MAVROS / RTAB-Map / vio_gate / the boundary detector are all alive before it
will arm, so a forgotten terminal cannot cause a flight.

Live boundary view: open http://<jetson-ip>:8080 in a browser on the same
WiFi (front + side "blue" vectors + the red repulsion arrow).
"""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from viman_mission.rtabmap_config import robust_flight_launch_args

MAP_DIR = "/media/jetson/ROS2_SSD/maps"


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("viman_mission"),
        "config", "mission_params.yaml")

    os.makedirs(MAP_DIR, exist_ok=True)
    db_path = os.path.join(
        MAP_DIR, f"corner_{datetime.now().strftime('%Y%m%d_%H%M%S')}.db")

    rtabmap_launch = os.path.join(
        get_package_share_directory("rtabmap_launch"),
        "launch", "rtabmap.launch.py")

    # RTAB-Map is launched via its own rtabmap.launch.py, which silently
    # ignores unknown launch args (so the `log_level` arg can't be trusted to
    # quiet it). Instead, once the nodes are up, call their standard
    # set_logger_levels service to force the loggers to WARN (30) — this is
    # guaranteed to work on any node. Only the 10 Hz "Odom:" / per-second rate
    # INFO spam is suppressed; WARN/ERROR (faults) still print. Skipped when
    # rtabmap_log:=info/debug so a debugging run keeps the full stream.
    def _quiet_call(node, loggers):
        lv = ", ".join(f"{{name: '{n}', level: 30}}" for n in loggers)
        return ExecuteProcess(
            cmd=["ros2", "service", "call",
                 f"/rtabmap/{node}/set_logger_levels",
                 "rcl_interfaces/srv/SetLoggerLevels",
                 f"{{levels: [{lv}]}}"],
            output="log", shell=False)

    quiet_rtabmap = TimerAction(
        period=12.0,
        condition=IfCondition(LaunchConfiguration("quiet_rtabmap")),
        actions=[
            _quiet_call("rgbd_odometry",
                        ["rtabmap.rgbd_odometry", "rgbd_odometry"]),
            _quiet_call("rtabmap", ["rtabmap.rtabmap", "rtabmap"]),
        ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "quiet_rtabmap", default_value="true",
            description="After startup, force the RTAB-Map loggers to WARN via "
                        "their set_logger_levels service so only faults/"
                        "warnings print. Set false to keep RTAB's full output."),
        DeclareLaunchArgument(
            "start_rtabmap", default_value="true",
            description="Include the RTAB-Map stack in this bringup"),
        DeclareLaunchArgument(
            "start_camera", default_value="true",
            description="Start the package's rs_pipeline camera node. Set "
                        "false if a camera driver runs separately — NEVER run "
                        "two camera drivers at once."),
        DeclareLaunchArgument(
            "params_file", default_value=params_file,
            description="Mission/gate/detector parameter YAML"),
        DeclareLaunchArgument(
            "rtabmap_log", default_value="warn",
            description="RTAB-Map console verbosity. 'warn' (default) hides the "
                        "10 Hz per-frame Odom readout but keeps fault warnings; "
                        "use 'info' for a full-detail VIO-debugging run, "
                        "'error' for near-silence."),

        # 0 — RealSense pipeline (hardware-stamped, aligned, TF published)
        Node(
            package="viman_mission",
            executable="rs_pipeline",
            name="rs_pipeline",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration("start_camera")),
            parameters=[LaunchConfiguration("params_file")],
        ),

        # 1 — RTAB-Map (robust preset; fail-resets harmlessly until seeded)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch),
            condition=IfCondition(LaunchConfiguration("start_rtabmap")),
            launch_arguments=robust_flight_launch_args(
                db_path, LaunchConfiguration("rtabmap_log")),
        ),

        # 2 — VIO gate (separate process, gate closed until validated)
        Node(
            package="viman_mission",
            executable="vio_gate",
            name="vio_gate",
            output="screen",
            emulate_tty=True,
            parameters=[LaunchConfiguration("params_file")],
        ),

        # 3 — Yellow boundary detector (per-line front + side vectors)
        Node(
            package="viman_mission",
            executable="yellow_boundary_detector",
            name="yellow_boundary_detector",
            output="screen",
            emulate_tty=True,
            parameters=[LaunchConfiguration("params_file")],
        ),

        # 4 — The corner mission
        Node(
            package="viman_mission",
            executable="boundary_test_auto",
            name="boundary_test_auto",
            output="screen",
            emulate_tty=True,
            parameters=[LaunchConfiguration("params_file")],
        ),

        # 5 — Silence RTAB's routine INFO spam ~12 s after launch (faults still
        #     print). Runs only when quiet_rtabmap:=true (default).
        quiet_rtabmap,
    ])
