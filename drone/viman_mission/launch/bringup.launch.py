"""Final-architecture ground bringup — everything starts on the ground,
nothing is spawned in the air.

    ros2 launch viman_mission bringup.launch.py
    ros2 launch viman_mission bringup.launch.py start_rtabmap:=false   # if run separately

Starts, each as its OWN parallel process:
  1. RTAB-Map stack (robust flight preset, full-res storage) — owned by
     the launch system, so Ctrl+C/shutdown delivers SIGINT and the .db
     is flushed cleanly. Never wrapped in a Python subprocess.
  2. vio_gate        — validated, gated RTAB→PX4 bridge (gate CLOSED).
  3. mission_director — preflight gate + mission state machine.

MAVROS and the RealSense driver are intentionally NOT included here —
start them the way you already do (their configs are setup-specific).
The mission_director's preflight gate verifies they're alive before
allowing arming, so a forgotten terminal can't cause a flight.

On the ground the downward camera sees nothing: RTAB odometry fail-
resets in a loop, harmlessly, until the drone is at altitude and the
mission seeds it. That is by design.
"""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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

    # DB path timestamped at bringup time (launch runs once, on the ground)
    os.makedirs(MAP_DIR, exist_ok=True)
    db_path = os.path.join(
        MAP_DIR, f"flight_{datetime.now().strftime('%Y%m%d_%H%M%S')}.db")

    rtabmap_launch = os.path.join(
        get_package_share_directory("rtabmap_launch"),
        "launch", "rtabmap.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_rtabmap", default_value="true",
            description="Include the RTAB-Map stack in this bringup"),
        DeclareLaunchArgument(
            "start_camera", default_value="true",
            description="Start the package's rs_pipeline camera node. "
                        "Set false if you run a camera driver separately "
                        "— NEVER run two camera drivers at once."),
        DeclareLaunchArgument(
            "params_file", default_value=params_file,
            description="Mission/gate parameter YAML"),
        DeclareLaunchArgument(
            "mission_node", default_value="mission_director",
            description="Which mission to fly: mission_director "
                        "(hover) or square_mission (1 m square)"),
        DeclareLaunchArgument(
            "start_whycode", default_value="true",
            description="Start the whycode_detector node that publishes "
                        "/whycode_node/markers for precision landing. "
                        "Set false if a real WhyCode node runs separately."),
        DeclareLaunchArgument(
            "start_boundary", default_value="false",
            description="Start yellow_boundary_detector (potential-field "
                        "yellow-line repulsion). Required for "
                        "mission_node:=boundary_test_auto. Default off to "
                        "save Jetson CPU on other missions."),

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

        # 1 — RTAB-Map (robust preset; runs from the ground, fail-resets
        #     harmlessly until seeded at altitude)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch),
            condition=IfCondition(LaunchConfiguration("start_rtabmap")),
            launch_arguments=robust_flight_launch_args(db_path),
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

        # 3 — WhyCode detector (publishes /whycode_node/markers for landing)
        Node(
            package="viman_mission",
            executable="whycode_detector",
            name="whycode_detector",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration("start_whycode")),
            parameters=[LaunchConfiguration("params_file")],
        ),

        # 3b — Yellow boundary detector (repulsion field for
        #      boundary_test_auto / any boundary-aware mission)
        Node(
            package="viman_mission",
            executable="yellow_boundary_detector",
            name="yellow_boundary_detector",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration("start_boundary")),
            parameters=[LaunchConfiguration("params_file")],
        ),

        # 4 — Mission node (selectable: hover or square)
        Node(
            package="viman_mission",
            executable=LaunchConfiguration("mission_node"),
            name=LaunchConfiguration("mission_node"),
            output="screen",
            emulate_tty=True,
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
