"""Autonomous yellow-line SHORT mission — Corner 1 only.

    ros2 launch viman_mission corner1.launch.py
    ros2 launch viman_mission corner1.launch.py start_detector:=false   # use your own detector
    ros2 launch viman_mission corner1.launch.py rtabmap_log:=info       # full RTAB detail

Flight: flow takeoff -> return to arena CENTRE + lock heading -> seed/validate
VIO -> climb to cruise_alt -> fly BACKWARD to the first (back) yellow line and
stop at the standoff -> strafe LEFT along it until the second (left) line = the
back-left corner (Corner 1) -> hold there (corner1_hold_s, default 20 s) ->
return HOME -> land. NO lawnmower survey.

Starts, each as its own parallel process:
  1. rs_pipeline               — RealSense colour+depth driver (optional)
  2. RTAB-Map stack            — VIO after handover
  3. vio_gate                  — validated, gated RTAB->PX4 bridge (gate CLOSED)
  4. yellow_boundary_detector  — OPTIONAL. Disable with start_detector:=false
                                 and run your own (manual) detector that
                                 publishes the same /viman/boundary/* topics.
  5. corner1_test_auto         — the SHORT mission state machine. Its ROS node
                                 name is 'boundary_test_auto' ON PURPOSE, so it
                                 reuses the tuned 'boundary_test_auto:' section
                                 of mission_params.yaml (cruise_alt, stop_dist,
                                 validate thresholds, etc.).

MAVROS is started separately, as usual.
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

    os.makedirs(MAP_DIR, exist_ok=True)
    db_path = os.path.join(
        MAP_DIR, f"corner1_{datetime.now().strftime('%Y%m%d_%H%M%S')}.db")

    rtabmap_launch = os.path.join(
        get_package_share_directory("rtabmap_launch"),
        "launch", "rtabmap.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_rtabmap", default_value="true",
            description="Include the RTAB-Map stack in this bringup"),
        DeclareLaunchArgument(
            "start_camera", default_value="true",
            description="Start the package's rs_pipeline camera node. Set "
                        "false if a camera driver runs separately — NEVER run "
                        "two camera drivers at once."),
        DeclareLaunchArgument(
            "start_detector", default_value="true",
            description="Start the package's yellow_boundary_detector. Set "
                        "false to run your OWN (manual) detector instead — it "
                        "must publish the same /viman/boundary/* topics."),
        DeclareLaunchArgument(
            "params_file", default_value=params_file,
            description="Mission/gate/detector parameter YAML"),
        DeclareLaunchArgument(
            "rtabmap_log", default_value="warn",
            description="RTAB-Map console verbosity: 'warn' (default) hides the "
                        "10 Hz per-frame Odom readout but keeps fault warnings; "
                        "'info' for a full-detail VIO-debugging run, 'error' "
                        "for near-silence."),

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

        # 3 — Yellow boundary detector (OPTIONAL — off => run your own)
        Node(
            package="viman_mission",
            executable="yellow_boundary_detector",
            name="yellow_boundary_detector",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration("start_detector")),
            parameters=[LaunchConfiguration("params_file")],
        ),

        # 4 — The SHORT Corner-1 mission. Node name kept as 'boundary_test_auto'
        #     so it reuses that tuned section of mission_params.yaml.
        Node(
            package="viman_mission",
            executable="corner1_test_auto",
            name="boundary_test_auto",
            output="screen",
            emulate_tty=True,
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
