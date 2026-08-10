"""Auto-calibrate the yellow HSV range for yellow_boundary_detector.

    ros2 launch viman_mission hsv_calibrate.launch.py
    ros2 launch viman_mission hsv_calibrate.launch.py duration_s:=60
    ros2 launch viman_mission hsv_calibrate.launch.py write_yaml:=false     # print only
    ros2 launch viman_mission hsv_calibrate.launch.py start_camera:=false   # camera already up

Starts the RealSense pipeline and the calibration node. HOLD the drone
~1–1.5 m directly over the yellow line for the whole window (default 45 s
— the longer window sees more lighting variation, giving a steadier fit).
At the end it prints the best hsv_low / hsv_high and (default) writes them into
the SOURCE mission_params.yaml with a timestamped .bak backup. Rebuild after:
    colcon build --packages-select viman_mission && source install/setup.bash
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_params = os.path.join(
        get_package_share_directory("viman_mission"),
        "config", "mission_params.yaml")
    src_params = os.path.expanduser(
        "~/drone_ws/src/viman_mission/config/mission_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_camera", default_value="true",
            description="Start the package's rs_pipeline camera node. Set false "
                        "if the camera is already running elsewhere."),
        DeclareLaunchArgument(
            "duration_s", default_value="45.0",
            description="Seconds to hold the drone over the line and collect "
                        "(45 s default — best detection: more lighting "
                        "variation sampled = steadier HSV fit)."),
        DeclareLaunchArgument(
            "write_yaml", default_value="true",
            description="Write the best hsv_low/hsv_high into the SOURCE "
                        "mission_params.yaml (with a .bak backup). false = "
                        "print only."),
        DeclareLaunchArgument(
            "params_file", default_value=src_params,
            description="SOURCE mission_params.yaml to update. Default assumes "
                        "~/drone_ws/src/viman_mission/config/mission_params.yaml"),

        # RealSense colour+depth pipeline (uses installed camera params)
        Node(
            package="viman_mission",
            executable="rs_pipeline",
            name="rs_pipeline",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration("start_camera")),
            parameters=[share_params],
        ),

        # HSV auto-calibration node
        Node(
            package="viman_mission",
            executable="hsv_calibrate",
            name="hsv_calibrate",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "duration_s": LaunchConfiguration("duration_s"),
                "write_yaml": LaunchConfiguration("write_yaml"),
                "params_file": LaunchConfiguration("params_file"),
            }],
        ),
    ])
