"""Launch the autonomous mission.

    ros2 launch viman_mission mission.launch.py
    ros2 launch viman_mission mission.launch.py params_file:=/path/to/overrides.yaml

auto_mission itself launches RTAB-Map and vision_bridge mid-flight
(INIT_CAM phase), so only the mission node starts here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("viman_mission"),
        "config", "mission_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="YAML file with mission parameters"),
        Node(
            package="viman_mission",
            executable="auto_mission",
            name="auto_mission",
            output="screen",
            emulate_tty=True,   # keep colored logs + 1 Hz telemetry table
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
