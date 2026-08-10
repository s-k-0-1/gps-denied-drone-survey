"""Manual-flight yellow-boundary guard test.

    ros2 launch viman_mission boundary_guard.launch.py
    ros2 launch viman_mission boundary_guard.launch.py start_camera:=true

Starts, each as its own process:
  1. rs_pipeline               — RealSense colour+depth driver
  2. yellow_boundary_detector  — potential-field yellow-line detector
                                 (live browser feed on port 8080)
  3. boundary_guard            — POSCTL virtual fence (seizes OFFBOARD
                                 near the line, parks at the standoff,
                                 hands back)

CAMERA RULE — exactly ONE RealSense driver may run:
  · No other camera driver running → leave start_camera:=true
    (default). rs_pipeline owns the camera.
  · Your own camera driver already running → pass start_camera:=false.
    rs_pipeline is skipped; the detector uses your driver's images.
  Getting this wrong shows up as "failed to set power state" /
  "No device connected" from rs_pipeline, but the detector still works
  as long as SOME driver is publishing (check: Intrinsics + FPS > 0).

NO RTAB-Map / vio_gate here — this test flies on optical flow with the
pilot in POSITION mode; the camera is used only for boundary detection.
MAVROS is started separately, as usual.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("viman_mission"),
        "config", "mission_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file", default_value=params_file,
            description="Parameter YAML"),
        DeclareLaunchArgument(
            "start_camera", default_value="true",
            description="Start rs_pipeline. Set false if a camera driver "
                        "already runs — NEVER run two at once."),
        DeclareLaunchArgument(
            "show_window", default_value="false",
            description="Open a live OpenCV window with the camera feed "
                        "+ both vectors on THIS machine (needs a display; "
                        "headless Jetson → leave false and use "
                        "rqt_image_view on /viman/boundary/image_debug)"),

        Node(
            package="viman_mission",
            executable="rs_pipeline",
            name="rs_pipeline",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration("start_camera")),
            parameters=[LaunchConfiguration("params_file")],
        ),
        Node(
            package="viman_mission",
            executable="yellow_boundary_detector",
            name="yellow_boundary_detector",
            output="screen",
            emulate_tty=True,
            parameters=[
                LaunchConfiguration("params_file"),
                {"show_window": ParameterValue(
                    LaunchConfiguration("show_window"), value_type=bool)},
            ],
        ),
        Node(
            package="viman_mission",
            executable="boundary_guard",
            name="boundary_guard",
            output="screen",
            emulate_tty=True,
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
