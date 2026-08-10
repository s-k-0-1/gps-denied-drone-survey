"""One-command bringup for the BOUNDARY-BOUNDED SURVEY mission.

    ros2 launch viman_mission survey_boundary.launch.py

Starts the whole stack in parallel, on the ground, exactly like
bringup.launch.py — plus the two pieces this mission needs:

  0. rs_pipeline              RealSense camera        [start_camera]
  1. RTAB-Map stack           robust flight preset    [start_rtabmap]
  2. vio_gate                 gated RTAB→PX4 bridge
  3. whycode_detector         /whycode_node/markers   [start_whycode]
  4. yellow_boundary_detector /viman/boundary/*       [start_yellow_boundary]
  5. survey_boundary_director the mission (reuses SurveyMission +
                              the /viman/boundary/* topics)

WHY A SEPARATE FILE (and not just bringup mission_node:=…)
──────────────────────────────────────────────────────────
survey_boundary_director SUBCLASSES survey_mission.SurveyMission, so it
must run under the node name "survey_mission" to pick up your tuned
`survey_mission:` block in mission_params.yaml. bringup.launch.py names
its mission node after the executable, which would rename this node to
"survey_boundary_director" and silently drop that tuning. Here we start
it with executable="survey_boundary_director" but name="survey_mission",
so all your survey parameters apply unchanged. bringup.launch.py itself
is left untouched.

MAVROS + the RealSense driver are started the usual way (not here) —
same as bringup. The director's inherited preflight gate blocks arming
until they're alive.

Overrides (same names as bringup where they overlap):
  start_camera:=false        run a camera driver separately
  start_rtabmap:=false       run the RTAB-Map stack separately
  start_whycode:=false       run a real WhyCode node separately
  start_yellow_boundary:=false   disable the boundary detector (then the
                                 survey degrades to hardcoded extents only)
  params_file:=/path/to.yaml override the parameter file
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
from launch_ros.parameter_descriptions import ParameterValue

from viman_mission.rtabmap_config import robust_flight_launch_args

MAP_DIR = "/media/jetson/ROS2_SSD/maps"


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("viman_mission"),
        "config", "mission_params.yaml")

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
            "start_whycode", default_value="true",
            description="Start the whycode_detector node that publishes "
                        "/whycode_node/markers for precision landing."),
        DeclareLaunchArgument(
            "start_yellow_boundary", default_value="true",
            description="Start the yellow_boundary_detector node "
                        "(publishes /viman/boundary/*). Keep true — the "
                        "survey turns are driven by these topics."),
        DeclareLaunchArgument(
            "params_file", default_value=params_file,
            description="Mission/gate/detector parameter YAML"),
        DeclareLaunchArgument(
            "boundary_start_corner", default_value="back_left",
            description="Where the drone is placed before arming: "
                        "'back_left' (step RIGHT, end at RIGHT line), "
                        "'back_right' (step LEFT, end at LEFT line), "
                        "'center' (start at arena MIDDLE — fly BACK then LEFT to "
                        "find the back-left corner, re-yaw, then survey as "
                        "back_left), or 'auto'. Overrides the YAML. "
                        "Usage: boundary_start_corner:=center"),

        # 0 — RealSense pipeline
        Node(
            package="viman_mission",
            executable="rs_pipeline",
            name="rs_pipeline",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration("start_camera")),
            parameters=[LaunchConfiguration("params_file")],
        ),

        # 1 — RTAB-Map (robust preset)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch),
            condition=IfCondition(LaunchConfiguration("start_rtabmap")),
            launch_arguments=robust_flight_launch_args(db_path),
        ),

        # 2 — VIO gate
        Node(
            package="viman_mission",
            executable="vio_gate",
            name="vio_gate",
            output="screen",
            emulate_tty=True,
            parameters=[LaunchConfiguration("params_file")],
        ),

        # 3 — WhyCode detector (precision-landing markers)
        Node(
            package="viman_mission",
            executable="whycode_detector",
            name="whycode_detector",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration("start_whycode")),
            parameters=[LaunchConfiguration("params_file")],
        ),

        # 4 — Yellow boundary detector (publishes /viman/boundary/*)
        Node(
            package="viman_mission",
            executable="yellow_boundary_detector",
            name="yellow_boundary_detector",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration("start_yellow_boundary")),
            parameters=[LaunchConfiguration("params_file")],
        ),

        # 5 — Bounded survey director.
        #     executable is the NEW coordinator, but the node NAME is pinned
        #     to "survey_mission" so it reads the `survey_mission:` params
        #     block (it subclasses SurveyMission and reuses that tuning).
        #     The dict AFTER the params file overrides boundary_start_corner
        #     with the launch argument, so `boundary_start_corner:=back_right`
        #     on the command line wins over the YAML and the code default.
        Node(
            package="viman_mission",
            executable="survey_boundary_director",
            name="survey_mission",
            output="screen",
            emulate_tty=True,
            parameters=[
                LaunchConfiguration("params_file"),
                {"boundary_start_corner": ParameterValue(
                    LaunchConfiguration("boundary_start_corner"),
                    value_type=str)},
            ],
        ),
    ])
