from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        'config',
        default_value='default.yaml',
        description='Path to the YAML parameters file in the config folder'
    )

    params_file_path = PathJoinSubstitution([
        FindPackageShare('whycode'),
        'config',
        LaunchConfiguration('config')
    ])

    node = Node(
        package='whycode',
        executable='whycode_node',
        name='whycode_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file_path],
        # prefix=['konsole -e gdb -ex run --args']
    )

    return LaunchDescription([
        params_file_arg,
        node
    ])
