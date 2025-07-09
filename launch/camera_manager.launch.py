import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    params_file = LaunchConfiguration("params_file")

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(get_package_share_directory("camera_manager"), "params", "config.yaml"),
        description="",
    )

    ld.add_action(declare_params_file_cmd)

    node = Node(
        package='camera_manager',
        # namespace="",
        executable='camera_manager',
        parameters=[
            params_file
        ],
        output="screen",
        arguments=['--ros-args', '--log-level', "info"],
        emulate_tty=True,
    )
    ld.add_action(node)

    return ld