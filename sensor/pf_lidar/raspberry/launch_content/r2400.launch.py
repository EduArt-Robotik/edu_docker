import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()
    parameter_file = PathJoinSubstitution([
      './',
      'r2400_parameter.yaml'
    ])

    node = Node(
        package='pf_driver',
        name='pf_r2400',
        executable='ros_main',
        output='screen',
        parameters=[parameter_file]
    )

    ld.add_action(node)

    return ld
