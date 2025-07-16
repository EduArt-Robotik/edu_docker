from launch import LaunchDescription

from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

def generate_launch_description():
    edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
    edu_robot_namespace_arg = DeclareLaunchArgument('edu_robot_namespace', default_value=os.getenv('EDU_ROBOT_NAMESPACE', 'eduard'))

    parameter_file = PathJoinSubstitution(["./", "laser_angle_filter.yaml",])

    tf_laser = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=[
        '0.11', '0.0', '0.125', '3.141592654', '0', '0',
        PathJoinSubstitution([edu_robot_namespace, 'base_link']),
        PathJoinSubstitution([edu_robot_namespace, 'laser'])
      ]
    )

    laser_filter = Node(
      package="laser_filters",
      executable="scan_to_scan_filter_chain",
      namespace=edu_robot_namespace,
      parameters=[
        parameter_file,
        {"filter1.params.box_frame": PathJoinSubstitution([edu_robot_namespace, 'base_link'])}
      ],
      remappings=[
        ('scan', 'scan/raw'),
        ('scan_filtered', 'scan')
      ],   
    )
    return LaunchDescription([
        edu_robot_namespace_arg,
        tf_laser,
        laser_filter
    ])
