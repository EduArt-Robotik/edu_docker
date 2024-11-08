import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  # Launch File Arguments
  edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
  edu_robot_namespace_arg = DeclareLaunchArgument(
    'edu_robot_namespace', default_value=os.getenv('EDU_ROBOT_NAMESPACE', default='eduard')
  )

  transform_laser = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
      '0.12', '0.0', '0.12', '0.0', '0', '0',
      PathJoinSubstitution([edu_robot_namespace, 'base_link']),
      'scanner_laser'
    ]
  )

  return LaunchDescription([
    edu_robot_namespace_arg,
    transform_laser
  ])
  