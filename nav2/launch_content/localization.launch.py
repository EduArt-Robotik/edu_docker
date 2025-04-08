import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  # robot namespace
  edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
  edu_robot_namespace_arg = DeclareLaunchArgument(
      'edu_robot_namespace', default_value=os.getenv('EDU_ROBOT_NAMESPACE', default='eduard')
  )

  # navigation
  localization_parameter = PathJoinSubstitution([
    './',
    'navigation.yaml'
  ])
  localization_launch_file = PathJoinSubstitution([
    FindPackageShare('nav2_bringup'),
    'launch',
    'localization_launch.py'
  ])
  localization = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(localization_launch_file),
    launch_arguments={
      'edu_robot_namespace': edu_robot_namespace,
      'params_file': localization_parameter
    }.items()
  )

  return LaunchDescription([
    edu_robot_namespace_arg,
    localization
  ])
