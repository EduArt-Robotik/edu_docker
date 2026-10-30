import os

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
  # robot namespace
  edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
  edu_robot_namespace_arg = DeclareLaunchArgument(
    'edu_robot_namespace', default_value=os.getenv('EDU_ROBOT_NAMESPACE', default='eduard')
  )

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
    parameters=[parameter_file],
    namespace=edu_robot_namespace
  )

  ld.add_action(edu_robot_namespace_arg)
  ld.add_action(node)

  return ld
