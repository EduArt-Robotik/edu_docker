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

  filter_param_file = PathJoinSubstitution(["./", "laser_filter.yaml",])


  node = Node(
    package='pf_driver',
    name='pf_r2400',
    executable='ros_main',
    output='screen',
    parameters=[
      parameter_file,
      {'frame_id': PathJoinSubstitution([edu_robot_namespace, 'pf_lidar'])}
    ],
    namespace=edu_robot_namespace
  )

  laser_filter = Node(
    package="laser_filters",
    executable="scan_to_scan_filter_chain",
    namespace=edu_robot_namespace,
    parameters=[
      filter_param_file,
      {"filter1.params.box_frame": PathJoinSubstitution([edu_robot_namespace, 'base_link'])}
    ],
    remappings=[
      ('scan', 'scan/raw'),
      ('scan_filtered', 'scan')
    ],
  )

  transform_laser = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
      '0.12', '0.0', '0.15', '0.0', '0', '0',
      PathJoinSubstitution([edu_robot_namespace, 'base_link']),
      PathJoinSubstitution([edu_robot_namespace, 'pf_lidar'])
    ]
  )    

  ld.add_action(edu_robot_namespace_arg)
  ld.add_action(node)
  ld.add_action(laser_filter)
  ld.add_action(transform_laser)

  return ld
