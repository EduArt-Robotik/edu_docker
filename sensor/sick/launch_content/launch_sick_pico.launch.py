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

  # Launching needed processes for localization
  sick_scan_launch_file = PathJoinSubstitution([
    FindPackageShare('sick_scan_xd'),
    'launch',
    'sick_picoscan.launch.py'
  ])
  sick_scan = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(sick_scan_launch_file),
    launch_arguments={
      'hostname': '192.168.0.70',
      'udp_receiver_ip': '192.168.0.100',
      'publish_frame_id': edu_robot_namespace + '/laser'
    }.items()
  )

  transform_laser_point_cloud = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
      '0.12', '0.0', '0.12', '0.0', '0', '0',
      PathJoinSubstitution([edu_robot_namespace, 'base_link']),
      'world'
    ]
  )
  transform_laser = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
      '0.12', '0.0', '0.12', '0.0', '0', '0',
      PathJoinSubstitution([edu_robot_namespace, 'base_link']),
      PathJoinSubstitution([edu_robot_namespace, 'laser_1'])
    ]
  )  

  return LaunchDescription([
    edu_robot_namespace_arg,
    # sick_scan,
    transform_laser,
    transform_laser_point_cloud
  ])
  