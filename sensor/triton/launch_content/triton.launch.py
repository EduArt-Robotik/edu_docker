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
  triton_ip = LaunchConfiguration('triton_ip')
  triton_ip_arg = DeclareLaunchArgument(
    'triton_ip', default_value=os.getenv('TRITON_IP', default='192.168.0.71')
  )

  # Nodes
  ## Publish tf transform for Triton sensor
  transform_triton = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
      '0.20', '0.0', '0.0', '0.0', '0', '0',
      PathJoinSubstitution([edu_robot_namespace, 'base_link']),
      PathJoinSubstitution([edu_robot_namespace, 'triton'])
    ]
  )
  triton = Node(
    package='accerion_driver',
    executable='accerion_driver',
    name='accerion_driver',
    namespace=edu_robot_namespace,
    parameters=[{
      'sensor_link': PathJoinSubstitution([edu_robot_namespace, 'triton']),
      'global_parent_frame': PathJoinSubstitution([edu_robot_namespace, 'map']),
      'local_parent_frame': PathJoinSubstitution([edu_robot_namespace, 'odom']),
      'local_ip': triton_ip
    }]
  )

  return LaunchDescription([
    edu_robot_namespace_arg,
    triton_ip_arg,
    transform_triton,
    triton
  ])
  