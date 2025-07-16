from launch import LaunchDescription

from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

def generate_launch_description():
    edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
    edu_robot_namespace_arg = DeclareLaunchArgument('edu_robot_namespace', default_value=os.getenv('EDU_ROBOT_NAMESPACE', 'eduard'))

    serial_port = LaunchConfiguration('serial_port')
    serial_port_arg = DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0')

    rplidar_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        PathJoinSubstitution([
          './',
          'launch',
          'rplidar_a2m8_launch.py'
        ])
      ]),
      launch_arguments={
        'serial_port' : serial_port, # actually it should be /dev/rplidar, but then a error code will be rise by the driver...
        'frame_id' : PathJoinSubstitution([edu_robot_namespace, 'laser'])
        # 'serial_baudrate' : '115200'
      }.items()
    )

    return LaunchDescription([
        edu_robot_namespace_arg,
        serial_port_arg,
        rplidar_node
    ])
