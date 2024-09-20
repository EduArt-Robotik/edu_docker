#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch File Arguments
    edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
    edu_robot_namespace_arg = DeclareLaunchArgument('edu_robot_namespace', default_value=os.getenv('EDU_ROBOT_NAMESPACE', default='eduard'))

    channel_type = LaunchConfiguration('channel_type', default='tcp')
    tcp_ip = LaunchConfiguration('tcp_ip', default='192.168.0.7')
    tcp_port = LaunchConfiguration('tcp_port', default='20108') 
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')

    return LaunchDescription([

        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),

        DeclareLaunchArgument(
            'tcp_ip',
            default_value=tcp_ip,
            description='Specifying tcp ip to connected lidar'),

        DeclareLaunchArgument(
            'tcp_port',
            default_value=tcp_port,
            description='Specifying tcp port to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),
        edu_robot_namespace_arg,
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            namespace=edu_robot_namespace,
            parameters=[{'channel_type': channel_type,
                         'tcp_ip': tcp_ip,
                         'tcp_port': tcp_port,
                         'frame_id': frame_id,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate}],
            output='screen'),
    ])


