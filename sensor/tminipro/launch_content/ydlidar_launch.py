#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

import os


def generate_launch_description():
    edu_robot_namespace = LaunchConfiguration('edu_robot_namespace')
    edu_robot_namespace_arg = DeclareLaunchArgument('edu_robot_namespace', default_value=os.getenv('EDU_ROBOT_NAMESPACE', 'eduard'))

    parameter_file = LaunchConfiguration('params_file')
    node_name = 'ydlidar_ros2_driver_node'

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join('./', 'TminiPro.yaml'),
        description='Filepath to the ROS2 parameters file to use.'
    )

    driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable=node_name,
        name=node_name,
        output='screen',
        emulate_tty=True,
        parameters=[
            parameter_file,
            # {"port": "/dev/tminipro"},        # Symlink doesn't work, uses default instead (/dev/ttyUSB0) 
            {"frame_id": PathJoinSubstitution([edu_robot_namespace, 'laser'])}
        ],
        namespace=edu_robot_namespace
    )
    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=[
            "--x", "0.115",
            "--y", "0.0",
            "--z", "0.125",
            "--roll",  "0.0",
            "--pitch", "0.0",
            "--yaw",  "1.570796327", 
            "--frame-id", PathJoinSubstitution([edu_robot_namespace, 'base_link']),
            "--child-frame-id", PathJoinSubstitution([edu_robot_namespace, 'laser'])],
    )

    return LaunchDescription([
        edu_robot_namespace_arg,
        params_declare,
        driver_node,
        tf2_node
    ])
