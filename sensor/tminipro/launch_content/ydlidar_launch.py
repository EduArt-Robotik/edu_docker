#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os


def generate_launch_description():
    robot_namespace = EnvironmentVariable('EDU_ROBOT_NAMESPACE', default_value="eduard")
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    parameter_file = LaunchConfiguration('params_file')
    node_name = 'ydlidar_ros2_driver_node'

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'TminiPro.yaml'),       # TODO: Uses "original" param file, not the modified one with the different port and frame_id
                                           description='FPath to the ROS2 parameters file to use.')

    driver_node = LifecycleNode(package='ydlidar_ros2_driver',
                                executable=node_name,
                                name=node_name,
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file,
                                            # {"port": "/dev/tminipro"},        # Symlink doesn't work, uses default instead (/dev/ttyUSB0) 
                                            {"frame_id": PathJoinSubstitution([robot_namespace, 'laser'])}
                                            ],
                                namespace='/',
                                remappings=[
                                    ('/scan', PathJoinSubstitution([robot_namespace, 'scan'])),
                                    ('/point_cloud', PathJoinSubstitution([robot_namespace, 'point_cloud']))
                                ]
                                )
    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    arguments=['0.11', '0.0', '0.125','3.141592654', '0.0', '0.0', # TODO: Values only copied from rplidar - must be checked!
                               PathJoinSubstitution([robot_namespace, 'base_link']),
                               PathJoinSubstitution([robot_namespace, 'laser'])], 
                    )

    return LaunchDescription([
        params_declare,
        driver_node,
        tf2_node,
    ])
