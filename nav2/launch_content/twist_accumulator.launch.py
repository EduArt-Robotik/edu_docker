import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Launch File Arguments
    robot_namespace = os.environ['EDU_ROBOT_NAMESPACE']
    if len(robot_namespace) == 0: robot_namespace = '/eduard/'
    if robot_namespace[0] != '/': robot_namespace = '/' + robot_namespace
    if robot_namespace[len(robot_namespace) - 1] != '/': robot_namespace += '/'

    # Twist Accumulation
    twist_accumulator = Node(
      package='edu_swarm',
      executable='twist_accumulator',
      name='twist_accumulator',
      namespace=robot_namespace,
      # parameters=[parameter_file],
      remappings=[
        ('twist/input_0', 'autonomous/nav2_cmd_vel'),
        ('twist/input_1', 'autonomous/null_cmd_vel'),
        ('twist/output', 'autonomous/cmd_vel')
      ],
      # prefix=['gdbserver localhost:3000'],
      output='screen'
    )

    null_cmd_vel_pub = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-r 20', robot_namespace + 'autonomous/null_cmd_vel', 'geometry_msgs/msg/Twist', ""],
        name='null_cmd_vel_pub',  # this is optional
        output='both',
    ),

    return LaunchDescription([
      twist_accumulator,
      null_cmd_vel_pub
    ])