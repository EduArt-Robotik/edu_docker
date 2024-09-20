from launch import LaunchDescription

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import Command, EnvironmentVariable, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    robot_namespace = EnvironmentVariable('EDU_ROBOT_NAMESPACE', default_value="eduard")

    rplidar_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        PathJoinSubstitution([
          './',
          'launch',
          'rplidar_a2m8_launch.py'
        ])
      ]),
      launch_arguments={
        'serial_port' : '/dev/ttyUSB0', # actually it should be /dev/rplidar, but then a error code will be rise by the driver...
        'frame_id' : PathJoinSubstitution([robot_namespace, 'laser'])
        # 'serial_baudrate' : '115200'
      }.items()
    )

    tf_laser = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=[
        '0.11', '0.0', '0.125', '3.141592654', '0', '0',
        PathJoinSubstitution([robot_namespace, 'base_link']),
        PathJoinSubstitution([robot_namespace, 'laser'])
      ]
    )
    return LaunchDescription([
        rplidar_node,
        tf_laser
    ])
