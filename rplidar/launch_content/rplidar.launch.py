from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, EnvironmentVariable, PathJoinSubstitution

def generate_launch_description():
    robot_namespace = EnvironmentVariable('EDU_ROBOT_NAMESPACE', default_value="eduard")

    rplidar_node = Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,  # A1 / A2
            # 'serial_baudrate': 256000, # A3
            'frame_id': PathJoinSubstitution([robot_namespace, 'laser']),
            'inverted': False,
            'angle_compensate': True,
            'topic_name': 'scan'
        }],
        namespace=robot_namespace
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
