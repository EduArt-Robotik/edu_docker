import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_namespace = os.environ['EDU_ROBOT_NAMESPACE']
    if len(robot_namespace) == 0: robot_namespace = '/eduard/'
    if robot_namespace[0] != '/': robot_namespace = '/' + robot_namespace
    if robot_namespace[len(robot_namespace) - 1] != '/': robot_namespace += '/'
    tf_prefix = robot_namespace[1:] if robot_namespace[0] == '/' else robot_namespace
    slam_params_file = LaunchConfiguration('slam_params_file')

    print('use robot namespace = ', robot_namespace)
    print('use tf prefix = ', tf_prefix)

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("slam_toolbox"),
                                   'config', 'mapper_params_online_sync.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'base_frame': tf_prefix + 'base_footprint',
        'map_frame': tf_prefix + 'map',
        'odom_frame': tf_prefix + 'odom',
        'scan_topic': robot_namespace + 'scan'
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file='./slam.yaml',
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    start_sync_slam_toolbox_node = Node(
        parameters=[
          configured_params,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace=robot_namespace,
        remappings=[('/map', robot_namespace + 'map')],
        output='screen'
      )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_sync_slam_toolbox_node)

    return ld
