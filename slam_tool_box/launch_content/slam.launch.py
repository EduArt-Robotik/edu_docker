import os
 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (AndSubstitution, LaunchConfiguration,
                                  NotSubstitution)

def generate_launch_description():
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")    
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_namespace = os.environ.get('EDU_ROBOT_NAMESPACE', '/eduard/')
    if len(robot_namespace) == 0: 
        robot_namespace = '/eduard/'
    if robot_namespace[0] != '/': 
        robot_namespace = '/' + robot_namespace
    if robot_namespace[len(robot_namespace) - 1] != '/': 
        robot_namespace += '/'
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
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the slamtoolbox. '
                    'Ignored when use_lifecycle_manager is true.')
    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='false',
        description='Enable bond connection during node activation')


    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'base_frame': tf_prefix + 'base_footprint',
        'map_frame': tf_prefix + 'map',
        'odom_frame': tf_prefix + 'odom',
        'scan_topic': robot_namespace + 'scan'
    }
 
    # RewrittenYaml returns a path to a temporary YAML file with substitutions applied
    configured_params = RewrittenYaml(
        source_file='./slam.yaml',
        root_key=EnvironmentVariable('EDU_ROBOT_NAMESPACE', default_value="eduard"),
        param_rewrites=param_substitutions,
        convert_types=True)
 
    start_sync_slam_toolbox_node = LifecycleNode(
        parameters=[
          configured_params,
          {
            'use_sim_time': use_sim_time,
            'use_lifecycle_manager': use_lifecycle_manager,
          }
        ],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace=EnvironmentVariable('EDU_ROBOT_NAMESPACE', default_value="eduard"),
        remappings=[('/map', robot_namespace + 'map')],
        output='screen'
      )
 
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(start_sync_slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_sync_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_sync_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    ld = LaunchDescription()
 
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_sync_slam_toolbox_node)
 
    return ld