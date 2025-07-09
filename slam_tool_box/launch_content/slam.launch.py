import os
 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler, GroupAction
from launch.substitutions import EnvironmentVariable, PythonExpression
from launch_ros.actions import Node, LifecycleNode, SetParameter
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
    # Get Robot Namespace
    robot_namespace = os.environ.get('EDU_ROBOT_NAMESPACE', 'eduard')
    if len(robot_namespace) == 0: robot_namespace = '/eduard/'                          # default value
    if robot_namespace[0] != '/': robot_namespace = '/' + robot_namespace               # ensure preceding "/"
    if robot_namespace[len(robot_namespace) - 1] != '/': robot_namespace += '/'         # ensure trailing "/"
    tf_prefix = robot_namespace[1:] if robot_namespace[0] == '/' else robot_namespace   # removes preceding "/" for tf_prefix

    # Check if environment variable "MAP_FILE" is set. 
    # If set -> use AMCL + MapServer. 
    # If not set -> use SLAM
    map_file = os.environ.get('MAP_FILE', '').strip()
    use_amcl = PythonExpression(['"', map_file, '" != ""'])
    use_slam = PythonExpression(['not (', '"', map_file, '" != "")'])

    # Check if environment variable "FILTER_FILE" is set. 
    # If set -> use Keep-out-zone filter
    # If not set -> -
    filter_file = os.environ.get('FILTER_FILE', '').strip()
    use_filter = PythonExpression(['"', filter_file, '" != ""'])

 
    # Debug prints
    print('use robot namespace = ', robot_namespace)
    print('use tf prefix = ', tf_prefix)
    print('provided map file =', map_file)
    
    # Create launch configurations
    namespace               = LaunchConfiguration('namespace')
    autostart               = LaunchConfiguration('autostart')
    use_lifecycle_manager   = LaunchConfiguration("use_lifecycle_manager")
    use_sim_time            = LaunchConfiguration('use_sim_time')
    use_respawn             = LaunchConfiguration('use_respawn')
    log_level               = LaunchConfiguration('log_level')

    # Set launch configurations
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value=EnvironmentVariable('EDU_ROBOT_NAMESPACE', default_value="eduard"),
        description='Top-level namespace')
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the slamtoolbox. '
                    'Ignored when use_lifecycle_manager is true.')
    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='false',
        description='Enable bond connection during node activation')
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    
    # Nodes Managed by the NAV2 Lifecyclemanager
    lifecycle_nodes = ['map_server',
                       'amcl',
                       ]
    if filter_file != "":
        lifecycle_nodes += ['filter_mask_server', 'costmap_filter_info_server']

    # Remappings
    remappings = [('/tf', '/tf'),
                  ('/tf_static', '/tf_static'),
                  ('/map', robot_namespace + 'map')]

    # Create our own temporary YAML files that include substitutions
    slam_param_substitutions = {
        'base_frame': tf_prefix + 'base_footprint',
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'map_frame': tf_prefix + 'map',
        'odom_frame': tf_prefix + 'odom',
        'scan_topic': robot_namespace + 'scan',
        'frame_id': tf_prefix + 'map',
        'base_frame_id': tf_prefix + 'base_footprint',
        'global_frame_id': tf_prefix + 'map',
        'yaml_filename': map_file,
        'odom_frame_id': tf_prefix + 'odom'
    }
    # RewrittenYaml returns a path to a temporary YAML file with substitutions applied
    slam_configured_params = RewrittenYaml(
        source_file='./slam.yaml',
        root_key=EnvironmentVariable('EDU_ROBOT_NAMESPACE', default_value="eduard"),
        param_rewrites=slam_param_substitutions,
        convert_types=True)


    map_amcl_param_substitutions = {
        'base_frame_id': tf_prefix + 'base_footprint',
        'global_frame_id': tf_prefix + 'map',
        'odom_frame_id': tf_prefix + 'odom',
        'frame_id': tf_prefix + 'map',
        'yaml_filename': map_file

        
    }
    map_amcl_configured_params = RewrittenYaml(
        source_file='./nav2_map_amcl.yaml',
        root_key=EnvironmentVariable('EDU_ROBOT_NAMESPACE', default_value="eduard"),
        param_rewrites=map_amcl_param_substitutions,
        convert_types=True)
    
    filtermask_param_substitutions = {
        'frame_id': tf_prefix + 'map',
        'yaml_filename': filter_file
    }
    filtermask_configured_params = RewrittenYaml(
        source_file='./nav2_filtermask.yaml',
        root_key=EnvironmentVariable('EDU_ROBOT_NAMESPACE', default_value="eduard"),
        param_rewrites=filtermask_param_substitutions,
        convert_types=True)
 

    group_filter = GroupAction(
        condition=IfCondition(use_filter),
        actions=[
            LogInfo(msg="Using Nav2 Keep-Out Filter"),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='filter_mask_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[filtermask_configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
                namespace=namespace,
            ),
            Node(
                package='nav2_map_server',
                executable='costmap_filter_info_server',
                name='costmap_filter_info_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[filtermask_configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
                namespace=namespace,
            ),
        ]
    )

    # Map_Server + AMCL
    group_amcl = GroupAction(
        condition=IfCondition(use_amcl),
        actions=[
            LogInfo(msg="Starting Nav2 Map Server & AMCL"),
            LogInfo(msg="Lifecycle Nodes: "),
            LogInfo(msg=lifecycle_nodes),
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[map_amcl_configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
                namespace=namespace,
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[map_amcl_configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
                namespace=namespace,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}],
                namespace=namespace
            ),
            # Also start Filter-group (Keep-Out-Zones)
            group_filter
        ]
    )

    # SLAM Toolbox
     # creates SLAM Toolbox node
    start_sync_slam_toolbox_node = LifecycleNode(
        parameters=[
          slam_configured_params,
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
     # Adds configure event
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(start_sync_slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )
     # Adds activation event
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
     # Group everything in one action that gets executed on launch
    group_slam = GroupAction(
        condition=IfCondition(use_slam),
        actions=[
            LogInfo(msg="Starting SLAM"),
            SetParameter('use_sim_time', use_sim_time),
            start_sync_slam_toolbox_node,
            configure_event,
            activate_event
    ]
    )

    
    ld = LaunchDescription()
    # Launch Arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    
    # Nodes
    ld.add_action(group_amcl)
    ld.add_action(group_slam)
 
    return ld