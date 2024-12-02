import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushROSNamespace
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    # Get the share directory of the package
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Update the params.yaml file path
    params_file_path = '/home/kousi/Documents/homies/src/homies5/config/params.yaml'

    # Launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_localization = LaunchConfiguration('use_localization')

    # Remapping transforms
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Handle parameter replacements
    replaced_params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': ('/', namespace)},
        condition=IfCondition(use_namespace)
    )

    configured_params = RewrittenYaml(
        source_file=replaced_params_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=True
    )

    # Set logging environment variable
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # Launch arguments
    declare_arguments = [
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument('use_namespace', default_value='false', description='Apply namespace to the stack'),
        DeclareLaunchArgument('slam', default_value='False', description='Run SLAM or not'),
        DeclareLaunchArgument('map', default_value='', description='Path to map YAML file'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('params_file', default_value=params_file_path,
                              description='Path to ROS 2 parameters file'),
        DeclareLaunchArgument('autostart', default_value='true', description='Automatically start Nav2 stack'),
        DeclareLaunchArgument('use_composition', default_value='True', description='Use composition'),
        DeclareLaunchArgument('use_respawn', default_value='False', description='Respawn nodes on crash'),
        DeclareLaunchArgument('log_level', default_value='info', description='Log level'),
        DeclareLaunchArgument('use_localization', default_value='True', description='Enable localization'),
    ]

    # Grouped actions
    bringup_cmd_group = GroupAction([
        PushROSNamespace(condition=IfCondition(use_namespace), namespace=namespace),
        Node(
            condition=IfCondition(use_composition),
            package='rclcpp_components',
            executable='component_container_isolated',
            name='nav2_container',
            output='screen',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_launch.py')),
            condition=IfCondition(PythonExpression([slam, ' and ', use_localization])),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'use_respawn': use_respawn,
                'params_file': replaced_params_file
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization_launch.py')),
            condition=IfCondition(PythonExpression(['not ', slam, ' and ', use_localization])),
            launch_arguments={
                'namespace': namespace,
                'map': map_yaml_file,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': replaced_params_file,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
                'container_name': 'nav2_container'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': replaced_params_file,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
                'container_name': 'nav2_container'
            }.items()
        )
    ])

    # Create launch description
    ld = LaunchDescription()

    # Add actions
    ld.add_action(stdout_linebuf_envvar)
    for arg in declare_arguments:
        ld.add_action(arg)
    ld.add_action(bringup_cmd_group)

    return ld
