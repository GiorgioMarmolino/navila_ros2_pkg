from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.conditions import IfCondition, UnlessCondition



def generate_launch_description():

    arg_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false',
                        choices=['true', 'false'],
                        description='use_sim_time')

    use_sim_time = LaunchConfiguration('use_sim_time')


    # Include Packages
    pkg_clearpath_common = FindPackageShare('clearpath_common')
    pkg_clearpath_diagnostics = FindPackageShare('clearpath_diagnostics')

    # Declare launch files
    launch_file_platform = PathJoinSubstitution([
        pkg_clearpath_common, 'launch', 'platform.launch.py'])
    launch_file_diagnostics = PathJoinSubstitution([
        pkg_clearpath_diagnostics, 'launch', 'diagnostics.launch.py'])

    # Include launch files
    launch_platform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_platform]),
        launch_arguments=
            [
                (
                    'use_sim_time'
                    ,
                    use_sim_time
                )
                ,
                (
                    'namespace'
                    ,
                    '/'
                )
                ,
            ]
    )

    launch_diagnostics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_diagnostics]),
        condition=UnlessCondition(use_sim_time)
    )

    # Nodes
    node_wireless_watcher = Node(
        name='wireless_watcher',
        executable='wireless_watcher',
        package='wireless_watcher',
        namespace='/',
        output='screen',
        parameters=
            [
                {
                    'hz': 1.0
                    ,
                    'dev': ''
                    ,
                    'connected_topic': 'platform/wifi_connected'
                    ,
                    'connection_topic': 'platform/wifi_status'
                    ,
                }
                ,
            ]
        ,
        condition=UnlessCondition(use_sim_time)
    )

    node_battery_state_estimator = Node(
        name='battery_state_estimator',
        executable='battery_state_estimator',
        package='clearpath_diagnostics',
        namespace='/',
        output='screen',
        arguments=
            [
                '-s'
                ,
                '/etc/clearpath/'
                ,
            ]
        ,
    )

    node_battery_state_control = Node(
        name='battery_state_control',
        executable='battery_state_control',
        package='clearpath_diagnostics',
        namespace='/',
        output='screen',
        arguments=
            [
                '-s'
                ,
                '/etc/clearpath/'
                ,
            ]
        ,
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(arg_use_sim_time)
    ld.add_action(launch_platform)
    ld.add_action(launch_diagnostics)
    ld.add_action(node_wireless_watcher)
    # ld.add_action(node_battery_state_estimator)
    # ld.add_action(node_battery_state_control)
    return ld
