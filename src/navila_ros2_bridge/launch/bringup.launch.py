import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    navila_package = 'navila_ros2_bridge'

    # Config file installed in share/<pkg>/config/
    # config_file = os.path.join(
    #     get_package_share_directory(navila_package),
    #     'config',
    #     'navila_params.yaml',
    # )

    # -------------------------------------------------------------------------------
    # Launch Configurations and Expressions
    enable_safety = LaunchConfiguration("enable_safety")
    env = LaunchConfiguration("env", default="sim")  # "sim" or "lab"

    #   safety ON  -> /cmd_vel_raw  (picked up by safety_layer_node)
    #   safety OFF -> /cmd_vel      (straight to twist_mux)
    action_out_topic = PythonExpression(["'/cmd_vel_raw' if '", enable_safety, "' == 'true' else '/cmd_vel'"])
    config_file = PythonExpression([
    "'",
    get_package_share_directory(navila_package),
    "/config/' + ('sim_navila_config.yaml' if '",
    env,
    "' == 'sim' else 'lab_navila_config.yaml')"
    ])
    # -------------------------------------------------------------------------------
    

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_safety",
            default_value="true",
            choices=["true", "false"],
            description="If true, start safety_layer_node between action node and "
                        "twist_mux; if false, action node publishes straight to /cmd_vel.",
        ),
        DeclareLaunchArgument(
            'env',
            default_value='sim',
            choices=['sim', 'lab'],
            description='Environment configuration'
        ),
        
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('husky_navigation'),
                'launch',
                'pointcloud_to_laserscan.launch.py'
                )
            ),
            launch_arguments={
                'config_file': config_file,
                }.items()
        ),

        # Action node: everything from config, except the dynamic output topic.
        Node(
            package=navila_package,
            executable='action_node',
            name='action_node',
            namespace='',
            output='screen',
            emulate_tty=True,
            parameters=[
                config_file,
                {"cmd_vel_topic": action_out_topic},
            ],
        ),

        # Safety layer: fully from config. Started only if enable_safety == true.
        Node(
            package=navila_package,
            executable='safety_layer_node',
            name='safety_layer_node',
            namespace='',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(enable_safety),
            parameters=[config_file],
        ),

        # Goal input terminal
        ExecuteProcess(
            cmd=[
                'xterm', '-title', 'NaVILA Goal Input', '-e',
                'bash -c "source /opt/ros/humble/setup.bash && '
                'source /home/ros_ws/install/setup.bash && '
                'ros2 run navila_ros2_bridge instruction_node; '
                'echo DONE; read"'
            ],
            output='screen',
            emulate_tty=True,
        ),

        # Start navila_super_node after 6 secs (params from config)
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package=navila_package,
                    executable='navila_super_node',
                    name='navila_super_node',
                    output='screen',
                    emulate_tty=True,
                    parameters=[config_file],
                ),
            ]
        ),
    ])