from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    SetEnvironmentVariable, 
    ExecuteProcess, 
    TimerAction,
    LogInfo,
    RegisterEventHandler,
)

from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    navila_package = 'navila_ros2_bridge'

    inference_rate_hz = LaunchConfiguration("inference_rate_hz")
    use_phi3          = LaunchConfiguration("use_phi3")
    phi3_4bit         = LaunchConfiguration("phi3_4bit")

    # ===========================================
    # Node 1: NaVILA bridge node
    navila_node = Node(
        package=navila_package,
        executable='navila_super_node',
        name='navila_super_node',
        namespace='',
        output='screen',
        parameters=[{
            "use_sim_time":      True,
            "use_phi3":          use_phi3,
            "inference_rate_hz": inference_rate_hz,
            "phi3_4bit":         phi3_4bit,
        }],
    )
    # ===========================================
    action_to_cmdvel_node = Node(
        package=navila_package,
        executable='action_to_cmdvel_node',
        name='action_to_cmdvel_node',
        namespace='',
        output='log',
        parameters=[{
            "use_sim_time": True,
        }],
    )
    # ===========================================
    instruction_node = ExecuteProcess(
        cmd=[
            'xterm',
            '-title', 'NaVILA Goal Input',
            '-e',
            (
                'bash -c "'
                'source /opt/ros/humble/setup.bash && '
                'source /home/ros_ws/install/setup.bash && '
                'ros2 run navila_ros2_bridge instruction_node; '
                'echo DONE; '
                'read"'
            )
        ],
        output='screen',
    )



    return LaunchDescription([

        # Tutti i nodi su domain 105 — nessun domain_bridge necessario
        SetEnvironmentVariable('ROS_DOMAIN_ID', '105'),

        DeclareLaunchArgument("inference_rate_hz", default_value="1.0"),
        DeclareLaunchArgument("use_phi3",          default_value="false"),
        DeclareLaunchArgument("phi3_4bit",         default_value="false"),


        # STARTUP SEQUENCE:
        LogInfo(msg='[LAUNCH] Starting navila_super_node...'),
        navila_node,
        RegisterEventHandler(# LOG WHEN NODE 1 STARTS
            OnProcessStart(
                target_action=navila_node,
                on_start=[
                    LogInfo(msg='[LAUNCH] navila_super_node started'),
                    LogInfo(msg='[LAUNCH] Waiting 10 seconds before next node...'),
                ]
            )
        ),
        TimerAction(
            period=10.0,
            actions=[
                LogInfo(msg='[LAUNCH] Starting action_to_cmdvel_node...'),
                action_to_cmdvel_node,
            ]
        ),

        TimerAction(
            period=15.0,
            actions=[
                LogInfo(msg='[LAUNCH] Starting instruction_node...'),
                instruction_node,
            ]
        ),
    ])