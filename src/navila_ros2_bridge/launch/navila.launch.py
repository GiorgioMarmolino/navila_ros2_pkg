from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    navila_package = 'navila_ros2_bridge'
    inference_rate_hz = LaunchConfiguration("inference_rate_hz")
    use_phi3          = LaunchConfiguration("use_phi3")
    phi3_4bit         = LaunchConfiguration("phi3_4bit")

    return LaunchDescription([
        DeclareLaunchArgument("inference_rate_hz", default_value="1.0"),
        DeclareLaunchArgument("use_phi3",          default_value="false"),
        DeclareLaunchArgument("phi3_4bit",         default_value="false"),

        # Avvia subito action_to_cmdvel e instruction
        Node(
            package=navila_package,
            executable='action_to_cmdvel_node',
            name='action_to_cmdvel_node',
            namespace='',
            output='screen',
            parameters=[{
                "use_sim_time": True,
            }],
        ),

        ExecuteProcess(
            cmd=[
                'xterm', '-title', 'NaVILA Goal Input', '-e',
                'bash -c "source /opt/ros/humble/setup.bash && '
                'source /home/ros_ws/install/setup.bash && '
                'ros2 run navila_ros2_bridge instruction_node; '
                'echo DONE; read"'
            ],
            output='screen',
        ),

        # Start navila_super_node after 4 secs
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package=navila_package,
                    executable='navila_super_node',
                    name='navila_super_node',
                    output='screen',
                    parameters=[{
                        "use_sim_time":      True,
                        "use_phi3":          use_phi3,
                        "inference_rate_hz": inference_rate_hz,
                        "phi3_4bit":         phi3_4bit,
                    }],
                ),
            ]
        ),
    ])