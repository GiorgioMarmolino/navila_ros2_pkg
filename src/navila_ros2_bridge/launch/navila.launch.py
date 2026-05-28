from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    navila_package = 'navila_ros2_bridge'

    navila_node = Node(
        package=navila_package,
        executable='navila_super_node',
        name='navila_super_node',
        output='screen',
        parameters=[{
            "use_sim_time":      True,
            "use_phi3":          LaunchConfiguration("use_phi3"),
            "inference_rate_hz": LaunchConfiguration("inference_rate_hz"),
            "phi3_4bit":         LaunchConfiguration("phi3_4bit"),
        }],
    )

    action_to_cmdvel_node = Node(
        package=navila_package,
        executable='action_to_cmdvel_node',
        name='action_to_cmdvel_node',
        output='log',
        parameters=[{
            "use_sim_time": True,
        }],
    )

    instruction_node = ExecuteProcess(
        cmd=[
            'xterm', '-title', 'NaVILA Goal Input', '-e',
            'bash -c "source /opt/ros/humble/setup.bash && '
            'source /home/ros_ws/install/setup.bash && '
            'ros2 run navila_ros2_bridge instruction_node; '
            'echo DONE; read"'
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument("inference_rate_hz", default_value="1.0"),
        DeclareLaunchArgument("use_phi3",          default_value="false"),
        DeclareLaunchArgument("phi3_4bit",         default_value="false"),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '105'),
        navila_node,
        action_to_cmdvel_node,
        instruction_node,
    ])