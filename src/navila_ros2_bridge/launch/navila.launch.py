from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

        # Republisher raw > compressed to reduce bandwidth (NaVILA only needs compressed images)
        Node(
            package='image_transport',
            executable='republish',
            name='camera_republisher',
            arguments=['raw', 'compressed'],
            remappings=[
                ('in',             '/sensors/front_camera/color/image_raw'),
                ('out/compressed', '/sensors/front_camera/color/image_raw/compressed'),
            ],
            parameters=[{
                'jpeg_quality': 70,   # ~100KB/frame (VLA ~1Hz)
            }],
        ),

        # Node 1: NaVILA bridge node
        Node(
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
        ),

        # Node 2: Action to cmd_vel converter
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

        # Node 3: Instruction publisher
        Node(
            package=navila_package,
            executable='instruction_node',
            name='instruction_node',
            namespace='',
            output='screen',
            parameters=[{
                "use_sim_time": True,
            }],
        ),
    ])