# import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    navila_package = 'navila_ros2_bridge'

    # config_file = os.path.join(
    #     get_package_share_directory(navila_package),
    #     "config",
    #     "navila_config.yaml"
    # )

    inference_rate_hz = LaunchConfiguration("inference_rate_hz")
    use_phi3 = LaunchConfiguration("use_phi3")
    phi3_4bit = LaunchConfiguration("phi3_4bit")



    #============================================================================================
    #       LAUNCH DESCRIPTION
    #============================================================================================


    return LaunchDescription([

        DeclareLaunchArgument(
            "inference_rate_hz",
            default_value="1.0"
        ),
        DeclareLaunchArgument(
            "use_phi3",
            default_value="false"
        ),
        DeclareLaunchArgument(
            "phi3_4bit",
            default_value="false"
        ),



        #========================================================================================
        # Node 1: NaVILA bridge node
        Node(
            package=navila_package,
            executable='navila_super_node',
            name='navila_super_node',
            namespace='',
            output='screen',
            # remappings=[
            #     ('/camera/image_raw', '/sensors/front_camera/color/image_raw'),
            #     ('/odom', '/platform/odom'),
            # ],
            parameters=[
                # config_file,
                {
                "use_phi3": use_phi3,
                "inference_rate_hz": inference_rate_hz,
                "phi3_4bit": phi3_4bit,
                }
            ],
        ),

        #========================================================================================
        # Node 2: Action to cmd_vel converter
        Node(
            package=navila_package,
            executable='action_to_cmdvel_node',
            name='action_to_cmdvel_node',
            namespace='',
            output='screen',
            ),

        #========================================================================================
        #Node 3: Instruction publisher
        # ExecuteProcess(
        #     cmd=[
        #         'xterm',
        #         '-title', 'NaVILA Goal Input',
        #         '-e', 'bash -c "source /opt/ros/humble/setup.bash && '
        #             'source /home/ros_ws/install/setup.bash && '
        #             'ros2 run navila_ros2_bridge instruction_node; '
        #             'echo DONE; read"'
        #     ],
        #     output='screen',
        # ),
        Node(
            package=navila_package,
            executable='instruction_node',
            name='instruction_node',
            namespace='',
            output='screen',
        )
    ])