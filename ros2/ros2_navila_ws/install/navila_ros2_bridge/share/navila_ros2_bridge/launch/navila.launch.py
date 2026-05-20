from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    navila_package = 'navila_ros2_bridge'
    return LaunchDescription([
        # Node 1: NaVILA bridge node
        Node(
            package=navila_package,
            executable='navila_node',
            name='navila_node',
            namespace='',
            output='screen',
            remappings=[
                ('/camera/image_raw', '/sensors/front_camera/color/image_raw'),
                ('/odom', '/platform/odom'),
            ],
        ),

        # Node 2: Action to cmd_vel converter
        Node(
            package=navila_package,
            executable='action_to_cmdvel_node',
            name='action_to_cmdvel_node',
            namespace='',
            output='screen',
            # se serve anche qui:
            # remappings=[('/cmd_vel', '/husky/cmd_vel')],
        ),

        #Node 3: Instruction publisher
        Node(
            package=navila_package,
            executable='instruction_node',
            name='instruction_node',
            namespace='',
            output='screen',
        ),
    ])