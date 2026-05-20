from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    husky_bringup_pkg = FindPackageShare('husky_bringup')

    platform_launch = PathJoinSubstitution([husky_bringup_pkg, 'launch', 'platform', 'platform-service.launch.py'])
    sensors_launch = PathJoinSubstitution([husky_bringup_pkg, 'launch', 'sensors', 'sensors-service.launch.py'])

    # Include launch files
    launch_platform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([platform_launch]),
    )

    launch_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([sensors_launch]),
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_platform)
    ld.add_action(launch_sensors)
    return ld
