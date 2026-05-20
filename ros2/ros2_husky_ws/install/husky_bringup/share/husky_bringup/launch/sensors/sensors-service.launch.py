from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Include Packages
    pkg_husky_bringup = FindPackageShare('husky_bringup')

    launch_file_lidar3d_0 = PathJoinSubstitution([pkg_husky_bringup, 'launch', 'sensors', 'lidar3d_0.launch.py'])
    launch_file_imu_0 = PathJoinSubstitution([pkg_husky_bringup, 'launch', 'sensors', 'imu_0.launch.py'])
    launch_file_gps_0 = PathJoinSubstitution([pkg_husky_bringup, 'launch', 'sensors', 'gps_0.launch.py'])

    # Include launch files
    launch_lidar3d_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_lidar3d_0]),
    )

    launch_imu_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_imu_0]),
    )

    launch_gps_0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_gps_0]),
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    # ld.add_action(launch_lidar3d_0)
    ld.add_action(launch_imu_0)
    # ld.add_action(launch_gps_0)
    return ld
