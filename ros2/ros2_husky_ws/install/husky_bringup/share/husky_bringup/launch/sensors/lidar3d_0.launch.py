from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Include Packages
    pkg_clearpath_sensors = FindPackageShare('clearpath_sensors')
    pkg_husky_bringup = FindPackageShare('husky_bringup')

    # Declare launch files
    launch_file_velodyne_lidar = PathJoinSubstitution([
        pkg_clearpath_sensors, 'launch', 'velodyne_lidar.launch.py']
    )

    config_file_lidar3d_0 = PathJoinSubstitution([pkg_husky_bringup, 'config', 'sensors', 'lidar3d_0.yaml'])

    # Include launch files
    launch_velodyne_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_velodyne_lidar]),
        launch_arguments=
            [
                (
                    'parameters'
                    ,
                    config_file_lidar3d_0
                )
                ,
                (
                    'namespace'
                    ,
                    'sensors/lidar3d_0'
                )
                ,
                (
                    'robot_namespace'
                    ,
                    '/'
                )
                ,
            ]
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_velodyne_lidar)
    return ld
