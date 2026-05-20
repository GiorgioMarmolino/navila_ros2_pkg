# Copyright 2023 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Gazebo World'),
    DeclareLaunchArgument('auto_start', default_value='true',
                          choices=['true', 'false'],
                          description='Auto-start Gazebo simulation'),
]


def gz_launch(context, *args, **kwargs):

    # Directories
    pkg_husky_simulator = get_package_share_directory(
        'husky_simulator')
    pkg_ros_gz_sim = get_package_share_directory(
        'ros_gz_sim')

    # Paths
    gz_sim_launch = PathJoinSubstitution(
        [pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    gui_config = PathJoinSubstitution(
        [pkg_husky_simulator, 'config', 'gui.config'])

    auto_start_option = ''
    auto_start = LaunchConfiguration('auto_start').perform(context)
    if (auto_start == 'true'):
        auto_start_option = ' -r'

    # Gazebo Simulator
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'),
                         '.sdf',
                         auto_start_option,
                         ' -v 1',
                         ' --gui-config ',
                         gui_config])
        ]
    )

    return [gz_sim]


def generate_launch_description():

    # Directories
    pkg_husky_simulator = get_package_share_directory('husky_simulator')

    # Determine all ros packages that are sourced
    packages_paths = [os.path.join(p, 'share') for p in os.getenv('AMENT_PREFIX_PATH').split(':')]

    # Set ignition resource path to include all sourced ros packages
    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_husky_simulator, 'worlds') + ':',
            os.path.join(pkg_husky_simulator, 'meshes') + ':',
            ':' + ':'.join(packages_paths)])

    # Clock bridge
    clock_bridge = Node(package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/sensors/lidar3d_0/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/sensors/lidar3d_0/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/sensors/imu_0/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/sensors/gps/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/front_camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/front_camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/front_camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/front_camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/front_camera/depth/image_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/rear_camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/rear_camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rear_camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/rear_camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rear_camera/depth/image_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
            '/model/robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ],
        remappings=[
            ('/sensors/gps/navsat', '/sensors/gps/fix'),
            ('/sensors/lidar3d_0/scan', '/sensors/lidar3d_0/scan'),
            ('/sensors/lidar3d_0/scan/points', '/velodyne_points'),
            ('/front_camera/color/camera_info', '/sensors/front_camera/color/camera_info'),
            ('/front_camera/color/image_raw', '/sensors/front_camera/color/image_raw'),
            ('/front_camera/depth/camera_info', '/sensors/front_camera/depth/camera_info'),
            ('/front_camera/depth/image_raw', '/sensors/front_camera/depth/image_raw'),
            ('/front_camera/depth/image_raw/points', '/sensors/front_camera/points'),
            ('/rear_camera/color/image_raw', '/sensors/rear_camera/color/image_raw'),
            ('/rear_camera/color/camera_info', '/sensors/rear_camera/color/camera_info'),
            ('/rear_camera/depth/camera_info', '/sensors/rear_camera/depth/camera_info'),
            ('/rear_camera/depth/image_raw', '/sensors/rear_camera/depth/image_raw'),
            ('/rear_camera/depth/image_raw/points', '/sensors/rear_camera/points'),
            ('/model/robot/tf', 'tf'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        parameters=[{'use_sim_time': True}]
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_sim_resource_path)
    ld.add_action(OpaqueFunction(function=gz_launch))
    ld.add_action(clock_bridge)
    return ld
