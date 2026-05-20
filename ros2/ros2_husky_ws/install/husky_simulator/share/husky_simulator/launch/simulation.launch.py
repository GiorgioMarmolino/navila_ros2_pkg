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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import SetEnvironmentVariable

ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Gazebo World'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('enable_front_camera_color', default_value='false'),
    DeclareLaunchArgument('enable_front_camera_depth', default_value='false'),
    DeclareLaunchArgument('enable_rear_camera_color', default_value='false'),
    DeclareLaunchArgument('enable_rear_camera_depth', default_value='false'),
]

for pose_element in ['x', 'y', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))

ARGUMENTS.append(DeclareLaunchArgument('z', default_value='0.3',
                 description='z component of the robot pose.'))


def generate_launch_description():
    # Directories
    pkg_husky_simulator = get_package_share_directory(
        'husky_simulator')

    # Paths
    gz_sim_launch = PathJoinSubstitution(
        [pkg_husky_simulator, 'launch', 'gz_sim.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_husky_simulator, 'launch', 'robot_spawn.launch.py'])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('world', LaunchConfiguration('world')),
            ('rviz', LaunchConfiguration('rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw'))]
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(SetEnvironmentVariable('ENABLE_FRONT_CAMERA_COLOR', LaunchConfiguration('enable_front_camera_color')))
    ld.add_action(SetEnvironmentVariable('ENABLE_FRONT_CAMERA_DEPTH', LaunchConfiguration('enable_front_camera_depth')))
    ld.add_action(SetEnvironmentVariable('ENABLE_REAR_CAMERA_COLOR', LaunchConfiguration('enable_rear_camera_color')))
    ld.add_action(SetEnvironmentVariable('ENABLE_REAR_CAMERA_DEPTH', LaunchConfiguration('enable_rear_camera_depth')))
    ld.add_action(gz_sim)
    ld.add_action(robot_spawn)
    return ld
