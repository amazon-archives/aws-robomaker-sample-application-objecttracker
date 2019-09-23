# /*******************************************************************************
# * Copyright 2019 ROBOTIS CO., LTD.
# *
# * Licensed under the Apache License, Version 2.0 (the "License");
# * you may not use this file except in compliance with the License.
# * You may obtain a copy of the License at
# *
# *     http://www.apache.org/licenses/LICENSE-2.0
# *
# * Unless required by applicable law or agreed to in writing, software
# * distributed under the License is distributed on an "AS IS" BASIS,
# * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# * See the License for the specific language governing permissions and
# * limitations under the License.
# *******************************************************************************/

# /* Author: Darby Lim */

import os

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    object_tracker_world = get_package_share_directory('object_tracker_simulation')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    gazebo_client = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('gui')))
    gazebo_server = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')))

    turtlebot3_description_reduced_mesh_share_dir = get_package_share_directory('turtlebot3_description_reduced_mesh')
    turtlebot3_description_reduced_mesh_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_description_reduced_mesh_share_dir, 'launch', 'spawn_turtlebot.launch.py'))
    )
    turtlebot3_state_publisher_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_description_reduced_mesh_share_dir, 'launch', 'turtlebot3_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='world',
            default_value=[os.path.join(object_tracker_world, 'worlds', 'empty.world'), ''],
            description='SDF world file'),
        DeclareLaunchArgument(
            name='gui',
            default_value='false',
            description='Argument for GUI Display'
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false'
        ),
        DeclareLaunchArgument(
            name='state',
            default_value='true',
            description='Set "true" to load "libgazebo_ros_state.so"'),
        gazebo_server,
        gazebo_client,
        turtlebot3_description_reduced_mesh_launch,
        turtlebot3_state_publisher_launch
    ])
