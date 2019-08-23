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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    waffle_pi_and_burger_model_file_name = 'turtlebot3_waffle_pi_and_burger.model'
    waffle_pi_and_burger_model_path = os.path.join(get_package_share_directory('turtlebot3_description_reduced_mesh'), 'worlds', waffle_pi_and_burger_model_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_description_reduced_mesh'), 'launch')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gzserver', '--verbose', waffle_pi_and_burger_model_path, '-s', 'libgazebo_ros_init.so', 'libgazebo_ros_state.so'],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/turtlebot3_waffle_pi_and_burger_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
