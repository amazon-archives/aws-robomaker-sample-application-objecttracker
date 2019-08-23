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
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory
from launch import LaunchIntrospector
from launch import LaunchService
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros import get_default_launch_description


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    waffle_pi_urdf_file_name = 'turtlebot3_waffle_pi.urdf'
    burger_urdf_file_name = 'turtlebot3_burger.urdf'

    waffle_pi_urdf_path = os.path.join(get_package_share_directory('turtlebot3_description_reduced_mesh'), 'urdf', waffle_pi_urdf_file_name)
    burger_urdf_path = os.path.join(get_package_share_directory('turtlebot3_description_reduced_mesh'), 'urdf', burger_urdf_file_name)

    # Launch Description
    ld = LaunchDescription()

    # Prepare the Robot State Publisher node for waffle_pi
    rsp_node_waffle_pi = Node(
        node_name='robot_state_publisher_waffle_pi',
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        output='screen',
        arguments=[waffle_pi_urdf_path, 'robot_description:=robot_state_publisher_waffle_pi']
    )
    rsp_node_burger = Node(
        node_name='robot_state_publisher_burger',
        package='robot_state_publisher',
        node_namespace='/tb3_burger',
        node_executable='robot_state_publisher',
        output='screen',
        arguments=[burger_urdf_path, 'robot_description:=robot_state_publisher_burger']
    )

    ld.add_action(rsp_node_waffle_pi)
    ld.add_action(rsp_node_burger)

    return ld
