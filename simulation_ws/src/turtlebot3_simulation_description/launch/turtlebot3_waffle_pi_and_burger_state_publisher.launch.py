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
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchIntrospector
from launch import LaunchService
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch_ros.actions import Node
from launch_ros import get_default_launch_description


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name1 = 'turtlebot3_waffle_pi.urdf'
    urdf_file_name2 = 'turtlebot3_burger.urdf'
    print("urdf_file_name : {}".format(urdf_file_name1))
    print("urdf_file_name : {}".format(urdf_file_name2))
    urdf1 = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', urdf_file_name1)
    urdf2 = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', urdf_file_name2)
 
    # Launch Description
    ld = LaunchDescription()

    # Prepare the Robot State Publisher node for waffle_pi
    rsp_node_wp = Node(
        node_name = 'robot_state_publisher_waffle_pi',
        
        package = 'robot_state_publisher',
        node_executable = 'robot_state_publisher',
        output = 'screen',
        arguments = [urdf1, 'robot_description:=robot_state_publisher_waffle_pi']
    )
    rsp_node_b = Node(
        node_name = 'robot_state_publisher_burger',
        package = 'robot_state_publisher',
        node_namespace = '/tb3_burger',
        node_executable = 'robot_state_publisher',
        output = 'screen',
        arguments = [urdf2, 'robot_description:=robot_state_publisher_burger']
    )

    ld.add_action( rsp_node_wp )
    ld.add_action( rsp_node_b )

    return ld

   
