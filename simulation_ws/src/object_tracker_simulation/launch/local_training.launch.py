# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this
# software and associated documentation files (the "Software"), to deal in the Software
# without restriction, including without limitation the rights to use, copy, modify,
# merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
# PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
# OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import os
import sys

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

def generate_launch_description():

    #############################
    ##  Local Training Worker  ##
    #############################

    ####################
    ##  GUI Argument  ##
    ####################
    gui = launch.actions.DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Argument for GUI Display')

    gazebo_ros = get_package_share_directory('gazebo_ros')
    gazebo_client = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('gui')))

    ###########################
    ##  Create World Launch  ##
    ###########################
    object_tracker_simulation_dir = get_package_share_directory('turtlebot3_description_reduced_mesh')
    object_tracker_simulation_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(object_tracker_simulation_dir, 'launch', 'turtlebot3_waffle_pi_and_burger.launch.py')))

    ###################################
    ##  Reinforcement Learning Node  ##
    ###################################
    rl_agent = launch_ros.actions.Node(
        package='object_tracker_simulation', node_executable='run_local_rl_agent.sh', output='screen',
        node_name='rl_agent', name='training')

    ########################
    ##  Launch Evalution  ##
    ########################
    ld = launch.LaunchDescription([gui, object_tracker_simulation_launch, rl_agent, gazebo_client])
    return ld


if __name__ == '__main__':
    generate_launch_description()
