# Copyright 2019 Open Source Robotics Foundation, Inc.
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
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():



        nav_sw1_params = os.path.join(
            get_package_share_directory('lampo_description'),
            'config',
            "nav_params_omni_1.yaml")


        lifecycle_nodes = ['planner_server','bt_navigator']
        

        
        planner_server = Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                namespace="sweepee_1",
                output='screen',
                # prefix=['xterm -e gdb -ex run --args'],
                # prefix=['valgrind --tool=callgrind'],
                parameters=[nav_sw1_params],
                arguments=['--ros-args', '--log-level', "info"])
        

                
        bt_navigator = Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                namespace="sweepee_1",
                output='screen',
                # prefix=['xterm -e gdb -ex run --args'],
                respawn=True,
                respawn_delay=2.0,
                parameters=[nav_sw1_params],
                arguments=['--ros-args', '--log-level', "info"])

        
        lf_manager = Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                namespace="sweepee_1",
                name='lifecycle_manager_navigation_plan',
                output='screen',
                arguments=['--ros-args', '--log-level', "info"],
                parameters=[{'autostart': True},
                            {'node_names': lifecycle_nodes},
                            {"use_sim_time": True}])


        nodes_to_start = [
                        planner_server,
                        bt_navigator,
                        TimerAction(
                                period=1.0,
                                actions=[lf_manager],
                        )
                        ]

        return LaunchDescription(nodes_to_start)