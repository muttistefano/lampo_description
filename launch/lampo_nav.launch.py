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


        map_yaml_file = os.path.join(
            get_package_share_directory('lampo_description'),
            'map',
            'map.yaml')

        param_map = {'yaml_filename': map_yaml_file}

        map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[param_map,{"use_sim_time": True}])


        nav_sw1_params = os.path.join(
            get_package_share_directory('lampo_description'),
            'config',
            "nav_params_1.yaml")


        amcl =  Node(
                package='nav2_amcl',
                namespace="sweepee_1",
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[nav_sw1_params],
                arguments=['--ros-args', '--log-level', "warn"])


        lifecycle_nodes = ["amcl",
                           'controller_server',
                        #    'smoother_server',
                           'planner_server',
                           'behavior_server',
                           'bt_navigator',
                        #    'waypoint_follower',
                        #    'velocity_smoother'
                        ]
        
        lifecycle_map   = ["map_server"]

        controller_server = Node(
                package='nav2_controller',
                executable='controller_server',
                namespace="sweepee_1",
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[nav_sw1_params],
                arguments=['--ros-args', '--log-level', "info"])
        
        nav2_smoother = Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                namespace="sweepee_1",
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[nav_sw1_params],
                arguments=['--ros-args', '--log-level', "info"])
        
        planner_server = Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                namespace="sweepee_1",
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[nav_sw1_params],
                arguments=['--ros-args', '--log-level', "info"])
        
        behavior_server = Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                namespace="sweepee_1",
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[nav_sw1_params],
                arguments=['--ros-args', '--log-level', "info"])
                
        bt_navigator = Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                namespace="sweepee_1",
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[nav_sw1_params],
                arguments=['--ros-args', '--log-level', "info"])
        
        nav2_waypoint_follower = Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                namespace="sweepee_1",
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[nav_sw1_params],
                arguments=['--ros-args', '--log-level', "info"])
        
        # Node(
        #         package='nav2_velocity_smoother',
        #         executable='velocity_smoother',
        #         name='velocity_smoother',
        #         output='screen',
        #         respawn=True,
        #         respawn_delay=2.0,
        #         parameters=[nav_sw1_params],
        #         arguments=['--ros-args', '--log-level', "info"],
        #         remappings=remappings +
        #                 [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        
        lf_manager = Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                namespace="sweepee_1",
                name='lifecycle_manager_navigation_nav',
                output='screen',
                arguments=['--ros-args', '--log-level', "info"],
                parameters=[{'autostart': True},
                            {'node_names': lifecycle_nodes}])

        lf_map    = Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation_map',
                output='screen',
                arguments=['--ros-args', '--log-level', "info"],
                parameters=[{'autostart': True},
                            {'node_names': lifecycle_map}])   

        nodes_to_start = [
                        map_server,
                        TimerAction(
                                period=1.0,
                                actions=[amcl,lf_map],
                        ),      
                        TimerAction(
                                period=2.0,
                                actions=[controller_server,planner_server,behavior_server,bt_navigator],
                        ),                   
                        TimerAction(
                                period=5.0,
                                actions=[lf_manager],
                        ),
                        # TimerAction(
                        # period=6.0,
                        # actions=[nav_sw2],
                        # ),
                        ]

        return LaunchDescription(nodes_to_start)