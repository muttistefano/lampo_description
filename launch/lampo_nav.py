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
from launch.actions import DeclareLaunchArgument
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
            parameters=[param_map])


    nav_sw1_params = os.path.join(
            get_package_share_directory('lampo_description'),
            'config',
            "nav_params_1.yaml")

    nav_sw1 = GroupAction(
        actions=[PushRosNamespace('sweepee_1'),
                IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'),"launch", 'navigation_launch.py')),
                launch_arguments={'namespace': "sweepee_1",
                                'use_sim_time': "true",
                                'autostart': "true",
                                'params_file': nav_sw1_params,
                                'use_lifecycle_mgr': 'false',
                                'map_subscribe_transient_local': 'true'}.items())])
    nodes_to_start = [map_server,nav_sw1]

    return LaunchDescription(nodes_to_start)