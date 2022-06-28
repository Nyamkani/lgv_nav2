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
# Author: kss

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    #common launch argument
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')


    #navigation2 launch arguments
    map_file_name = '6f.yaml'
    map_dir = LaunchConfiguration('map', default=os.path.join(get_package_share_directory('lgv_navigation'), 'map', map_file_name))
    nav_param_file_name = 'nav2_params.yaml'
    nav_param_dir = LaunchConfiguration('params_file', default=os.path.join(get_package_share_directory('lgv_navigation'),'param', nav_param_file_name))
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    auto_start = LaunchConfiguration('auto_start', default= 'true' )
    #slam_on = LaunchConfiguration('slam', default='True')               #this will not be used

    return LaunchDescription([

        #common launch argument
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),

        #navigation2 launch arguments
        DeclareLaunchArgument('map', default_value=map_dir, description='Full path to map file to load'),
        #DeclareLaunchArgument('slam', default_value=slam_on, description='NAV2 with launching slam'),                    #this will not be used
        DeclareLaunchArgument('params_file', default_value=nav_param_dir, description='Full path to navigation2 param file to load'),

        # IncludeLaunchDescription(PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
        #     launch_arguments={
        #         'map': map_dir,
        #         'use_sim_time': use_sim_time,
        #         'params_file': nav_param_dir,
        #         #'slam': slam_on,   #this will not be used
        #         'auto_start': auto_start, }.items(),
        # )
        IncludeLaunchDescription(PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav_param_dir,
                #'slam': slam_on,   #this will not be used
                'auto_start': auto_start, }.items(),
        )

    ])
