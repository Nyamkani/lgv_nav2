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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("lgv_slam"),
                                   'config', 'slam_toolbox_config.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        # executable='async_slam_toolbox_node',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_async_slam_toolbox_node)

    return ld






# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node


# def generate_launch_description():
#     #common launch argument
#     use_sim_time = LaunchConfiguration('use_sim_time', default='false')

#     #slam_toolbox launch arguments
#     #map_dir = LaunchConfiguration('map', default=os.path.join(get_package_share_directory('lgv_navigation'), 'map', 'testmap.yaml'))
#     slam_config_file_name = 'slam_toolbox_config.yaml'
#     slam_config_dir = LaunchConfiguration('params_file', default=os.path.join(get_package_share_directory('lgv_slam'),'config', slam_config_file_name))
#     slam_launch_file_dir = os.path.join(get_package_share_directory('slam_toolbox'), 'launch')

#     return LaunchDescription([

#         #common launch argument
#         DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),

#         #slam_toolbox launch arguments
#         #DeclareLaunchArgument('map', default_value=map_dir, description='Full path to map file to load'),
#         DeclareLaunchArgument('config', default_value=slam_config_dir, description='Full path to slam toolbox param file to load'),

#         IncludeLaunchDescription(PythonLaunchDescriptionSource([slam_launch_file_dir, '/online_async_launch.py']), #online_async_launch
#         #IncludeLaunchDescription(PythonLaunchDescriptionSource([slam_launch_file_dir, '/localization_launch.py']),
#              launch_arguments={
#                  #'map': map_dir,
#                  'use_sim_time': use_sim_time,
#                  'config': slam_config_file_name}.items(),
#         ),

#     ])
