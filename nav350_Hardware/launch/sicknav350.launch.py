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

import launch
import launch_ros
import os

from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)



def generate_launch_description():
    #for declaring the common arguments
    #pkg_share = launch_ros.substitutions.FindPackageShare(package='lgv_nav350').find('lgv_nav350')
    
    Static_tf_map_to_odom_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='Static_tf_map_to_odom',
        arguments = ['0', '0', '0.68', '0', '0', '0', 'map', 'odom'],
    )

    Static_tf_nav350_to_base_link_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='Static_tf_nav350_to_base_link',
        arguments = ['0', '0', '0', '0', '0', '0', 'nav350', 'base_link'],
    )

    NAV350_node = launch_ros.actions.Node(
        package='nav350_hardware',
        executable='nav350_Hardware',
        name='nav350_Hardware',
        output='screen',
        parameters=[
            {"ipaddress": "192.168.1.10"},
            {"port": 2111},
            {"mode": 4},  #Operating Mode: 0=powerdown, 1=standby, 2=mapping, 3=ladnmark, 4=navigation
            {"perform_mapping": False},
            {"publish_odom": True},
            {"wait_command": 1},
            {"mask_command": 1}, #Mask: Landmarkmode 0 = reflectors, 1 = reflectors+scan; Nav mode 0 = pose+reflectors, 1 = pose+scan, 2 = pose+reflectors+scan
            {"scan_rate": 8}, #5~8hz
            {"frame_id": "odom"},  #from
            {"target_frame_id": "nav350"},  #to
            {"reflector_frame_id": "nav350"},  #display reflector from
            {"reflector_child_frame_id": "reflector"}, #to
            {"scan_inverted": False},
            {"odom_inverted": False},
        ],
    )


    return launch.LaunchDescription([
        Static_tf_map_to_odom_node,
        #Static_tf_nav350_to_base_link_node,
        NAV350_node,
    ])
