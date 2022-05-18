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
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    #for declaring the common arguments
    pkg_share = launch_ros.substitutions.FindPackageShare(package='lgv_bringup').find('lgv_bringup')
    default_model_path = os.path.join(pkg_share, 'src/description/diffbot.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    world_path=os.path.join(pkg_share, 'world/my_world.sdf')

    #for launching python launch file 
    nav2_launch_file_dir = os.path.join(get_package_share_directory('lgv_navigation'), 'launch')
    slam_launch_file_dir = os.path.join(get_package_share_directory('lgv_slam'), 'launch')
    nav350_launch_file_dir = os.path.join(get_package_share_directory('lgv_nav350'), 'launch')
    diffbot_launch_file_dir = os.path.join(get_package_share_directory('diffbot_bringup'), 'launch')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),('/tf_static', 'tf_static')]

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}],
        #remappings=remappings,
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    spawn_entity = launch_ros.actions.Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'lgv', '-topic', 'robot_description'],
        output='screen'
    )
    robot_localization_node = launch_ros.actions.Node(
          package='robot_localization',
          executable='ekf_node',
          name='ekf_filter_node',
          output='screen',
          parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )



    return launch.LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource([nav350_launch_file_dir, '/sicknav350.launch.py'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([diffbot_launch_file_dir, '/diffbot.launch.py'])),
        #launch.actions.DeclareLaunchArgument(name='gui', default_value='true', description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='false', description='Flag to enable use_sim_time'),
        #launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        #joint_state_publisher_node,
        robot_state_publisher_node,
        #spawn_entity,
        #robot_localization_node,
        rviz_node,
        IncludeLaunchDescription(PythonLaunchDescriptionSource([nav2_launch_file_dir, '/nav2.launch.py'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([slam_launch_file_dir, '/slam.launch.py'])),
    ])
