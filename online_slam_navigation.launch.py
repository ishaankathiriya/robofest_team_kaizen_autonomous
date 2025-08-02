#!/usr/bin/env python3

# Copyright 2024 Open Source Robotics Foundation, Inc.
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

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file', default='/home/ishaan/turtlebot_sim/unknown_environment.world')
    robot_name = LaunchConfiguration('robot_name', default='turtlebot3_waffle')
    x_pose = LaunchConfiguration('x_pose', default='-6.0')
    y_pose = LaunchConfiguration('y_pose', default='-6.0')
    z_pose = LaunchConfiguration('z_pose', default='0.01')
    roll = LaunchConfiguration('roll', default='0.00')
    pitch = LaunchConfiguration('pitch', default='0.00')
    yaw = LaunchConfiguration('yaw', default='0.00')
    
    # Get package directories
    turtlebot3_description_dir = get_package_share_directory('turtlebot3_description')
    
    # SLAM Toolbox config file
    slam_params_file = '/home/ishaan/turtlebot_sim/online_slam_config.yaml'
    
    # Nav2 params file (without map_server)
    nav2_params_file = '/home/ishaan/turtlebot_sim/nav2_online_slam_params.yaml'
    
    # RViz config file
    rviz_config_file = '/home/ishaan/turtlebot_sim/online_slam_nav.rviz'
    
    # Robot description - let's use the robot description topic instead of file
    urdf_file = os.path.join(turtlebot3_description_dir, 'urdf', 'turtlebot3_waffle.urdf')
    
    # Read URDF content
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Set environment variables for Gazebo
    env = os.environ.copy()
    gazebo_model_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
    if 'GAZEBO_MODEL_PATH' in env:
        env['GAZEBO_MODEL_PATH'] += ':' + gazebo_model_path
    else:
        env['GAZEBO_MODEL_PATH'] = gazebo_model_path
    
    return LaunchDescription([
        
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        DeclareLaunchArgument(
            'world_file',
            default_value=world_file,
            description='Full path to world file to load'),
            
        DeclareLaunchArgument(
            'robot_name',
            default_value=robot_name,
            description='Name of the robot'),
            
        DeclareLaunchArgument(
            'x_pose',
            default_value=x_pose,
            description='Initial x position of robot'),
            
        DeclareLaunchArgument(
            'y_pose',
            default_value=y_pose,
            description='Initial y position of robot'),
            
        DeclareLaunchArgument(
            'z_pose',
            default_value=z_pose,
            description='Initial z position of robot'),
            
        # Launch Gazebo with our custom world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
            launch_arguments={
                'world': world_file,
                'verbose': 'true'
            }.items()
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }]
        ),
        
        # Spawn TurtleBot3 in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_turtlebot3',
            arguments=[
                '-entity', robot_name,
                '-topic', '/robot_description',
                '-x', x_pose,
                '-y', y_pose,
                '-z', z_pose,
                '-R', roll,
                '-P', pitch,
                '-Y', yaw
            ],
            output='screen'
        ),
        
        # Launch SLAM Toolbox online async
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        ),
        
        # Launch Nav2 without map_server (for online SLAM)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_file,
                'autostart': 'true'
            }.items()
        ),
        
        # Launch RViz with custom config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz', default='true'))
        ),
        
    ])