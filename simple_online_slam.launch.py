#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file', default='/home/ishaan/turtlebot_sim/simple_unknown_world.world')
    
    # Get package directories
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    # Configuration files
    slam_params_file = '/home/ishaan/turtlebot_sim/online_slam_config.yaml'
    nav2_params_file = '/home/ishaan/turtlebot_sim/nav2_online_slam_params.yaml'
    rviz_config_file = '/home/ishaan/turtlebot_sim/online_slam_nav.rviz'
    
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
        
        # Launch TurtleBot3 simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_gazebo'),
                    'launch',
                    'turtlebot3_world.launch.py'
                ])
            ]),
            launch_arguments={
                'world': world_file,
                'x_pose': '-2.0',
                'y_pose': '-2.0'
            }.items()
        ),
        
        # Launch SLAM Toolbox online async (delayed to ensure robot is spawned)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
                )
            ]
        ),
        
        # Launch Nav2 without map_server (delayed to ensure SLAM is ready)
        TimerAction(
            period=10.0,
            actions=[
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
                )
            ]
        ),
        
        # Launch RViz with custom config (delayed)
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('use_rviz', default='true'))
                )
            ]
        ),
        
    ])