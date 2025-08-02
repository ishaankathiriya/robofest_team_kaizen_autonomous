#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='/home/ishaan/turtlebot_sim/simple_unknown_world.world')
    
    # Package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    # Configuration files
    slam_params_file = '/home/ishaan/turtlebot_sim/online_slam_config.yaml'
    nav2_params_file = '/home/ishaan/turtlebot_sim/nav2_online_slam_params.yaml'
    rviz_config_file = '/home/ishaan/turtlebot_sim/online_slam_nav.rviz'
    
    return LaunchDescription([
        
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        DeclareLaunchArgument(
            'world',
            default_value=world,
            description='Full path to world file to load'
        ),
        
        # Launch Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items()
        ),
        
        # Launch Gazebo client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            )
        ),
        
        # Spawn TurtleBot3
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': '-2.0',
                'y_pose': '-2.0'
            }.items()
        ),
        
        # Launch SLAM Toolbox (delayed to ensure robot is spawned)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[slam_params_file, {'use_sim_time': use_sim_time}]
                )
            ]
        ),
        
        # Launch Nav2 (delayed to ensure SLAM is ready)
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'params_file': nav2_params_file,
                        'autostart': 'true'
                    }.items()
                )
            ]
        ),
        
        # Launch RViz (delayed to ensure Nav2 is ready)
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen'
                )
            ]
        ),
        
    ])