#!/bin/bash

# Simple TurtleBot3 Online SLAM Launcher
echo "============================================"
echo "TurtleBot3 Simple Online SLAM Navigation"
echo "============================================"

# Set environment variables
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models

echo "Environment set: TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"

# Check if all required packages are available
echo "Checking required packages..."
if ! ros2 pkg list | grep -q "turtlebot3_gazebo"; then
    echo "ERROR: turtlebot3_gazebo package not found"
    echo "Install with: sudo apt install ros-humble-turtlebot3-gazebo"
    exit 1
fi

if ! ros2 pkg list | grep -q "slam_toolbox"; then
    echo "ERROR: slam_toolbox package not found"
    echo "Install with: sudo apt install ros-humble-slam-toolbox"
    exit 1
fi

if ! ros2 pkg list | grep -q "nav2_bringup"; then
    echo "ERROR: nav2_bringup package not found"
    echo "Install with: sudo apt install ros-humble-nav2-bringup"
    exit 1
fi

echo "All packages found!"
echo ""
echo "Launching simple online SLAM navigation..."
echo "This will start components in sequence with delays"
echo ""

# Start the simplified launch file
ros2 launch /home/ishaan/turtlebot_sim/simple_online_slam.launch.py