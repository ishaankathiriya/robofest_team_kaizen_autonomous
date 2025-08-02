#!/bin/bash

# TurtleBot3 Online SLAM Navigation Launcher Script
# This script sets up and launches the complete online SLAM navigation system

echo "=================================================="
echo "TurtleBot3 Online SLAM Navigation System"
echo "=================================================="

# Set environment variables
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models

echo "Environment variables set:"
echo "TURTLEBOT3_MODEL: $TURTLEBOT3_MODEL"
echo "GAZEBO_MODEL_PATH: $GAZEBO_MODEL_PATH"

echo ""
echo "Launching TurtleBot3 Online SLAM Navigation..."
echo "This will start:"
echo "  1. Gazebo with unknown environment"
echo "  2. TurtleBot3 Waffle robot"
echo "  3. SLAM Toolbox (online mapping)"
echo "  4. Nav2 navigation stack"
echo "  5. RViz visualization"
echo ""

# Check if required packages are installed
echo "Checking required packages..."

required_packages=("slam-toolbox" "nav2-bringup" "turtlebot3-gazebo")
missing_packages=()

for package in "${required_packages[@]}"; do
    if ! ros2 pkg list | grep -q "$package"; then
        missing_packages+=("$package")
    fi
done

if [ ${#missing_packages[@]} -ne 0 ]; then
    echo "ERROR: Missing required packages:"
    for package in "${missing_packages[@]}"; do
        echo "  - ros-humble-$package"
    done
    echo ""
    echo "Please install missing packages with:"
    echo "sudo apt update && sudo apt install $(printf 'ros-humble-%s ' "${missing_packages[@]}")"
    exit 1
fi

echo "All required packages found!"
echo ""

# Launch the system
echo "Starting the online SLAM navigation system..."
echo "Press Ctrl+C to stop the system"
echo ""

# Execute the launch file
ros2 launch /home/ishaan/turtlebot_sim/online_slam_navigation.launch.py

echo ""
echo "Online SLAM navigation system stopped."