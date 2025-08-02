#!/bin/bash

echo "=================================================="
echo "TurtleBot3 Fixed Online SLAM Navigation System"
echo "=================================================="

# Set environment variables
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models

echo "Environment variables set:"
echo "TURTLEBOT3_MODEL: $TURTLEBOT3_MODEL"

# Test setup first
echo ""
echo "Running setup test..."
if ! ./test_basic_setup.sh; then
    echo "Setup test failed. Please check requirements."
    exit 1
fi

echo ""
echo "Launching Fixed Online SLAM Navigation..."
echo "Components will start in sequence with delays:"
echo "  0s: Gazebo"
echo "  3s: SLAM Toolbox"
echo "  8s: Nav2 Navigation"
echo "  12s: RViz"
echo ""
echo "Please wait for all components to load..."
echo "Press Ctrl+C to stop the system"
echo ""

# Launch the fixed system
ros2 launch /home/ishaan/turtlebot_sim/fixed_online_slam.launch.py

echo ""
echo "Online SLAM navigation system stopped."