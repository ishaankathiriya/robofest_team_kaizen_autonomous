#!/bin/bash

echo "Testing Basic TurtleBot3 Setup..."

# Set environment
export TURTLEBOT3_MODEL=waffle

# Test 1: Check if packages are installed
echo "1. Checking package installation..."
packages=("turtlebot3_gazebo" "slam_toolbox" "nav2_bringup")
for pkg in "${packages[@]}"; do
    if ros2 pkg list | grep -q "$pkg"; then
        echo "  ✓ $pkg found"
    else
        echo "  ✗ $pkg NOT found - install with: sudo apt install ros-humble-$pkg"
        exit 1
    fi
done

# Test 2: Check config files exist
echo "2. Checking configuration files..."
files=("/home/ishaan/turtlebot_sim/online_slam_config.yaml" 
       "/home/ishaan/turtlebot_sim/nav2_online_slam_params.yaml"
       "/home/ishaan/turtlebot_sim/simple_unknown_world.world")
for file in "${files[@]}"; do
    if [ -f "$file" ]; then
        echo "  ✓ $file exists"
    else
        echo "  ✗ $file NOT found"
        exit 1
    fi
done

# Test 3: Check URDF file
echo "3. Checking URDF file..."
urdf_path=$(ros2 pkg prefix turtlebot3_description)/share/turtlebot3_description/urdf/turtlebot3_waffle.urdf
if [ -f "$urdf_path" ]; then
    echo "  ✓ TurtleBot3 URDF found at $urdf_path"
else
    echo "  ✗ TurtleBot3 URDF NOT found"
    echo "    Install with: sudo apt install ros-humble-turtlebot3-description"
    exit 1
fi

echo ""
echo "✓ All basic checks passed!"
echo ""
echo "Now you can try running the system:"
echo "  ./run_simple_slam.sh"
echo ""
echo "Or follow the step-by-step instructions in:"
echo "  STEP_BY_STEP_INSTRUCTIONS.md"