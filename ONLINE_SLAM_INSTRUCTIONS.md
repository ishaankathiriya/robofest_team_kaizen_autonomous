# TurtleBot3 Online SLAM Navigation Instructions

## Overview
This system implements **true autonomous navigation** where TurtleBot3 simultaneously performs SLAM (mapping) and navigation in an unknown environment. The robot can navigate from one location to another without any pre-existing map.

## System Components
- **Gazebo**: Custom unknown environment with static obstacles
- **TurtleBot3 Waffle**: Differential drive robot with LiDAR
- **SLAM Toolbox**: Online asynchronous SLAM for real-time mapping
- **Nav2**: Navigation stack configured for online SLAM
- **Autonomous Goal Publisher**: Python node for autonomous exploration
- **RViz**: Visualization and manual goal setting

## Prerequisites

### 1. Install Required Packages
```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox ros-humble-nav2-bringup ros-humble-turtlebot3-gazebo
```

### 2. Set Environment Variables
```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models
```

## Quick Start

### Method 1: Using the Run Script (Recommended)
```bash
cd /home/ishaan/turtlebot_sim
./run_online_slam.sh
```

### Method 2: Manual Launch
```bash
# Set environment variables
export TURTLEBOT3_MODEL=waffle

# Launch the complete system
ros2 launch /home/ishaan/turtlebot_sim/online_slam_navigation.launch.py
```

## What Happens When You Launch

1. **Gazebo Opens**: Shows an unknown environment with various obstacles
2. **TurtleBot3 Spawns**: Robot appears at position (-6, -6) in the world
3. **SLAM Starts**: Robot begins building a map as it moves
4. **Nav2 Initializes**: Navigation stack starts without any pre-existing map
5. **RViz Opens**: Visualization showing:
   - Real-time map building
   - Robot position and laser scans
   - Navigation paths and costmaps

## Operating the System

### Option 1: Manual Goal Setting
1. **In RViz**: Use the "2D Goal Pose" tool (green arrow) to set navigation targets
2. **Click and Drag**: Click on the map where you want the robot to go, drag to set orientation
3. **Watch**: Robot plans a path and navigates autonomously while updating the map

### Option 2: Autonomous Exploration
```bash
# In a new terminal, run the autonomous goal publisher
cd /home/ishaan/turtlebot_sim
python3 autonomous_goal_publisher.py
```

This will make the robot:
- Automatically set exploration goals
- Navigate to frontiers (boundaries between known and unknown areas)
- Continuously explore the environment
- Build a complete map autonomously

## Key Features Demonstrated

### 1. Online SLAM
- Robot builds map in real-time as it moves
- No pre-existing map required
- Handles loop closures and drift correction
- Map updates continuously during navigation

### 2. Autonomous Navigation
- Plans paths in partially known environments
- Avoids obstacles using sensor data
- Replans when obstacles are detected
- Works with dynamic goal setting

### 3. True Unknown Environment Navigation
- Robot starts with no knowledge of environment
- Navigates successfully to goals in unexplored areas
- Builds spatial understanding incrementally
- Handles complex obstacle arrangements

## Understanding the Visualization

### RViz Display Elements

**Map (Gray/Black/White)**:
- **White**: Free space (safe to navigate)
- **Black**: Obstacles (walls, boxes, etc.)
- **Gray**: Unknown/unexplored areas

**Robot (Colored Model)**:
- Shows current robot position and orientation
- LiDAR scan data (white points) shows sensor readings

**Costmaps**:
- **Local Costmap**: Small area around robot for immediate navigation
- **Global Costmap**: Larger area showing global path planning space
- **Colors**: Blue=free, Yellow/Red=obstacles, Purple=inflation zones

**Path Planning**:
- **Green Line**: Current planned path to goal
- **Updates**: Path replanning happens automatically when obstacles change

### SLAM Visualization
- **Graph Nodes**: Purple markers showing SLAM pose graph
- **Real-time Updates**: Map expands as robot explores new areas
- **Loop Closures**: Automatic correction when robot recognizes previously visited areas

## Configuration Files

### SLAM Configuration
- **File**: `online_slam_config.yaml`
- **Purpose**: Configures SLAM Toolbox for online operation
- **Key Settings**: Mapping frequency, loop closure, optimization parameters

### Nav2 Configuration  
- **File**: `nav2_online_slam_params.yaml`
- **Purpose**: Configures Nav2 for operation without pre-existing maps
- **Key Changes**: Removed static map layer, optimized for online SLAM

### World File
- **File**: `unknown_environment.world`
- **Purpose**: Gazebo world with interesting obstacle configuration
- **Features**: Walls, boxes, cylinders, narrow passages, L-shaped obstacles

## Troubleshooting

### Common Issues

1. **Robot doesn't move**: 
   - Check that Nav2 is fully initialized (wait 10-15 seconds after launch)
   - Verify goal is set in free space (white areas in RViz)

2. **No map appearing**:
   - Verify SLAM Toolbox is running: `ros2 topic echo /map`
   - Check LiDAR data: `ros2 topic echo /scan`

3. **Navigation fails**:
   - Ensure goal is reachable in currently mapped area
   - Check costmap configuration if robot seems overly conservative

4. **SLAM quality issues**:
   - Move robot slower for better scan matching
   - Ensure environment has sufficient features for localization

### Useful Commands

```bash
# Check system status
ros2 node list
ros2 topic list

# Monitor key topics
ros2 topic echo /map
ros2 topic echo /scan
ros2 topic echo /cmd_vel

# Check transforms
ros2 run tf2_tools view_frames.py

# Manual teleoperation (if needed)
ros2 run turtlebot3_teleop teleop_keyboard
```

## Advanced Usage

### Custom Goal Publisher
Modify `autonomous_goal_publisher.py` to:
- Change exploration strategy
- Adjust goal selection criteria
- Add custom behaviors

### Parameter Tuning
Edit configuration files to:
- Improve SLAM quality
- Optimize navigation performance
- Adjust robot behavior

### World Modification
Edit `unknown_environment.world` to:
- Add more obstacles
- Change environment complexity
- Test different scenarios

## Expected Results

After running the system, you should observe:
1. **Incremental Mapping**: Map builds progressively as robot explores
2. **Successful Navigation**: Robot reaches goals in previously unknown areas  
3. **Obstacle Avoidance**: Robot navigates around obstacles intelligently
4. **Loop Closure**: Map corrections when robot revisits areas
5. **Autonomous Exploration**: Complete environment mapping (with goal publisher)

## Technical Notes

### Coordinate Frames
- **map**: Global reference frame created by SLAM
- **odom**: Odometry frame (continuous but drifts)
- **base_link**: Robot's coordinate frame
- **laser**: LiDAR sensor frame

### Key Topics
- `/map`: Occupancy grid map from SLAM
- `/scan`: LiDAR sensor data
- `/goal_pose`: Navigation goals
- `/cmd_vel`: Robot velocity commands
- `/plan`: Current navigation path

This system demonstrates the state-of-the-art in autonomous robotics, combining real-time SLAM with intelligent navigation for true unknown environment exploration!