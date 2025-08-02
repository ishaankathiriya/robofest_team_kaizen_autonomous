# TurtleBot3 Online SLAM Troubleshooting Guide

## Issues and Solutions

### Issue 1: Nav2 Plugin Naming Error
**Error**: `nav2_navfn_planner::NavfnPlanner does not exist`
**Solution**: Fixed in `nav2_online_slam_params.yaml` - changed to `nav2_navfn_planner/NavfnPlanner`

### Issue 2: Gazebo Crashing with Physics Errors
**Error**: `SetParam(gravity) std::any_cast error`
**Solutions**: 
- Use simplified world file: `simple_unknown_world.world`
- Fixed gravity parameters in world file
- Use proper Gazebo launch approach with gzserver/gzclient

### Issue 3: URDF File Path Issues
**Error**: Robot spawning failures or file not found
**Solution**: Use TurtleBot3's standard spawn_turtlebot3.launch.py instead of manual spawning

### Issue 4: White Robot Model in RViz
**Cause**: Robot description not properly loaded
**Solution**: Ensure robot_state_publisher is running with correct URDF

## Current Working Solutions

### Option 1: Fixed Launch System (Recommended)
```bash
cd /home/ishaan/turtlebot_sim
./run_fixed_slam.sh
```

### Option 2: Step-by-Step Manual Launch
Follow `STEP_BY_STEP_INSTRUCTIONS.md` for manual debugging

### Option 3: Simple Launch System
```bash
cd /home/ishaan/turtlebot_sim
./run_simple_slam.sh
```

## Testing and Validation

### 1. Test Basic Setup
```bash
./test_basic_setup.sh
```

### 2. Test Individual Components

**Test Gazebo Only:**
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py world:=/home/ishaan/turtlebot_sim/simple_unknown_world.world
```

**Test SLAM Only (after Gazebo is running):**
```bash
ros2 launch slam_toolbox online_async_launch.py params_file:=/home/ishaan/turtlebot_sim/online_slam_config.yaml use_sim_time:=true
```

**Test Nav2 Only (after SLAM is running):**
```bash
ros2 launch nav2_bringup navigation_launch.py params_file:=/home/ishaan/turtlebot_sim/nav2_online_slam_params.yaml use_sim_time:=true
```

### 3. Monitor System Status

**Check topics:**
```bash
ros2 topic list
ros2 topic echo /scan
ros2 topic echo /map
ros2 topic echo /tf
```

**Check nodes:**
```bash
ros2 node list
```

**Check services:**
```bash
ros2 service list
```

## Key Fixes Applied

1. **Plugin Names**: Changed `::` to `/` in Nav2 configuration
2. **World File**: Simplified with proper physics settings
3. **Launch Sequence**: Added proper timing delays between components
4. **Gazebo Approach**: Use standard gzserver/gzclient launch
5. **Robot Spawning**: Use TurtleBot3's standard spawn method

## System Architecture

```
Components Start Sequence:
0s:  Gazebo (gzserver + gzclient)
     └── Robot spawns automatically
3s:  SLAM Toolbox
     └── Starts building map from laser data
8s:  Nav2 Navigation Stack
     └── Initializes without static map
12s: RViz
     └── Visualization ready for interaction
```

## Expected Results

When working correctly, you should see:

1. **Gazebo**: TurtleBot3 spawned in simple world with obstacles
2. **RViz**: 
   - Robot model properly colored
   - Laser scan data (white points)
   - Map building progressively (gray/black/white)
   - Navigation tools available

3. **Navigation**: 
   - Set goals with "2D Goal Pose" tool
   - Robot navigates while building map
   - Real-time obstacle avoidance

4. **SLAM**: 
   - Map expands as robot explores
   - Loop closure when revisiting areas
   - Consistent localization

## Performance Tips

1. **Wait for Full Initialization**: Allow 15+ seconds for all components
2. **Set Reasonable Goals**: Only set goals in explored (white) areas
3. **Monitor Resource Usage**: SLAM can be CPU intensive
4. **Use Simple World**: Start with simple_unknown_world.world

## Support Commands

**Kill all ROS processes:**
```bash
pkill -f ros2
pkill -f gazebo
```

**Reset environment:**
```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
```

**Check ROS installation:**
```bash
ros2 doctor
```

This guide should help resolve the most common issues encountered during setup and operation.