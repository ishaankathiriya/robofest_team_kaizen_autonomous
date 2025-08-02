# Step-by-Step Online SLAM Instructions

If the automated scripts are having issues, follow these manual steps to debug and run the system:

## Step 1: Set Environment Variables
```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models
```

## Step 2: Test Basic TurtleBot3 Simulation First
```bash
# In Terminal 1: Launch basic TurtleBot3 world
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py world:=/home/ishaan/turtlebot_sim/simple_unknown_world.world x_pose:=-2.0 y_pose:=-2.0
```

Wait for Gazebo to fully load and the robot to spawn. You should see the TurtleBot3 in Gazebo.

## Step 3: Launch SLAM Toolbox
```bash
# In Terminal 2: Start SLAM
ros2 launch slam_toolbox online_async_launch.py params_file:=/home/ishaan/turtlebot_sim/online_slam_config.yaml use_sim_time:=true
```

## Step 4: Launch Nav2 (without map_server)
```bash
# In Terminal 3: Start Navigation
ros2 launch nav2_bringup navigation_launch.py params_file:=/home/ishaan/turtlebot_sim/nav2_online_slam_params.yaml use_sim_time:=true
```

## Step 5: Launch RViz
```bash
# In Terminal 4: Start visualization
rviz2 -d /home/ishaan/turtlebot_sim/online_slam_nav.rviz
```

## Step 6: Test Manual Navigation
1. In RViz, you should see:
   - The robot model
   - Laser scan data (white points)
   - An empty map that fills as robot moves
   
2. Use the "2D Goal Pose" tool to set a navigation target
3. Watch the robot navigate while building the map

## Step 7: Launch Autonomous Goal Publisher (Optional)
```bash
# In Terminal 5: Start autonomous exploration
cd /home/ishaan/turtlebot_sim
python3 autonomous_goal_publisher.py
```

## Troubleshooting Common Issues

### Issue 1: Plugin naming error (nav2_navfn_planner::NavfnPlanner)
**Fix**: The config file should use `nav2_navfn_planner/NavfnPlanner` (with `/` not `::`).

### Issue 2: Gazebo crashes with physics errors
**Fix**: Use the simplified world file: `simple_unknown_world.world`

### Issue 3: Robot appears white in RViz
**Cause**: URDF/robot description issues
**Fix**: Check that robot_state_publisher is running properly

### Issue 4: No map appearing
**Check**: 
```bash
ros2 topic echo /map
ros2 topic echo /scan
```

### Issue 5: Navigation not working
**Check**: Ensure transforms are working
```bash
ros2 run tf2_tools view_frames
```

## Manual Teleoperation (for testing)
If you need to move the robot manually for testing:
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

## Key Topics to Monitor
```bash
# Check if robot is publishing sensor data
ros2 topic echo /scan

# Check if SLAM is working
ros2 topic echo /map

# Check robot position
ros2 topic echo /tf

# Check navigation commands
ros2 topic echo /cmd_vel
```

## Expected Behavior
1. **Gazebo**: Robot spawns in simple world with obstacles
2. **SLAM**: Map builds progressively as robot moves/scans
3. **Navigation**: Robot can navigate to goals in mapped areas
4. **RViz**: Real-time visualization of mapping and navigation

This step-by-step approach helps identify exactly where issues occur!