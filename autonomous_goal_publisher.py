#!/usr/bin/env python3

"""
Autonomous Goal Publisher Node for Online SLAM Navigation

This node publishes navigation goals to enable autonomous exploration
and navigation in unknown environments while SLAM is running.

Author: Generated for TurtleBot3 Online SLAM Demo
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
import random
import math
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class AutonomousGoalPublisher(Node):
    def __init__(self):
        super().__init__('autonomous_goal_publisher')
        
        # Initialize parameters
        self.declare_parameter('goal_timeout', 30.0)  # seconds
        self.declare_parameter('exploration_radius', 5.0)  # meters
        self.declare_parameter('min_goal_distance', 2.0)  # meters
        self.declare_parameter('max_goal_distance', 8.0)  # meters
        
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.exploration_radius = self.get_parameter('exploration_radius').value
        self.min_goal_distance = self.get_parameter('min_goal_distance').value
        self.max_goal_distance = self.get_parameter('max_goal_distance').value
        
        # Publishers
        self.goal_publisher = self.create_publisher(
            PoseStamped, 
            '/goal_pose', 
            10
        )
        
        # Subscribers
        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # State variables
        self.current_map = None
        self.robot_pose = None
        self.last_goal_time = self.get_clock().now()
        self.goal_sent = False
        
        # Timer for goal publishing
        self.goal_timer = self.create_timer(5.0, self.goal_timer_callback)
        
        # Timer for robot pose updates
        self.pose_timer = self.create_timer(1.0, self.update_robot_pose)
        
        self.get_logger().info('Autonomous Goal Publisher initialized')
        self.get_logger().info(f'Goal timeout: {self.goal_timeout}s')
        self.get_logger().info(f'Exploration radius: {self.exploration_radius}m')

    def map_callback(self, msg):
        """Process incoming map data"""
        self.current_map = msg
        self.get_logger().debug('Map updated')

    def update_robot_pose(self):
        """Update current robot pose from TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            self.robot_pose = transform.transform
            
        except Exception as e:
            self.get_logger().debug(f'Could not get robot pose: {e}')
            self.robot_pose = None

    def goal_timer_callback(self):
        """Main goal publishing logic"""
        if not self.should_publish_goal():
            return
            
        goal_pose = self.generate_exploration_goal()
        if goal_pose:
            self.publish_goal(goal_pose)
        else:
            self.get_logger().info('No valid goal found, continuing exploration...')

    def should_publish_goal(self):
        """Determine if a new goal should be published"""
        if self.current_map is None:
            self.get_logger().debug('No map available yet')
            return False
            
        if self.robot_pose is None:
            self.get_logger().debug('No robot pose available yet')
            return False
            
        current_time = self.get_clock().now()
        time_since_last_goal = (current_time - self.last_goal_time).nanoseconds / 1e9
        
        if not self.goal_sent or time_since_last_goal > self.goal_timeout:
            return True
            
        return False

    def generate_exploration_goal(self):
        """Generate a valid exploration goal"""
        if not self.current_map or not self.robot_pose:
            return None
            
        # Convert robot pose to map coordinates
        robot_x = self.robot_pose.translation.x
        robot_y = self.robot_pose.translation.y
        
        # Try to find a valid goal up to 50 attempts
        for _ in range(50):
            # Generate random goal within exploration radius
            angle = random.uniform(0, 2 * math.pi)
            distance = random.uniform(self.min_goal_distance, self.max_goal_distance)
            
            goal_x = robot_x + distance * math.cos(angle)
            goal_y = robot_y + distance * math.sin(angle)
            
            if self.is_valid_goal(goal_x, goal_y):
                return self.create_pose_stamped(goal_x, goal_y, angle)
                
        # If no random goal found, try frontier-based exploration
        return self.find_frontier_goal()

    def is_valid_goal(self, x, y):
        """Check if a goal position is valid (in free space)"""
        if not self.current_map:
            return False
            
        # Convert world coordinates to map indices
        map_x = int((x - self.current_map.info.origin.position.x) / self.current_map.info.resolution)
        map_y = int((y - self.current_map.info.origin.position.y) / self.current_map.info.resolution)
        
        # Check bounds
        if (map_x < 0 or map_x >= self.current_map.info.width or
            map_y < 0 or map_y >= self.current_map.info.height):
            return False
            
        # Check if position is free (value 0 in occupancy grid)
        index = map_y * self.current_map.info.width + map_x
        if index >= len(self.current_map.data):
            return False
            
        cell_value = self.current_map.data[index]
        
        # Free space is typically 0, occupied is 100, unknown is -1
        return cell_value >= 0 and cell_value < 50

    def find_frontier_goal(self):
        """Find goals at the frontier between known and unknown space"""
        if not self.current_map:
            return None
            
        frontiers = []
        width = self.current_map.info.width
        height = self.current_map.info.height
        
        # Search for frontier cells (free cells adjacent to unknown cells)
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                index = y * width + x
                
                if self.current_map.data[index] == 0:  # Free cell
                    # Check if adjacent to unknown space
                    adjacent_indices = [
                        (y-1) * width + x,  # North
                        (y+1) * width + x,  # South
                        y * width + (x-1),  # West
                        y * width + (x+1)   # East
                    ]
                    
                    for adj_index in adjacent_indices:
                        if (adj_index >= 0 and adj_index < len(self.current_map.data) and
                            self.current_map.data[adj_index] == -1):  # Unknown
                            
                            # Convert to world coordinates
                            world_x = x * self.current_map.info.resolution + self.current_map.info.origin.position.x
                            world_y = y * self.current_map.info.resolution + self.current_map.info.origin.position.y
                            
                            # Check if within exploration radius
                            if self.robot_pose:
                                dx = world_x - self.robot_pose.translation.x
                                dy = world_y - self.robot_pose.translation.y
                                distance = math.sqrt(dx*dx + dy*dy)
                                
                                if (self.min_goal_distance <= distance <= self.max_goal_distance):
                                    frontiers.append((world_x, world_y))
                            break
        
        if frontiers:
            # Choose a random frontier
            frontier = random.choice(frontiers)
            angle = random.uniform(0, 2 * math.pi)
            return self.create_pose_stamped(frontier[0], frontier[1], angle)
            
        return None

    def create_pose_stamped(self, x, y, yaw):
        """Create a PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose

    def publish_goal(self, goal_pose):
        """Publish the goal pose"""
        self.goal_publisher.publish(goal_pose)
        self.last_goal_time = self.get_clock().now()
        self.goal_sent = True
        
        self.get_logger().info(
            f'Published goal: x={goal_pose.pose.position.x:.2f}, '
            f'y={goal_pose.pose.position.y:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    
    node = AutonomousGoalPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()