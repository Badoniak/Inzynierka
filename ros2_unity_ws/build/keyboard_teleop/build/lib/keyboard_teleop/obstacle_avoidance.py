#!/usr/bin/env python3
"""
Obstacle Avoidance Node for LDRobot LiDAR
Subscribes to laser scan data and publishes velocity commands to navigate straight while avoiding obstacles.
Publishes to: /cmd_vel_nav

Usage:
    python3 obstacle_avoidance.py
    
    # Or with custom parameters:
    ros2 run ldlidar_node obstacle_avoidance.py --ros-args -p linear_speed:=0.2 -p safe_distance:=0.8
    
    # Or directly:
    python3 obstacle_avoidance.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math
import sys


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # Declare and get parameters with descriptions
        self.declare_parameter('linear_speed', 1.3)  # m/s - forward speed
        self.declare_parameter('angular_speed', 1.5)  # rad/s - turning speed
        self.declare_parameter('safe_distance', 0.6)  # meters - distance to keep from obstacles
        self.declare_parameter('danger_distance', 0.4)  # meters - critical distance to stop
        self.declare_parameter('scan_topic', '/LaserScan')
        self.declare_parameter('cmd_vel_topic', '/diff_drive_controller/cmd_vel')
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.danger_distance = self.get_parameter('danger_distance').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        
        # Create subscriber to laser scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )
        
        # Create publisher for velocity commands
        self.vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        
        # Timer for publishing commands at fixed rate
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # State variables
        self.latest_scan = None
        self.obstacle_detected = False
        self.turn_direction = 0  # -1: left, 0: straight, 1: right
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Obstacle Avoidance Node Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Subscribing to: {self.scan_topic}')
        self.get_logger().info(f'Publishing to: {self.cmd_vel_topic}')
        self.get_logger().info(f'Linear speed: {self.linear_speed} m/s')
        self.get_logger().info(f'Angular speed: {self.angular_speed} rad/s')
        self.get_logger().info(f'Safe distance: {self.safe_distance} m')
        self.get_logger().info(f'Danger distance: {self.danger_distance} m')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Robot will go straight and avoid obstacles')
        self.get_logger().info('Press Ctrl+C to stop')
        self.get_logger().info('=' * 60)
    
    def scan_callback(self, msg):
        """Process incoming laser scan data"""
        self.latest_scan = msg
        
        # Log first scan received
        if not hasattr(self, '_first_scan_logged'):
            self._first_scan_logged = True
            valid_points = sum(1 for r in msg.ranges if r > 0.01 and np.isfinite(r))
            self.get_logger().info(f'✓ First scan received! Total points: {len(msg.ranges)}, Valid points: {valid_points}')
            self.get_logger().info(f'  Range: {msg.range_min:.2f}m to {msg.range_max:.2f}m')
            self.get_logger().info(f'  Angle: {math.degrees(msg.angle_min):.1f}° to {math.degrees(msg.angle_max):.1f}°')
        
    def analyze_scan(self):
        """Analyze laser scan to detect obstacles and determine best direction"""
        if self.latest_scan is None:
            return False, 0
        
        ranges = np.array(self.latest_scan.ranges)
        angles = np.linspace(
            self.latest_scan.angle_min,
            self.latest_scan.angle_max,
            len(ranges)
        )
        
        # Filter out invalid readings (inf, nan, 0)
        valid_mask = np.isfinite(ranges) & (ranges > 0.01)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        if len(valid_ranges) == 0:
            return False, 0
        
        # Define sectors for obstacle detection
        # LiDAR reports angles from 0 to 2π (0° to 360°)
        # Front sector: 330° to 360° and 0° to 30° (11π/6 to 2π and 0 to π/6)
        # Left sector: 30° to 120° (π/6 to 2π/3)
        # Right sector: 240° to 330° (4π/3 to 11π/6)
        
        # Front: angles close to 0 or close to 2π
        front_mask = (valid_angles <= math.pi/6) | (valid_angles >= 11*math.pi/6)
        # Left: 30° to 120°
        left_mask = (valid_angles > math.pi/6) & (valid_angles <= 2*math.pi/3)
        # Right: 240° to 330°
        right_mask = (valid_angles >= 4*math.pi/3) & (valid_angles < 11*math.pi/6)
        
        # Find minimum distance in each sector
        front_min = np.min(valid_ranges[front_mask]) if np.any(front_mask) else float('inf')
        left_min = np.min(valid_ranges[left_mask]) if np.any(left_mask) else float('inf')
        right_min = np.min(valid_ranges[right_mask]) if np.any(right_mask) else float('inf')
        
        # Debug: log sector info occasionally
        if not hasattr(self, '_analyze_counter'):
            self._analyze_counter = 0
        self._analyze_counter += 1
        if self._analyze_counter % 30 == 1:
            front_count = np.sum(front_mask)
            left_count = np.sum(left_mask)
            right_count = np.sum(right_mask)
            self.get_logger().info(f'📊 Distances - Front: {front_min:.2f}m ({front_count} pts), Left: {left_min:.2f}m ({left_count} pts), Right: {right_min:.2f}m ({right_count} pts)')
            self.get_logger().info(f'   Angle range: {math.degrees(valid_angles.min()):.1f}° to {math.degrees(valid_angles.max()):.1f}°')
        
        # Check for obstacles in front
        obstacle_in_front = front_min < self.safe_distance
        danger_in_front = front_min < self.danger_distance
        
        # Determine turn direction based on which side has more space
        turn_direction = 0
        if obstacle_in_front:
            if left_min > right_min:
                turn_direction = -1  # Turn left
                self.get_logger().info(f'Obstacle ahead! Turning LEFT (front: {front_min:.2f}m, left: {left_min:.2f}m, right: {right_min:.2f}m)')
            else:
                turn_direction = 1  # Turn right
                self.get_logger().info(f'Obstacle ahead! Turning RIGHT (front: {front_min:.2f}m, left: {left_min:.2f}m, right: {right_min:.2f}m)')
        
        return danger_in_front or obstacle_in_front, turn_direction
    
    def control_loop(self):
        """Main control loop - publishes velocity commands"""
        cmd = Twist()
        
        if self.latest_scan is None:
            # No scan data yet, stop
            if not hasattr(self, '_waiting_logged'):
                self._waiting_logged = True
                self._waiting_counter = 0
                self.get_logger().warn('⏳ Waiting for scan data...')
                self.get_logger().warn(f'   Expecting data on topic: {self.scan_topic}')
                self.get_logger().warn('   Please check:')
                self.get_logger().warn('   1. Is the LiDAR node running? (ros2 node list)')
                self.get_logger().warn('   2. Is scan data being published? (ros2 topic list)')
                self.get_logger().warn(f'   3. Check scan topic: ros2 topic echo {self.scan_topic}')
            
            self._waiting_counter = getattr(self, '_waiting_counter', 0) + 1
            if self._waiting_counter % 50 == 0:  # Every 5 seconds
                self.get_logger().warn(f'⏳ Still waiting for scan data... ({self._waiting_counter//10}s)')
            
            self.vel_pub.publish(cmd)
            return
        
        # Analyze the scan for obstacles
        obstacle_detected, turn_direction = self.analyze_scan()
        
        # Log what's happening (throttled to every 20 iterations ~ 2 seconds)
        if not hasattr(self, '_control_counter'):
            self._control_counter = 0
        self._control_counter += 1
        
        if not obstacle_detected:
            # No obstacles, go straight
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            if self._control_counter % 20 == 1:
                self.get_logger().info(f'✓ Path clear - Moving forward at {self.linear_speed} m/s')
        else:
            # Obstacle detected, turn away
            if turn_direction == 0:
                # Very close obstacle, stop
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                if self._control_counter % 20 == 1:
                    self.get_logger().warn('⚠ DANGER! Stopping.')
            else:
                # Turn in the appropriate direction
                cmd.linear.x = self.linear_speed * 0.3  # Slow down while turning
                cmd.angular.z = turn_direction * self.angular_speed
                direction_str = "LEFT" if turn_direction < 0 else "RIGHT"
                if self._control_counter % 20 == 1:
                    self.get_logger().info(f'↻ Turning {direction_str} - vel: {cmd.linear.x:.2f} m/s, ang: {cmd.angular.z:.2f} rad/s')
        
        # Publish the command
        self.vel_pub.publish(cmd)
        
        # Log published command occasionally for debugging
        if self._control_counter % 50 == 1:
            self.get_logger().info(f'📤 Publishing: linear.x={cmd.linear.x:.3f}, angular.z={cmd.angular.z:.3f}')


def main(args=None):
    rclpy.init(args=args)
    
    node = ObstacleAvoidance()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        # Stop the robot before shutting down
        stop_cmd = Twist()
        node.vel_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
