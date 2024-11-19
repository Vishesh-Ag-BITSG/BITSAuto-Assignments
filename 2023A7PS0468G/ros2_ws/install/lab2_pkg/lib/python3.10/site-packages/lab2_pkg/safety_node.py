#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        
        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        
        # Publisher
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Initialize variables
        self.current_velocity = 0.0
        self.laser_data = None
        
        self.declare_parameter('collision_threshold', 1.0)  # meters
        self.collision_threshold = self.get_parameter('collision_threshold').get_parameter_value().double_value

    def scan_callback(self, msg):
        self.laser_data = np.array(msg.ranges)

    def odom_callback(self, msg):
        self.current_velocity = msg.twist.twist.linear.x
        self.check_for_collisions()

    def check_for_collisions(self):
        if self.laser_data is None:
            return
        
        # Filter laser data
        laser_data_filtered = np.clip(self.laser_data, 0.01, np.inf)  # Avoid inf/nan values
        
        # Calculate Time to Collision (iTTC)
        iTTC = laser_data_filtered / (self.current_velocity + 1e-6)  # Avoid division by zero
        iTTC[np.isnan(iTTC)] = np.inf  # Handle NaNs
        iTTC[iTTC < 0] = np.inf  # Ignore negative values
        
        # Check for imminent collision
        if np.min(iTTC) < self.collision_threshold:
            self.brake()

    def brake(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0  # Set speed to 0 m/s
        self.drive_pub.publish(msg)
        self.get_logger().info('Braking! Collision imminent.')

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

