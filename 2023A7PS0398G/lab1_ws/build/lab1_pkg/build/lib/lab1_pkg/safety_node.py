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
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )
        self.current_speed = 0.0

    def lidar_callback(self, msg):
        # Compute iTTCs based on the LaserScan data
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = np.nan  # Handle inf values
        ranges = ranges[~np.isnan(ranges)]  # Remove NaNs

        # Example: Basic logic to determine if an obstacle is close
        if len(ranges) > 0:
            min_distance = np.min(ranges)
            self.get_logger().info(f'Minimum distance: {min_distance}')
            # Threshold for stopping
            if min_distance < 1.0:  # 1 meter threshold
                self.stop_car()

    def odom_callback(self, msg):
        self.current_speed = msg.twist.twist.linear.x

    def stop_car(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0  # Stop the car
        self.drive_publisher.publish(msg)
        self.get_logger().info('Car is stopping!')

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

