#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        
        # Publisher for the drive command
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        # Subscribers for odometry and laser scan
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.speed = 0.0
        self.last_scan_ranges = None
        self.brake_active = False
    def odom_callback(self, odom_msg):
        # Update current speed from odometry
        self.speed = odom_msg.twist.twist.linear.x
        
    def scan_callback(self, scan_msg):
        # Calculate iTTC
        if self.last_scan_ranges is not None:
            range_rate = self.calculate_range_rate(scan_msg.ranges)
            iTTCs = self.calculate_iTTC(scan_msg.ranges, range_rate)

            # Log iTTC values for debugging
            # self.get_logger().info(f'iTTCs: {iTTCs}')

            # Check if any iTTC is below a threshold (e.g., 1.0 seconds)
            if np.any(iTTCs < 1.0) and not self.brake_active:
                self.brake()
                self.brake_active = True  # Set brake flag to active
            elif np.all(iTTCs >= 1.0):
                self.brake_active = False  # Reset brake flag if safe

        # Store the current scan ranges for the next calculation
        self.last_scan_ranges = scan_msg.ranges
        
    def calculate_range_rate(self, ranges):
        """Calculates the range rate for each angle in the LaserScan."""
        range_rate = np.zeros(len(ranges))
        
        for i in range(len(ranges)):
            angle = i * np.pi / len(ranges)  # Calculate the angle for this index
            range_rate[i] = -self.speed * np.cos(angle)  # Speed projection on the angle
            
        return range_rate
    
    def calculate_iTTC(self, ranges, range_rate):
        """Calculates Instantaneous Time to Collision (iTTC) for each range measurement."""
        iTTCs = np.zeros(len(ranges))
        
        for i in range(len(ranges)):
            r = ranges[i]
            r_dot = range_rate[i]
            
            # Check if range or range rate is valid
            if r <= 0 or r_dot >= 0:  # Avoid division by zero or negative ranges
                iTTCs[i] = float('inf')
            else:
                # Prevent overflow by clamping r_dot to a minimum value
                r_dot = max(r_dot, 1e-6)  # Use a small positive number to avoid overflow
                
                iTTCs[i] = r / (-r_dot)  # iTTC = r / -r_dot
        
        return iTTCs
    
    def brake(self):
        """Publish a braking command."""
        brake_msg = AckermannDriveStamped()
        brake_msg.drive.speed = 0.0  # Set speed to 0 to stop
        self.drive_pub.publish(brake_msg)
        self.get_logger().info('Braking! Speed set to 0.')

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    safety_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
