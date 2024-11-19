#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.
        # TODO: create ROS subscribers and publishers.

        self.ttc_thershold = 0.5

        self.drive_publisher  = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # self.get_getlogger().info("SafetyNode has ")

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x


    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        ranges = np.array(scan_msg.ranges)
        angle_increment = scan_msg.angle_increment

        min_ttc = float('inf')

        for i, distance in enumerate(ranges):
            if distance == float('inf') or distance <= 0.1:
                continue
        # TODO: publish command to brake
        
        angle = scan_msg.angle_min + i*angle_increment

        relative_speed = self.speed * np.cos(angle)

        if relative_speed > 0:
            ttc = distance / relative_speed
            if ttc < min_ttc:
                min_ttc = ttc

        if min_ttc < self.ttc_thershold:
            self.get_logger().warn('Emergency braking! TTC below threshold.')
            self.publish_brake_command()
        
        def publish_brake_command(self):

            brake_msg = AckermannDriveStamped()
            brake_msg.drive.speed = 0.0 # Stop the vehicle
            brake_msg.drive.steering_angle = 0.00 # Keep the steering angle neutral
            self.drive_publisher.publish(brake_msg)
            self.get_logger().info('Published brake command!')

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()