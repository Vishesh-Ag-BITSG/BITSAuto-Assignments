#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class TalkerNode(Node):

    def __init__(self):
        super().__init__('talker')

        self.declare_parameter('v', 0.0)
        self.declare_parameter('d', 0.0)

        self.get_logger().info("Talker node has started")

        self.cmd_drive_pub_ = self.create_publisher(AckermannDriveStamped, "drive", 10)

        self.timer_ = self.create_timer(1, self.publish_drive)

    def publish_drive(self):
        
        car = AckermannDriveStamped()

        car.drive.speed = self.get_parameter('v').value
        car.drive.steering_angle = self.get_parameter('d').value

        self.get_logger().info(f"Speed is {car.drive.speed}. Steering angle is {car.drive.steering_angle}")

        self.cmd_drive_pub_.publish(car)

def main(args=None):

    rclpy.init(args=args)

    node = TalkerNode()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()