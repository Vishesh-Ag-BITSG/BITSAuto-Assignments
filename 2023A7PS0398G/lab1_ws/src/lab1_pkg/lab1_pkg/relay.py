#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class Relay(Node):
    def __init__(self):
        super().__init__('relay')
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'drive',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: speed {msg.drive.speed}, steering_angle {msg.drive.steering_angle}')
        modified_speed = msg.drive.speed * 3
        modified_steering_angle = msg.drive.steering_angle * 3
        new_msg = AckermannDriveStamped()
        new_msg.drive.speed = modified_speed
        new_msg.drive.steering_angle = modified_steering_angle
        self.publisher_.publish(new_msg)
        self.get_logger().info(f'Publishing: modified speed {new_msg.drive.speed}, modified steering_angle {new_msg.drive.steering_angle}')

def main(args=None):
    rclpy.init(args=args)
    node = Relay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

