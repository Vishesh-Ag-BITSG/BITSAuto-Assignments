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
            self.drive_callback,
            10
        )
        self.subscription
        
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)

    def drive_callback(self, msg):
        new_speed = msg.drive.speed * 3
        new_steering_angle = msg.drive.steering_angle * 3

        new_msg = AckermannDriveStamped()
        new_msg.drive.speed = new_speed
        new_msg.drive.steering_angle = new_steering_angle


        self.publisher_.publish(new_msg)
        self.get_logger().info(f'Relaying: speed={new_speed}, steering_angle={new_steering_angle}')

def main(args=None):
    rclpy.init(args=args)
    relay = Relay()

    rclpy.spin(relay)

    relay.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
