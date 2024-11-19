#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class RelayNode(Node):

    def __init__(self):

        super().__init__('relay')

        self.get_logger().info('Relay node has started')

        self.cmd_drive_sub_ = self.create_subscription(AckermannDriveStamped, 'drive', self.subscribe_drive_callback, 10)

    def subscribe_drive_callback(self, msg: AckermannDriveStamped):

        car_relay = AckermannDriveStamped()

        car_relay.drive.speed = msg.drive.speed * 3
        car_relay.drive.steering_angle = msg.drive.steering_angle * 3

        self.cmd_drive_relay_pub_ = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)

        self.cmd_drive_relay_pub_.publish(car_relay)

        self.get_logger().info(f"Relay speed is {car_relay.drive.speed}. Relay steering angle is {car_relay.drive.steering_angle}")





def main(args=None):

    rclpy.init(args=args)

    node = RelayNode()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()