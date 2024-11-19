#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class Talker(Node):
    def __init__(self):
        super().__init__('talker')

        # Declare parameters with default values
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('steering_angle', 0.5)

        # Create a publisher
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)

        # Set a timer to publish messages at regular intervals
        self.timer = self.create_timer(0.1, self.publish_message)

    def publish_message(self):
        # Retrieve the current values of the parameters
        speed = self.get_parameter('speed').get_parameter_value().double_value
        steering_angle = self.get_parameter('steering_angle').get_parameter_value().double_value

        # Create and publish the AckermannDriveStamped message
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = steering_angle
        self.publisher_.publish(msg)

        # Log the message being published
        self.get_logger().info(f'Publishing: speed {msg.drive.speed}, steering_angle {msg.drive.steering_angle}')

def main(args=None):
    rclpy.init(args=args)
    node = Talker()

    # Spin the node so the callback continues running
    rclpy.spin(node)

    # Clean up after shutting down the node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

