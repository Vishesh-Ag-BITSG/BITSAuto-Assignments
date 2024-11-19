#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class Relay(Node):

    def __init__(self):
        super().__init__("relay_node")
        self.get_logger().info("Relay node running")

        self.create_subscription(AckermannDriveStamped,"/drive",self.driveSubriberCallback,10)
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive_relay', 10)

    def driveSubriberCallback(self,msg:AckermannDriveStamped):
        speed = msg.drive.speed
        steering_angle = msg.drive.steering_angle

        self.get_logger().info(f"v:{speed} d:{steering_angle}")
        
        self.publishDriveRelay(speed,steering_angle)

    def publishDriveRelay(self,v,d):

        msg = AckermannDriveStamped()
        msg.drive.speed = v*3
        msg.drive.steering_angle = d*3
        self.publisher.publish(msg)

def main(args=None):

    rclpy.init(args=args)
    node = Relay()
    rclpy.spin(node)
