#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class Talker(Node):

    def __init__(self):
        super().__init__("talker_node")
        self.get_logger().info("Talker node running")

        self.declare_parameter('v', 1.0)  # Default value for velocity
        self.declare_parameter('d', 0.0)  # Default value for steering angle

        # Get the parameters
        self.velocity = self.get_parameter('v').get_parameter_value().double_value
        self.steering_angle = self.get_parameter('d').get_parameter_value().double_value

        # Publisher to control the car using AckermannDriveStamped
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)

    def publishAckermannMsg(self):

        msg = AckermannDriveStamped()
        msg.drive.speed = self.velocity
        msg.drive.steering_angle = self.steering_angle
        self.publisher.publish(msg)
        #self.get_logger().info(f'Publishing: Speed={self.velocity}, Steering Angle={self.steering_angle}')

def main(args=None):

    rclpy.init(args=args)
    node = Talker()
    while rclpy.ok():
        node.publishAckermannMsg() 
    rclpy.spin(node)

if __name__ == "__main__":

    main()
    

    
        
