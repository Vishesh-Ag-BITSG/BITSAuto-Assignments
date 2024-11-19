import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        # Publisher
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Initialize parameters
        self.velocity = 0.0
        self.previous_ranges = None
        self.iTTC_threshold = 2.0  # Adjust this threshold as necessary

    def lidar_callback(self, msg):
        # Get ranges from LaserScan message
        ranges = np.array(msg.ranges)
        current_time = self.get_clock().now().nanoseconds / 1e9  # Convert to seconds

        # Calculate iTTC
        iTTCs = self.calculate_iTTC(ranges, current_time)
        
        # Check for collision
        if np.any(iTTCs < self.iTTC_threshold):
            self.publish_brake()

    def odom_callback(self, msg):
        # Get the longitudinal velocity
        self.velocity = msg.twist.twist.linear.x

    def calculate_iTTC(self, ranges, current_time):
        if self.previous_ranges is None:
            self.previous_ranges = ranges
            return np.full_like(ranges, np.inf)  # First iteration, return infinities
        
        # Calculate range rate
        delta_time = 0.1  # This should match your timer rate
        range_rate = (ranges - self.previous_ranges) / delta_time
        
        # Update previous ranges
        self.previous_ranges = ranges
        
        # Calculate iTTC
        iTTCs = np.maximum(-range_rate + ranges, 0) / np.maximum(ranges, 1e-6)  # Prevent division by zero
        return iTTCs

    def publish_brake(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Braking: speed set to 0.0 m/s')

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

