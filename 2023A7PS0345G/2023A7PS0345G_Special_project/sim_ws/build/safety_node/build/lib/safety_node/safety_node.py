import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.velocity = 0.0
    
    def odom_callback(self, msg):
        self.velocity = msg.twist.twist.linear.x  # Get longitudinal velocity

    def scan_callback(self, msg):
        ranges = msg.ranges
        iTTCs = self.calculate_iTTC(ranges)

        if any(iTTC < threshold for iTTC in iTTCs):
            self.stop_car()

    def calculate_iTTC(self, ranges):
        # Implement iTTC calculation here
        iTTCs = []
        for r in ranges:
            # Add range rate and iTTC calculation logic
            iTTC = max(r / -range_rate, 0)
            iTTCs.append(iTTC)
        return iTTCs

    def stop_car(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.0  # Stop the car
        self.drive_publisher.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
