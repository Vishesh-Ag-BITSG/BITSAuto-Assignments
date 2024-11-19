import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)

        # Timer to publish messages at a fixed rate
        self.timer = self.create_timer(0.01, self.publish_drive_message)

        # Declare parameters with default values
        self.declare_parameter('v', 0.0)  
        self.declare_parameter('d', 0.0)  

    def publish_drive_message(self):
        # Get the latest parameter values
        v = self.get_parameter('v').value
        d = self.get_parameter('d').value

        # Create and populate the message
        msg = AckermannDriveStamped()
        msg.drive.speed = v
        msg.drive.steering_angle = d
        self.publisher_.publish(msg)

        # Log the published message
        self.get_logger().info(f'Publishing: speed={v}, steering_angle={d}')

def main(args=None):
    rclpy.init(args=args)
    talker_node = TalkerNode()

    # Spin the node
    rclpy.spin(talker_node)

    # Clean up and shutdown
    talker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

