#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class SafetyNode:
    def __init__(self):
        rospy.init_node('safety_node', anonymous=True)

        # Subscribe to sensor data (e.g., distance sensor)
        self.distance_sub = rospy.Subscriber('/sensor/distance', Float32, self.distance_callback)

        # Publisher for safety control (e.g., stop the robot if unsafe)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.safety_threshold = 1.0  # Safety threshold for stopping (meters)
        self.current_distance = None

    def distance_callback(self, msg):
        self.current_distance = msg.data
        rospy.loginfo(f"Current distance: {self.current_distance} meters")

        # Safety logic: Stop the robot if distance is below threshold
        if self.current_distance < self.safety_threshold:
            self.stop_robot()

    def stop_robot(self):
        # Send a stop command to the robot
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        rospy.loginfo("Safety threshold exceeded! Stopping the robot.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        safety_node = SafetyNode()
        safety_node.run()
    except rospy.ROSInterruptException:
        pass