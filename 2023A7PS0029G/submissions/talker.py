import rospy
from ackermann_msgs.msg import AckermannDriveStamped

def talker():
    rospy.init_node('talker', anonymous=True)

    # Declare the parameters
    v = rospy.get_param('~v', 0.0)  # Default value is 0.0 if not set
    d = rospy.get_param('~d', 0.0)  # Default value is 0.0 if not set

    pub = rospy.Publisher('drive', AckermannDriveStamped, queue_size=10)

    rate = rospy.Rate(10)  # Publish at 10 Hz

    while not rospy.is_shutdown():
        msg = AckermannDriveStamped()
        msg.drive.speed = v
        msg.drive.steering_angle = d

        rospy.loginfo(f"Publishing speed: {v}, steering_angle: {d}")
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass