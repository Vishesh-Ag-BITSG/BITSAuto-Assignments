import rospy
from ackermann_msgs.msg import AckermannDriveStamped

def callback(msg):
    # Modify speed and steering_angle by multiplying them by 3
    msg.drive.speed *= 3
    msg.drive.steering_angle *= 3

    rospy.loginfo(f"Received speed: {msg.drive.speed}, steering_angle: {msg.drive.steering_angle}")
    pub.publish(msg)

def relay():
    rospy.init_node('relay', anonymous=True)

    # Subscribe to the 'drive' topic
    rospy.Subscriber('drive', AckermannDriveStamped, callback)

    # Publisher for the 'drive_relay' topic
    global pub
    pub = rospy.Publisher('drive_relay', AckermannDriveStamped, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        relay()
    except rospy.ROSInterruptException:
        pass