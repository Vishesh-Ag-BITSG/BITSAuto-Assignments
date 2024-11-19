#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

class SafetyNode
{
public:
    SafetyNode()
    {
        // Initialize the ROS node
        ros::NodeHandle nh;

        // Subscribe to sensor data
        distance_sub = nh.subscribe("/sensor/distance", 10, &SafetyNode::distanceCallback, this);

        // Publisher for cmd_vel (stop the robot if unsafe)
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        safety_threshold = 1.0;  // Safety threshold in meters
    }

    void distanceCallback(const std_msgs::Float32::ConstPtr& msg)
    {
        current_distance = msg->data;
        ROS_INFO("Current distance: %f meters", current_distance);

        // Safety logic: Stop the robot if distance is below threshold
        if (current_distance < safety_threshold)
        {
            stopRobot();
        }
    }

    void stopRobot()
    {
        // Send a stop command to the robot
        geometry_msgs::Twist stop_msg;
        cmd_vel_pub.publish(stop_msg);
        ROS_WARN("Safety threshold exceeded! Stopping the robot.");
    }

private:
    ros::Subscriber distance_sub;
    ros::Publisher cmd_vel_pub;
    double safety_threshold;
    double current_distance;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "safety_node");

    SafetyNode safety_node;

    ros::spin();

    return 0;
}