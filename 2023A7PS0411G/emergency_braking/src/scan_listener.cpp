#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <memory>
#include <cmath> 

class ScanListener : public rclcpp::Node
{
public:
    ScanListener() : Node("scan_listener")
    {   
      cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      laser_scan_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
          this, "/scan");
      odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
          this, "/ego_racecar/odom");

      sync_ = std::make_shared<Sync>(SyncPolicy(10), *laser_scan_sub_, *odom_sub_);
      sync_->registerCallback(std::bind(&ScanListener::callback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& laser_scan_msg,
                  const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg)
    {
        // Process synchronized data here
        int ranges_size = laser_scan_msg->ranges.size();
        std::vector<float> ttc_values(ranges_size);
        double vel = odom_msg->twist.twist.linear.x;

        if(vel > 0) {
          float angle = -2.3499999046325684;
          float increment = 0.004351851996034384;
          for(int i = 0; i < ranges_size; i++) {
            double relative_vel = vel * std::cos(angle + i*increment);
            if(relative_vel) {
              ttc_values[i] = laser_scan_msg->ranges[i] / relative_vel;
            }
          }
        }


        float threshold = 1.2; // Define your threshold value here
        if (std::any_of(ttc_values.begin(), ttc_values.end(), [threshold](float ttc) {
            return ttc < threshold && ttc > 0; // Check if any value is less than the threshold
        })) {
          auto zero_vel = geometry_msgs::msg::Twist();
          zero_vel.linear.x = 0;
          cmd_vel_publisher_->publish(zero_vel);
          RCLCPP_INFO(this->get_logger(), "TTC below threshold. Velocity set to zero.");
         }  
    }

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, nav_msgs::msg::Odometry>;
    using Sync = message_filters::Synchronizer<SyncPolicy>;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> laser_scan_sub_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    std::shared_ptr<Sync> sync_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanListener>());
    rclcpp::shutdown();
    return 0;
}