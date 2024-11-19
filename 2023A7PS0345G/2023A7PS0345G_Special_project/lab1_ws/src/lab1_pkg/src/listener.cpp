#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>

class Listener : public rclcpp::Node
{
public:
    Listener() : Node("listener")
    {
        subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "topic_name", 10,
            std::bind(&Listener::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received: speed=%.2f, steering_angle=%.2f", msg->speed, msg->steering_angle);
    }

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Listener>());
    rclcpp::shutdown();
    return 0;
}
