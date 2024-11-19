#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

using namespace std::chrono_literals;  // This line allows using the "ms" literal

class Talker : public rclcpp::Node
{
public:
    Talker()
        : Node("talker")
    {
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&Talker::publish_message, this));  // Using 100ms here
    }

private:
    void publish_message()
    {
        auto message = ackermann_msgs::msg::AckermannDriveStamped();
        message.drive.speed = 0.5; // Set speed
        message.drive.steering_angle = 0.1; // Set steering angle
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: speed=%f, steering_angle=%f", message.drive.speed, message.drive.steering_angle);
    }

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Talker>());
    rclcpp::shutdown();
    return 0;
}
