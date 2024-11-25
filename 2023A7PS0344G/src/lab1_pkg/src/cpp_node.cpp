#include "rclcpp/rclcpp.hpp"
#include "lab1_pkg/cpp_header.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyCustomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}