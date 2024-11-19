cmake_minimum_required(VERSION 3.8)
project(lab1_pkg)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# Add executables
add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp ackermann_msgs)

add_executable(relay src/relay.cpp)
ament_target_dependencies(relay rclcpp ackermann_msgs)

# Install the executables
install(TARGETS
    talker
    relay
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/
)


ament_package()
