import rclpy
from rclpy.node import Node
from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode

def generate_launch_description():
    return LaunchDescription([
        LaunchNode(
            package='lab1_pkg',
            executable='talker',
            name='talker',
            output='screen',
            parameters=[{'v': 1.0, 'd': 0.5}]  # Set your desired parameters here
        ),
        LaunchNode(
            package='lab1_pkg',
            executable='relay',
            name='relay',
            output='screen'
        ),
    ])
