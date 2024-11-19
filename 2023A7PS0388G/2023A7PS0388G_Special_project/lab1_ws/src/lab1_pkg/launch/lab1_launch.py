import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('v', default_value='1.0', description='Speed Parameter'),
        DeclareLaunchArgument('d', default_value='0.0', description='Steering Angle Parameter'),
        Node(
            package='lab1_pkg',
            executable='talker',
            name='talker',
            output='screen',
            parameters=[{'v': 1.0, 'd': 0.0}]
        ),
        Node(
            package='lab1_pkg',
            executable='relay',
            name='relay',
            output='screen',
        ),
    ])
