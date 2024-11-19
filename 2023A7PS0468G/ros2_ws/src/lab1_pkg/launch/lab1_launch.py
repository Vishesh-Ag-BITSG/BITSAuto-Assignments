# my_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='talker',
            name='talker_node',
            parameters=[{'v': 1.0, 'd': 0.5}],
            output='screen'
        ),
        Node(
            package='my_package',
            executable='relay',
            name='relay_node',
            output='screen'
        )
    ])

