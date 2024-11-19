#!/usr/bin/env python
import rospy
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('v', default_value='1.0', description='Speed parameter'),
        DeclareLaunchArgument('d', default_value='0.5', description='Steering angle parameter'),

        Node(
            package='lab1_pkg',
            executable='talker.py',
            name='talker',
            output='screen',
            parameters=[{'v': LaunchConfiguration('v'), 'd': LaunchConfiguration('d')}]
        ),

        Node(
            package='lab1_pkg',
            executable='relay.py',
            name='relay',
            output='screen'
        )
    ])