from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    talker = Node(
        package='lab1_pkg',
        executable='talker.py',
        parameters= [
            {'v': 2.0},
            {'d': 1.0}
        ]
    )

    relay = Node(
        package='lab1_pkg',
        executable='relay.py'
    )

    ld.add_action(talker)
    ld.add_action(relay)

    return ld