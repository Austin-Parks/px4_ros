#!/usr/bin/python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_ros',
            executable='px4_ros_node',
            name='uav1_px4_ros_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'rqtrepub': False}
            ]
        )
    ])

