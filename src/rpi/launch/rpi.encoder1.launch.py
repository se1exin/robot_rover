from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    encoder_monitor = Node(
        package='rpi',
        executable='encoder_monitor',
        name='encoder_monitor_1',
        output='screen',
        parameters=[
            {
                "encoder_pin": 24,
                "angle_topic": '/left_motor_angle',
                "direction_topic": '/left_motor_direction',
                "delay_topic": '/left_track_delay',
            },
        ]
    )

    return LaunchDescription([
        encoder_monitor,
    ])
