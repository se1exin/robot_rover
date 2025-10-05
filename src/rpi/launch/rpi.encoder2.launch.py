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
                "encoder_pin": 25,
                "angle_topic": '/right_motor_angle',
                "direction_topic": '/right_motor_direction',
                "delay_topic": '/right_track_delay',
            },
        ]
    )

    return LaunchDescription([
        encoder_monitor,
    ])
