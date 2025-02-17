from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    motor_node = Node(
        package='rpi',
        executable='rpi_motors',
        name='rpi_motors',
        output='screen'
    )

    return LaunchDescription([
        motor_node,
    ])
