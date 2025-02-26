from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    lidar_node = Node(
        package='rpi',
        executable='fm1828_publisher',
        name='fm1828_publisher',
        output='screen'
    )

    return LaunchDescription([
        lidar_node,
    ])
