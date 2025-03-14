from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    servo_node = Node(
        package='rpi',
        executable='servo_controller',
        name='servo_controller',
        output='screen',
        parameters=[
            {
                "servo_gpio": 26,
                "servo_topic": 'cam1_servo',
            },
        ]
    )

    return LaunchDescription([
        servo_node,
    ])
