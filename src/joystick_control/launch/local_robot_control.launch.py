from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the joystick driver node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0'
            }]
        ),

        # Start the joystick control node
        Node(
            package='joystick_control',
            executable='joystick_control',
            name='joystick_control',
            output='screen'
        ),

        # Start the Arduino communication node
        Node(
            package='arduino_comm',
            executable='arduino_comm',
            name='arduino_comm',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyACM0',
                'baud_rate': 9600
            }]
        ),
    ])
