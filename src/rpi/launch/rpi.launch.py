from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the stepper driver node
        # Node(
        #     package='rpi',
        #     executable='rpi_motors',
        #     name='rpi_motors',
        #     output='screen'
        # ),

        # Start the realsense cam node
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera_node',
            output='screen',
            parameters=[
                {
                    'enable_color': True,             # Enable color stream
                    'color_width': 640,              # Set color stream width
                    'color_height': 480,             # Set color stream height
                    'color_fps': 15,                 # Frames per second for color stream
                    'enable_depth': True,            # Enable depth stream
                    'depth_width': 640,              # Set depth stream width
                    'depth_height': 480,             # Set depth stream height
                    'depth_fps': 15,                 # Frames per second for depth stream
                }
            ]
        ),
    ])
