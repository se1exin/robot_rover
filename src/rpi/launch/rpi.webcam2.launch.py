from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    webcam_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='webcam_2',
        output='screen',
        namespace='webcam_2',
        parameters=[
            {
                "video_device": os.environ.get("WEBCAM_2", ""),
                "image_size": [1280,720],
            },
        ]
    )


    return LaunchDescription([
        webcam_node,
    ])
