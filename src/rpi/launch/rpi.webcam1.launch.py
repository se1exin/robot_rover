from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    webcam_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='webcam_1',
        output='screen',
        namespace='webcam_1',
        parameters=[
            {
                "video_device": os.environ.get("WEBCAM_1", ""),
                "image_size": [1280,720],
                # "output_encoding": "yuv422_yuy2"
            },
        ]
    )


    return LaunchDescription([
        webcam_node,
    ])
