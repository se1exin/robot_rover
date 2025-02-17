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

    webcam_1_node = Node(
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

    webcam_2_node = Node(
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

    rpi_cam_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera_node',
        output='screen',
    )

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera_node',
        output='screen',
        parameters=[
            {
                "initial_reset": True,
                "enable_rgbd": True,
                "enable_sync": True,
                "enable_infra1": False,
                "enable_infra2": False,
                "align_depth.enable": True,
                "enable_color": True,
                "enable_depth": True,
                "rgb_camera.color_profile": "320x180x6",
                "depth_module.depth_profile": "480x270x6",
                # "depth_module.infra_profile": "640x480x15",
                "publish_tf": False,
            }
        ]
    )


    return LaunchDescription([
        # webcam_1_node,
        # webcam_2_node,
        rpi_cam_node,
        motor_node,
        # realsense_node,
    ])
