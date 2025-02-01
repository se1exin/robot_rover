from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the stepper driver node
        Node(
            package='rpi',
            executable='rpi_motors',
            name='rpi_motors',
            output='screen'
        ),

        # Start the realsense cam node
        Node(
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
                }
            ]
        ),
    ])
