from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Raspberry Pi NoIR Camera Module
    rpi_cam_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='cam_2',
        output='screen',
        parameters=[
            {
                "camera": "/base/axi/pcie@120000/rp1/i2c@80000/ov5647@36",  # Rpi NoIR Camera v1.3
                "format": "XRGB8888",
                "width": 1280,
                "height": 1024
            },
        ]
    )

    return LaunchDescription([
        rpi_cam_node,
    ])
