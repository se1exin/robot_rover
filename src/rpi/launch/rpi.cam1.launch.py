from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Raspberry Pi Camera Module
    rpi_cam_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='cam_1',
        output='screen',
        parameters=[
            {
                "camera": "/base/axi/pcie@120000/rp1/i2c@88000/ov5647@36",
            },
        ]
    )

    return LaunchDescription([
        rpi_cam_node,
    ])
