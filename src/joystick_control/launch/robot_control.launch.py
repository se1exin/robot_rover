from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0'
        }]
    )

    xbox_controller_node = Node(
        package='joystick_control',
        executable='joystick_control',
        name='joystick_control',
        output='screen'
    )

    # First rqt_image_view instance (for webcam_1)
    rqt_image_view_1 = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view_1',
        arguments=['/webcam_1/image_raw/compressed'],
        output='screen'
    )

    # Second rqt_image_view instance (for webcam_2)
    rqt_image_view_2 = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view_2',
        arguments=['/webcam_2/image_raw/compressed'],
        output='screen'
    )

    rqt_image_view_3 = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view_3',
        arguments=['/camera/realsense2_camera_node/depth/image_rect_raw/compressed'],
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        xbox_controller_node,
        rqt_image_view_1,
        rqt_image_view_2,
        # rqt_image_view_3,
    ])
