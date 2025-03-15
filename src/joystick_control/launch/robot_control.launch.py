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

    rqt_image_cam_1 = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_cam_1',
        arguments=['/cam_1/image_raw/compressed'],
        output='screen'
    )

    rqt_image_cam_2 = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_cam_2',
        arguments=['/cam_2/image_raw/compressed'],
        output='screen'
    )

    rqt_image_webcam_1 = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_webcam_1',
        arguments=['/webcam_1/image_raw/compressed'],
        output='screen'
    )

    rqt_image_webcam_2 = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_webcam_2',
        arguments=['/webcam_2/image_raw/compressed'],
        output='screen'
    )

    

    return LaunchDescription([
        joy_node,
        xbox_controller_node,
        rqt_image_cam_1,
        rqt_image_cam_2,
        # rqt_image_webcam_1,
        # rqt_image_webcam_2,
    ])
