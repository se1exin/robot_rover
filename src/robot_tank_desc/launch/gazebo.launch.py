import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = "robot_tank_desc"

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    world = os.path.join(get_package_share_directory(pkg_name), "worlds", "basic.sdf")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "robot", "-z", "0.1"],
        output="screen",
    )

    bridge_params = os.path.join(
        get_package_share_directory(pkg_name), "config", "gz_bridge.yaml"
    )
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
        output="screen",
    )

    ros_gz_cam_1_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/cam_1/image_raw"],
    )

    ros_gz_cam_2_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/cam_2/image_raw"],
    )


    joy_params = os.path.join(
        get_package_share_directory(pkg_name), "config", "joystick.yaml"
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[joy_params],
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_node",
        parameters=[joy_params],
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/cmd_vel_in','/cmd_vel'),
            ('/cmd_vel_out','/diff_cont/cmd_vel')
            ]
        )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    diff_drive_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    # Run the node
    return LaunchDescription(
        [
            rsp,
            gazebo,
            ros_gz_bridge,
            ros_gz_cam_1_image_bridge,
            ros_gz_cam_2_image_bridge,
            spawn_entity,
            joy_node,
            teleop_node,
            joint_state_broadcaster_spawner,
            diff_drive_base_controller_spawner,
            twist_stamper
        ]
    )
