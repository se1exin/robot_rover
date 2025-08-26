import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = "robot_tank_desc"
    
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'false'}.items()
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
        # remappings=[('/cmd_vel', '/diff_cont/cmd_vel')]
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': False}],
        remappings=[
            ('/cmd_vel_in','/cmd_vel'),
            ('/cmd_vel_out','/diff_cont/cmd_vel')
            ]
        )


    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    diffdrive_pubsub = Node(
        package='diffdrive_pubsub',
        executable='diffdrive_pubsub',
    )

    # Run the node
    return LaunchDescription(
        [
            rsp,
            joy_node,
            teleop_node,
            delayed_diff_drive_spawner,
            delayed_joint_broad_spawner,
            twist_stamper,
            diffdrive_pubsub
        ]
    )
