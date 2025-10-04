from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Find the package where the config files are located
    pkg_share = FindPackageShare(package='robot_tank_desc').find('robot_tank_desc')

    # Define the path to the parameter file
    params_file = PathJoinSubstitution([pkg_share, 'config', 'mapper_params_online_async.yaml'])

    # Declare a launch argument to allow custom parameter files
    param_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=params_file,
        description='Path to the parameter file for SLAM Toolbox'
    )

    # Node configuration for slam_toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',  # Make sure the executable name matches
        name='slam_toolbox',
        output='screen',
        parameters=[LaunchConfiguration('slam_params_file')],
    )

    # Create and return LaunchDescription object
    return LaunchDescription([
        param_file_arg,
        slam_toolbox_node
    ])
