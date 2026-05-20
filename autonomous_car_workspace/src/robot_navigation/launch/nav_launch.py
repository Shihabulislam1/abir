import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Find package share directory
    pkg_share = FindPackageShare('robot_navigation')

    # Declare configuration parameters file path launch argument
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'robot_params.yml']),
        description='Path to the ROS2 parameters file'
    )

    # Serial Bridge Node
    serial_bridge_node = Node(
        package='robot_navigation',
        executable='serial_bridge',
        name='serial_bridge_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    # Brain Node
    brain_node = Node(
        package='robot_navigation',
        executable='brain',
        name='brain_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    # Lidar Monitor Node
    lidar_monitor_node = Node(
        package='robot_navigation',
        executable='lidar_monitor',
        name='lidar_monitor_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    # Vision Node
    vision_node = Node(
        package='robot_navigation',
        executable='vision',
        name='vision_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    return LaunchDescription([
        params_file_arg,
        serial_bridge_node,
        brain_node,
        lidar_monitor_node,
        vision_node
    ])
