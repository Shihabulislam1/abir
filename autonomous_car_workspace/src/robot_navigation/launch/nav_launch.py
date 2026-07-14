import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource

def generate_launch_description():
    # Find package share directory
    pkg_share = FindPackageShare('robot_navigation')

    # Declare configuration parameters file path launch argument
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'robot_params.yml']),
        description='Path to the ROS2 parameters file'
    )

    # Rosbridge WebSocket Server
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
            ])
        )
    )

    # RPLidar
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rplidar_ros'),
                'launch',
                'rplidar_a2m12_launch.py'
            ])
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': '256000'
        }.items()
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

    # Sign Detector Node
    sign_detector_node = Node(
        package='robot_navigation',
        executable='sign_detector',
        name='sign_detector_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    return LaunchDescription([
        params_file_arg,
        rosbridge_launch,
        rplidar_launch,
        serial_bridge_node,
        brain_node,
        lidar_monitor_node,
        vision_node,
        sign_detector_node
    ])
