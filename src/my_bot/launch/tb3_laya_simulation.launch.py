"""Use real local/remote Laya decisions with the existing Gazebo safety gate."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('my_bot')
    return LaunchDescription([
        DeclareLaunchArgument('laya_server', default_value='http://127.0.0.1:8765/decide'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                share, 'launch', 'tb3_astar_yolo_simulation.launch.py')),
            launch_arguments={'require_laya': 'true'}.items()),
        Node(package='my_bot', executable='laya_bridge',
             parameters=[{'use_sim_time': True,
                          'server_url': LaunchConfiguration('laya_server')}], output='screen'),
    ])
