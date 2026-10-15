"""Localization, A*/Pure Pursuit, and detection for a physical robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('my_bot')
    settings = os.path.join(share, 'config', 'astar_simulation.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to the real environment map YAML'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'yolo_detection.launch.py')),
            launch_arguments={'use_sim_time': 'false', 'target_class': ''}.items()),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'use_sim_time': False, 'yaml_filename': LaunchConfiguration('map')}],
             output='screen'),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             parameters=[settings, {'use_sim_time': False, 'set_initial_pose': False}],
             output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization',
             parameters=[{'use_sim_time': False, 'autostart': True,
                          'node_names': ['map_server', 'amcl']}], output='screen'),
        Node(package='my_bot', executable='astar_pure_pursuit', name='astar_pure_pursuit',
             parameters=[settings, {'use_sim_time': False, 'require_safety': True}],
             remappings=[('/cmd_vel', '/cmd_vel_nav')], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'astar_yolo.rviz')],
             parameters=[{'use_sim_time': False}],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
