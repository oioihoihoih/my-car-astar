"""Navigate with A* and Pure Pursuit while detecting camera objects with YOLO."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bot_share = get_package_share_directory('my_bot')
    launch_dir = os.path.join(bot_share, 'launch')
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='waffle_pi',
                              choices=['waffle', 'waffle_pi']),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('demo', default_value='true',
                              description='Send one example goal after localization and YOLO are ready'),
        DeclareLaunchArgument('demo_goal_x', default_value='0.3'),
        DeclareLaunchArgument('demo_goal_y', default_value='-0.5'),
        DeclareLaunchArgument('demo_goal_yaw', default_value='0.0'),
        DeclareLaunchArgument('person_demo', default_value='false'),
        DeclareLaunchArgument('require_laya', default_value='false'),
        DeclareLaunchArgument('world', default_value=os.path.join(
            bot_share, 'worlds', 'tb3_astar_yolo.world')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                launch_dir, 'person_safety_gate.launch.py')),
            launch_arguments={'use_sim_time': 'true', 'auto_arm_startup': 'true',
                              'output_cmd_topic': '/cmd_vel', 'clear_time': '3.0',
                              'require_laya': LaunchConfiguration('require_laya')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                launch_dir, 'yolo_detection.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                launch_dir, 'tb3_astar_simulation.launch.py')),
            launch_arguments={
                'model': LaunchConfiguration('model'),
                'gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
                'world': LaunchConfiguration('world'),
                'map': os.path.join(bot_share, 'maps', 'tb3_world_yolo.yaml'),
                'rviz_config': os.path.join(bot_share, 'rviz', 'astar_yolo.rviz'),
                'cmd_vel_topic': '/cmd_vel_nav',
                'require_safety': 'true',
            }.items()),
        Node(
            package='my_bot', executable='navigation_demo_goal',
            name='navigation_demo_goal',
            condition=IfCondition(LaunchConfiguration('demo')),
            parameters=[{
                'use_sim_time': True,
                'require_safety': True,
                'goal_x': ParameterValue(LaunchConfiguration('demo_goal_x'), value_type=float),
                'goal_y': ParameterValue(LaunchConfiguration('demo_goal_y'), value_type=float),
                'goal_yaw': ParameterValue(LaunchConfiguration('demo_goal_yaw'), value_type=float),
            }], output='screen'),
        Node(
            package='my_bot', executable='spawn_demo_person',
            condition=IfCondition(LaunchConfiguration('person_demo')),
            parameters=[{'use_sim_time': True}], output='screen'),
    ])
