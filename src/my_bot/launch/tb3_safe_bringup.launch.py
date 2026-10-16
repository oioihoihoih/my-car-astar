"""Physical TurtleBot3 bringup with a robot-local motion gate and watchdog."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    bot_share = get_package_share_directory('my_bot')
    tb3_bringup = get_package_share_directory('turtlebot3_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('require_laya', default_value='false'),
        DeclareLaunchArgument('laya_server', default_value='http://127.0.0.1:8765/decide'),
        GroupAction([
            SetRemap(src='cmd_vel', dst='/cmd_vel_safe'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
                tb3_bringup, 'launch', 'robot.launch.py'))),
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                bot_share, 'launch', 'person_safety_gate.launch.py')),
            launch_arguments={'use_sim_time': 'false', 'auto_arm_startup': 'false',
                              'motor_power_on_trip': 'true',
                              'require_laya': LaunchConfiguration('require_laya'),
                              'output_cmd_topic': '/cmd_vel_safe'}.items()),
        Node(package='my_bot', executable='actuator_watchdog',
             parameters=[{'use_sim_time': False}], output='screen'),
        Node(package='my_bot', executable='laya_bridge',
             condition=IfCondition(LaunchConfiguration('require_laya')),
             parameters=[{'use_sim_time': False,
                          'server_url': LaunchConfiguration('laya_server')}], output='screen'),
    ])
