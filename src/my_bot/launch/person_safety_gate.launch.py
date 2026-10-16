"""Run this interlock on the robot SBC; detector/navigation may run remotely."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('auto_arm_startup', default_value='false'),
        DeclareLaunchArgument('motor_power_on_trip', default_value='false'),
        DeclareLaunchArgument('input_cmd_topic', default_value='/cmd_vel_nav'),
        DeclareLaunchArgument('output_cmd_topic', default_value='/cmd_vel_safe'),
        DeclareLaunchArgument('clear_time', default_value='1.0'),
        DeclareLaunchArgument('require_laya', default_value='false'),
        DeclareLaunchArgument('laya_timeout', default_value='3.0'),
        DeclareLaunchArgument('laya_min_confidence', default_value='0.7'),
        Node(
            package='my_bot', executable='person_safety_gate', name='person_safety_gate',
            parameters=[{
                name: ParameterValue(LaunchConfiguration(name), value_type=bool)
                for name in ['use_sim_time', 'auto_arm_startup', 'motor_power_on_trip', 'require_laya']
            }, {
                'input_cmd_topic': LaunchConfiguration('input_cmd_topic'),
                'output_cmd_topic': LaunchConfiguration('output_cmd_topic'),
                'clear_time': ParameterValue(LaunchConfiguration('clear_time'), value_type=float),
                'laya_timeout': ParameterValue(LaunchConfiguration('laya_timeout'), value_type=float),
                'laya_min_confidence': ParameterValue(
                    LaunchConfiguration('laya_min_confidence'), value_type=float),
            }], output='screen'),
    ])
