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
        Node(
            package='my_bot', executable='person_safety_gate', name='person_safety_gate',
            parameters=[{
                name: ParameterValue(LaunchConfiguration(name), value_type=bool)
                for name in ['use_sim_time', 'auto_arm_startup', 'motor_power_on_trip']
            }, {
                'input_cmd_topic': LaunchConfiguration('input_cmd_topic'),
                'output_cmd_topic': LaunchConfiguration('output_cmd_topic'),
                'clear_time': ParameterValue(LaunchConfiguration('clear_time'), value_type=float),
            }], output='screen'),
    ])
