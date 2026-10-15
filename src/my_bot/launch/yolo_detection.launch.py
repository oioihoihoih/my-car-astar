"""Run the detector and image viewer alongside an existing robot simulation."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def check_environment(context, workspace):
    python = Path(LaunchConfiguration('yolo_python').perform(context))
    if not python.is_file():
        raise RuntimeError(
            f'YOLO interpreter is missing: {python}. Run scripts/setup_yolo.py first.')
    (workspace / '.yolo-config').mkdir(parents=True, exist_ok=True)
    (workspace / '.cache/matplotlib').mkdir(parents=True, exist_ok=True)
    return []


def generate_launch_description():
    bot_share = get_package_share_directory('my_bot')
    workspace = Path(bot_share).parents[3]
    return LaunchDescription([
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument(
            'yolo_python', default_value=str(workspace / '.venv-yolo/bin/python3')),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('confidence', default_value='0.35'),
        DeclareLaunchArgument('input_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('target_class', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('image_size', default_value='640'),
        DeclareLaunchArgument('max_fps', default_value='5.0'),
        OpaqueFunction(function=check_environment, kwargs={'workspace': workspace}),
        SetEnvironmentVariable('YOLO_CONFIG_DIR', str(workspace / '.yolo-config')),
        SetEnvironmentVariable('MPLCONFIGDIR', str(workspace / '.cache/matplotlib')),
        SetEnvironmentVariable('PYTHONNOUSERSITE', '1'),
        Node(
            package='my_bot', executable='yolo_camera', name='yolo_camera',
            prefix=LaunchConfiguration('yolo_python'),
            parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'input_topic': LaunchConfiguration('input_topic'),
                'model_path': os.path.join(bot_share, 'weights', 'yolo11n.pt'),
                'confidence': ParameterValue(
                    LaunchConfiguration('confidence'), value_type=float),
                'device': LaunchConfiguration('device'),
                'target_class': ParameterValue(LaunchConfiguration('target_class'), value_type=str),
                'image_size': ParameterValue(LaunchConfiguration('image_size'), value_type=int),
                'max_fps': ParameterValue(LaunchConfiguration('max_fps'), value_type=float),
            }], output='screen'),
        Node(
            package='rqt_image_view', executable='rqt_image_view',
            arguments=['/yolo/image'],
            condition=IfCondition(LaunchConfiguration('viewer')),
            output='screen'),
    ])
