"""Launch a real Gazebo stop-sign model, TurtleBot camera, and YOLO viewer."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def robot_nodes(context, tb3_share):
    model = LaunchConfiguration('model').perform(context)
    with open(os.path.join(tb3_share, 'urdf', f'turtlebot3_{model}.urdf')) as stream:
        description = stream.read()
    return [
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'use_sim_time': True, 'robot_description': description}],
            output='screen'),
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=[
                '-entity', model,
                '-file', os.path.join(tb3_share, 'models', f'turtlebot3_{model}', 'model.sdf'),
                '-x', '0.0', '-y', '0.0', '-z', '0.01'],
            output='screen'),
    ]


def check_environment(context, workspace):
    python = Path(LaunchConfiguration('yolo_python').perform(context))
    if not python.is_file():
        raise RuntimeError(
            f'YOLO interpreter is missing: {python}. Run scripts/setup_yolo.py first.')
    (workspace / '.yolo-config').mkdir(parents=True, exist_ok=True)
    (workspace / '.cache/matplotlib').mkdir(parents=True, exist_ok=True)
    return []


def join_paths(*paths):
    return os.pathsep.join(dict.fromkeys(p for p in paths if p))


def generate_launch_description():
    bot_share = get_package_share_directory('my_bot')
    tb3_share = get_package_share_directory('turtlebot3_gazebo')
    gazebo_share = get_package_share_directory('gazebo_ros')
    workspace = Path(bot_share).parents[3]
    gazebo_resources = '/usr/share/gazebo-11'
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='waffle_pi',
                              choices=['waffle', 'waffle_pi']),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument(
            'yolo_python', default_value=str(workspace / '.venv-yolo/bin/python3')),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('confidence', default_value='0.35'),
        OpaqueFunction(function=check_environment, kwargs={'workspace': workspace}),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', join_paths(
            os.environ.get('GAZEBO_RESOURCE_PATH', ''), gazebo_resources)),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', join_paths(
            os.path.join(bot_share, 'models'),
            os.path.join(tb3_share, 'models'),
            os.path.join(gazebo_resources, 'models'),
            os.environ.get('GAZEBO_MODEL_PATH', ''))),
        SetEnvironmentVariable(
            'YOLO_CONFIG_DIR', str(workspace / '.yolo-config')),
        SetEnvironmentVariable('PYTHONNOUSERSITE', '1'),
        SetEnvironmentVariable(
            'MPLCONFIGDIR', str(workspace / '.cache/matplotlib')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                gazebo_share, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': os.path.join(
                bot_share, 'worlds', 'yolo_detection.world')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                gazebo_share, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui'))),
        OpaqueFunction(function=robot_nodes, kwargs={'tb3_share': tb3_share}),
        Node(
            package='my_bot', executable='yolo_camera', name='yolo_camera',
            prefix=LaunchConfiguration('yolo_python'),
            parameters=[{
                'use_sim_time': True,
                'model_path': os.path.join(bot_share, 'weights', 'yolo11n.pt'),
                'confidence': ParameterValue(LaunchConfiguration('confidence'), value_type=float),
                'device': LaunchConfiguration('device'),
                'target_class': 'stop sign',
                'max_fps': 5.0,
            }], output='screen'),
        Node(
            package='rqt_image_view', executable='rqt_image_view',
            arguments=['/yolo/image'],
            condition=IfCondition(LaunchConfiguration('viewer')),
            output='screen'),
    ])
