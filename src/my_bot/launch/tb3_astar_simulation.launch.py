"""Start Gazebo, saved-map AMCL, RViz, and the exercise A* controller."""

import os

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
                '-x', LaunchConfiguration('x_pose'),
                '-y', LaunchConfiguration('y_pose'),
                '-Y', LaunchConfiguration('yaw'), '-z', '0.01'],
            output='screen'),
    ]


def generate_launch_description():
    bot_share = get_package_share_directory('my_bot')
    tb3_share = get_package_share_directory('turtlebot3_gazebo')
    gazebo_share = get_package_share_directory('gazebo_ros')
    settings = os.path.join(bot_share, 'config', 'astar_simulation.yaml')
    model_default = os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=model_default,
                              choices=['burger', 'waffle', 'waffle_pi']),
        DeclareLaunchArgument('map', default_value=os.path.join(
            bot_share, 'maps', 'tb3_world.yaml')),
        DeclareLaunchArgument('world', default_value=os.path.join(
            tb3_share, 'worlds', 'turtlebot3_world.world')),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(
            bot_share, 'rviz', 'astar_simulation.rviz')),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Open the Gazebo client'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('auto_initial_pose', default_value='true',
                              description='Initialize AMCL at the Gazebo spawn pose'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('require_safety', default_value='false'),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', os.pathsep.join(filter(None, [
            os.environ.get('GAZEBO_RESOURCE_PATH', ''), '/usr/share/gazebo-11']))),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join(filter(None, [
            os.path.join(bot_share, 'models'), os.path.join(tb3_share, 'models'),
            '/usr/share/gazebo-11/models', os.environ.get('GAZEBO_MODEL_PATH', '')]))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                gazebo_share, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                gazebo_share, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui'))),
        OpaqueFunction(function=robot_nodes, kwargs={'tb3_share': tb3_share}),
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            parameters=[{'use_sim_time': True, 'yaml_filename': LaunchConfiguration('map')}],
            output='screen'),
        Node(
            package='nav2_amcl', executable='amcl', name='amcl',
            parameters=[settings, {
                'use_sim_time': True,
                'set_initial_pose': ParameterValue(
                    LaunchConfiguration('auto_initial_pose'), value_type=bool),
                'initial_pose.x': ParameterValue(LaunchConfiguration('x_pose'), value_type=float),
                'initial_pose.y': ParameterValue(LaunchConfiguration('y_pose'), value_type=float),
                'initial_pose.yaw': ParameterValue(LaunchConfiguration('yaw'), value_type=float),
            }], output='screen'),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[{'use_sim_time': True, 'autostart': True,
                         'node_names': ['map_server', 'amcl']}], output='screen'),
        Node(
            package='my_bot', executable='astar_pure_pursuit',
            name='astar_pure_pursuit', parameters=[settings, {
                'use_sim_time': True,
                'require_safety': ParameterValue(LaunchConfiguration('require_safety'), value_type=bool),
            }], remappings=[('/cmd_vel', LaunchConfiguration('cmd_vel_topic'))],
            output='screen'),
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
