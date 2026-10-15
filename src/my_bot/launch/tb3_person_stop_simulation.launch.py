"""Preload a real person, then move it into view during the navigation demo."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory('my_bot')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                share, 'launch', 'tb3_astar_yolo_simulation.launch.py')),
            launch_arguments={'person_demo': 'true', 'world': os.path.join(
                share, 'worlds', 'tb3_person_stop.world')}.items()),
    ])
