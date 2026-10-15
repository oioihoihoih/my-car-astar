from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'my_bot'


def resource_tree(directory):
    return [
        (os.path.join('share', package_name, root),
         [os.path.join(root, name) for name in files])
        for root, _, files in os.walk(directory) if files
    ]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'weights'), glob('weights/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),

    ] + resource_tree('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wlgh',
    maintainer_email='wlgh@todo.todo',
    description='TurtleBot3 A* and Pure Pursuit navigation with YOLO person detection and motion gating',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'go_to_goal = my_bot.go_to_goal:main',
            'pure_pursuit_node = my_bot.pure_purshit_node:main',
            'pure_pursuit_obstacle=my_bot.pure_pursuit_obstacle:main',
            'astar_pure_pursuit = my_bot.astar_pure_pursuit:main',
            'yolo_camera = my_bot.yolo_camera:main',
            'navigation_demo_goal = my_bot.navigation_demo_goal:main',
            'person_safety_gate = my_bot.person_safety_gate:main',
            'spawn_demo_person = my_bot.spawn_demo_person:main',
            'actuator_watchdog = my_bot.actuator_watchdog:main',
        ],
    },
)
