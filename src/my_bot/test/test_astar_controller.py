"""Verify motion gating when sensor data or a replacement goal is invalid."""

import math
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from my_bot.astar_pure_pursuit import AStarPurePursuit


@pytest.fixture
def controller():
    rclpy.init()
    node = AStarPurePursuit()
    commands = []
    node.cmd_pub = SimpleNamespace(publish=lambda msg: commands.append(
        (msg.linear.x, msg.angular.z)))
    node.update_robot_pose = lambda: True
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.width = msg.info.height = 30
    msg.info.resolution = 0.1
    msg.info.origin.orientation.w = 1.0
    msg.data = [0] * 900
    node.map_callback(msg)
    node.current_pose = [1.0, 1.0]
    yield node, commands
    node.destroy_node()
    rclpy.shutdown()


def set_goal(node, x=2.0, frame='map'):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.pose.position.x, msg.pose.position.y = x, 1.0
    msg.pose.orientation.w = 1.0
    node.goal_callback(msg)


def scan(node, value):
    msg = LaserScan()
    msg.angle_min, msg.angle_increment = -math.pi, math.pi / 180
    msg.range_min, msg.range_max = 0.1, 5.0
    msg.ranges = [float(value)] * 360
    node.scan_callback(msg)


def test_wait_for_localization_and_fresh_scan(controller):
    node, commands = controller
    set_goal(node)
    node.control_loop()
    assert commands[-1] == (0.0, 0.0)
    node.localized = True
    node.control_loop()
    assert commands[-1] == (0.0, 0.0)
    assert 'fresh /scan' in node.status
    scan(node, math.inf)  # A valid no-return measurement means clear to range_max.
    node.last_scan_time -= int(2e9)
    node.control_loop()
    assert commands[-1] == (0.0, 0.0)
    scan(node, 2.0)
    node.control_loop()
    assert commands[-1][0] > 0.0


def test_invalid_scan_and_obstacle_stop_then_resume(controller):
    node, commands = controller
    node.localized = True
    set_goal(node)
    for value in (math.nan, 0.0, 0.2):
        scan(node, value)
        node.control_loop()
        assert commands[-1] == (0.0, 0.0)
    scan(node, 2.0)
    node.control_loop()
    assert commands[-1][0] > 0.0


def test_invalid_replacement_goal_cancels_old_motion(controller):
    node, commands = controller
    node.localized = True
    scan(node, 2.0)
    set_goal(node)
    node.control_loop()
    assert commands[-1][0] > 0.0
    set_goal(node, frame='odom')
    assert node.goal is None and node.waypoints == []
    assert commands[-1] == (0.0, 0.0)
    set_goal(node, x=10.0)
    node.control_loop()
    assert node.goal is None and 'outside the map' in node.status
    assert commands[-1] == (0.0, 0.0)


def test_occupied_goal_and_stale_transform_stop(controller):
    node, commands = controller
    node.localized = True
    scan(node, 2.0)
    node.blocked[node.geometry.world_to_grid(2.0, 1.0)] = True
    set_goal(node)
    node.control_loop()
    assert node.goal is None and 'occupied' in node.status
    assert commands[-1] == (0.0, 0.0)
    node.blocked[node.geometry.world_to_grid(2.0, 1.0)] = False
    set_goal(node)
    node.control_loop()
    assert commands[-1][0] > 0.0
    node.update_robot_pose = lambda: False
    node.control_loop()
    assert commands[-1] == (0.0, 0.0)


def test_person_interlock_cancels_goal_and_requires_a_new_goal(controller):
    node, commands = controller
    node.require_safety = True
    node.localized = True
    scan(node, 2.0)
    set_goal(node)
    node.control_loop()
    assert commands[-1][0] > 0.0
    node.safety_callback(Bool(data=True))
    assert node.goal is None and node.waypoints == []
    assert commands[-1] == (0.0, 0.0)
    set_goal(node)
    assert node.goal is None
    node.safety_callback(Bool(data=False))
    node.control_loop()
    assert node.goal is None and commands[-1] == (0.0, 0.0)
    set_goal(node)
    node.control_loop()
    assert commands[-1][0] > 0.0
