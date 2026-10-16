"""Exercise real ROS gate callbacks with simulated model/perception messages."""

import json
from types import SimpleNamespace
import time
import uuid

from geometry_msgs.msg import Twist
import pytest
import rclpy
from std_msgs.msg import String

from my_bot.person_safety_gate import PersonSafetyGate


def model_reply(action='continue', confidence=0.95):
    return {'schema_version': 1, 'request_id': str(uuid.uuid4()), 'backend': 'laya',
            'model_id': 'convaiinnovations/laya', 'action': action,
            'confidence': confidence, 'request_age_s': 0.01}


@pytest.fixture
def gate(tmp_path):
    rclpy.init(args=['--ros-args', '-p', 'require_laya:=true',
                     '-p', 'auto_arm_startup:=true', '-p', 'clear_time:=0.1',
                     '-p', 'state_file:=' + str(tmp_path / 'latch.json')])
    node = PersonSafetyGate()
    commands = []
    node.output_pub = SimpleNamespace(publish=lambda m: commands.append((m.linear.x, m.angular.z)))
    now = time.monotonic()
    ros_now = node.get_clock().now().nanoseconds
    for offset in (0.5, 0.3, 0.02):
        stamp = ros_now - int(offset * 1e9)
        node.policy.observe({'schema_version': 1, 'confidence_threshold': 0.35,
                             'monitored_classes': ['person'],
                             'header': {'stamp': {'sec': stamp // 10**9, 'nanosec': stamp % 10**9}},
                             'detections': []}, now - offset, ros_now)
    command = Twist()
    command.linear.x = 0.05
    node.on_command(command)
    yield node, commands
    node.destroy_node()
    rclpy.shutdown()


def test_model_is_required_before_motion(gate):
    node, commands = gate
    node.tick()
    assert commands[-1] == (0.0, 0.0) and not node.policy.ever_ready
    node.on_laya(String(data=json.dumps(model_reply())))
    assert commands[-1][0] == 0.05


def test_network_loss_latches_and_reconnection_requires_explicit_reset(gate):
    node, commands = gate
    node.on_laya(String(data=json.dumps(model_reply())))
    assert commands[-1][0] == 0.05
    node.laya_policy.last_request -= 4.0
    node.tick()
    assert commands[-1] == (0.0, 0.0)
    assert 'timed out' in node.policy.reason
    node.on_laya(String(data=json.dumps(model_reply())))
    assert commands[-1] == (0.0, 0.0) and node.policy.reason
    response = SimpleNamespace(success=False, message='')
    node.on_reset(None, response)
    assert response.success
    assert node.policy.command is None  # Old motion cannot resume on reset.


def test_person_stop_has_priority_even_when_model_allows_motion(gate):
    node, commands = gate
    node.on_laya(String(data=json.dumps(model_reply())))
    assert commands[-1][0] == 0.05
    node.policy.trip('Person detected')
    node.on_laya(String(data=json.dumps(model_reply())))
    assert commands[-1] == (0.0, 0.0)
    assert node.policy.reason == 'Person detected'


def test_model_wait_latches_while_robot_is_armed(gate):
    node, commands = gate
    node.on_laya(String(data=json.dumps(model_reply())))
    node.on_laya(String(data=json.dumps(model_reply('wait'))))
    assert commands[-1] == (0.0, 0.0)
    assert node.policy.reason == 'Laya requested wait'
