"""Check person latching, stale observations, and explicit rearming rules."""

import math

import pytest

from my_bot.person_stop_policy import PersonStopPolicy


def report(t, person=False):
    stamp = round(t * 10**9)
    return {
        'schema_version': 1, 'monitored_classes': ['person', 'stop sign'],
        'confidence_threshold': 0.35,
        'header': {'stamp': {'sec': stamp // 10**9, 'nanosec': stamp % 10**9}},
        'detections': [{'class_id': 0, 'label': 'person', 'confidence': 0.8}] if person else [],
    }


def clear_observations(policy, start=10.0):
    for i in range(7):
        t = start + i * 0.2
        policy.observe(report(t), t, round(t * 10**9))
    return t


def armed():
    policy = PersonStopPolicy(auto_arm=True)
    t = clear_observations(policy)
    assert policy.check(t)[0]
    return policy, t


def test_person_requires_clear_observations_and_manual_reset():
    policy, t = armed()
    policy.set_command((0.1, 0, 0, 0, 0, 0), t)
    assert policy.output(t)[0][0] > 0
    policy.observe(report(t + 0.1, True), t + 0.1, round((t + 0.1) * 10**9))
    assert policy.output(t + 0.1)[0] == (0.0,) * 6
    assert not policy.reset(t + 0.1)[0]
    t = clear_observations(policy, t + 0.2)
    assert not policy.check(t)[0]  # Leaving the camera cannot release the latch.
    assert policy.reset(t)[0]
    assert policy.output(t)[0] == (0.0,) * 6  # No buffered motion after reset.
    policy.set_command((0.1, 0, 0, 0, 0, 0), t + 0.01)
    assert policy.output(t + 0.01)[0][0] > 0


def test_duplicate_capture_stamps_cannot_refresh_watchdog_even_if_ros_clock_pauses():
    policy, t = armed()
    for i in range(1, 8):
        policy.observe(report(t), t + i * 0.2, round(t * 10**9))
    assert not policy.check(t + 1.4)[0]
    assert policy.reason == 'Perception stream lost'


def test_late_frame_does_not_count_as_a_fresh_heartbeat():
    policy, t = armed()
    policy.observe(report(t + 0.1), t + 0.1, round((t + 2) * 10**9))
    assert not policy.check(t + 0.1)[0]
    assert 'stale' in policy.reason


@pytest.mark.parametrize('change', ['class_filter', 'threshold', 'missing_stamp', 'nan_confidence'])
def test_invalid_or_person_blind_detector_cannot_arm(change):
    policy, t = armed()
    data = report(t + 0.1)
    if change == 'class_filter':
        data['monitored_classes'] = ['stop sign']
    elif change == 'threshold':
        data['confidence_threshold'] = 0.8
    elif change == 'missing_stamp':
        data['header']['stamp']['sec'] = 0
        data['header']['stamp']['nanosec'] = 0
    else:
        data['detections'] = [{'label': 'person', 'class_id': 0, 'confidence': math.nan}]
    policy.observe(data, t + 0.1, round((t + 0.1) * 10**9))
    assert not policy.check(t + 0.1)[0]


def test_restart_cannot_clear_a_saved_person_latch():
    policy = PersonStopPolicy(auto_arm=True, restored_reason='Person detected')
    t = clear_observations(policy)
    assert not policy.check(t)[0]
    assert policy.reset(t)[0]


def test_command_timeout_outputs_zero_without_replaying_old_command():
    policy, t = armed()
    policy.set_command((0.1, 0, 0, 0, 0, 0), t)
    assert policy.output(t + 0.3)[0] == (0.0,) * 6


def test_nonfinite_motion_is_latched_off():
    policy, t = armed()
    policy.set_command((math.nan, 0, 0, 0, 0, 0), t)
    assert policy.output(t)[0] == (0.0,) * 6
    assert policy.reason == 'Invalid velocity command'
