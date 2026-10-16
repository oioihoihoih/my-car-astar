"""Reject stale, replayed, malformed and uncertain remote model responses."""

import math
import uuid
import pytest
from my_bot.laya_protocol import LayaDecisionPolicy, describe_state


def reply(**changes):
    return dict({'schema_version': 1, 'request_id': str(uuid.uuid4()), 'backend': 'laya',
                 'model_id': 'convaiinnovations/laya', 'action': 'continue',
                 'confidence': 0.9, 'request_age_s': 0.2}, **changes)


def test_expire_from_request_start_and_ignore_replay():
    policy = LayaDecisionPolicy()
    assert not policy.check(10)[0]
    report = reply(request_age_s=1.0)
    policy.observe(report, 10)
    assert policy.check(10)[0]
    policy.observe(report, 11.5)
    assert not policy.check(12.1)[0]


@pytest.mark.parametrize('fields', [
    {'action': 'go_fast'}, {'action': 'wait'}, {'action': 'stop'},
    {'confidence': 0.4}, {'confidence': math.nan}, {'confidence': True},
    {'request_age_s': 3.1}, {'request_age_s': -1.0}, {'backend': 'mock'},
])
def test_model_cannot_allow_motion_with_unsafe_reply(fields):
    policy = LayaDecisionPolicy()
    policy.observe(reply(**fields), 10)
    assert not policy.check(10)[0]


def test_new_valid_reply_recovers_policy_but_not_a_robot_latch():
    policy = LayaDecisionPolicy()
    policy.observe({}, 10)
    assert not policy.check(10)[0]
    policy.observe(reply(), 10.1)
    assert policy.check(10.1)[0]


def test_state_schema_does_not_accept_text_in_place_of_sensor_flags():
    with pytest.raises(ValueError):
        describe_state({'camera_fresh': 'true'})
