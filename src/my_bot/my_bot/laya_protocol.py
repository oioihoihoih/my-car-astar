"""Small, ROS-independent contract for local or remote Laya decisions."""

from collections import deque
import math


MODEL_ID = 'convaiinnovations/laya'
MODEL_REVISION = '7b928d828b7b0e022f929d9bd2e44165aa270148'
ACTIONS = ('continue', 'wait', 'stop')
QUESTIONS = {
    'motion': {
        'type': 'choice',
        'instructions': 'Choose the motion policy for this mobile robot from the observed state.',
        'criteria': {
            'continue': 'The camera and lidar are fresh, the person is absent, and the path is clear.',
            'wait': 'A temporary obstacle blocks the path; stay stationary and wait.',
            'stop': 'A person is present or camera or lidar data is unavailable; stay stationary.',
        },
    },
}


def finite_number(value, minimum=0.0, maximum=math.inf):
    return (type(value) in (int, float) and math.isfinite(value)
            and minimum <= value <= maximum)


def validate_state(state):
    if not isinstance(state, dict):
        raise ValueError('state must be an object')
    for key in ('camera_fresh', 'lidar_fresh', 'person_visible', 'front_clear'):
        if type(state.get(key)) is not bool:
            raise ValueError(f'{key} must be a boolean')
    distance = state.get('front_distance_m')
    if distance is not None and not finite_number(distance):
        raise ValueError('Invalid front_distance_m')
    return {key: state[key] for key in
            ('camera_fresh', 'lidar_fresh', 'person_visible', 'front_clear')} | {
                'front_distance_m': distance}


def describe_state(state):
    s = validate_state(state)
    return ('Camera data: ' + ('fresh' if s['camera_fresh'] else 'unavailable')
            + '. Lidar data: ' + ('fresh' if s['lidar_fresh'] else 'unavailable')
            + '. Person: ' + ('present' if s['person_visible'] else 'absent')
            + '. Path ahead: ' + ('clear' if s['front_clear'] else 'blocked') + '.')


class LayaDecisionPolicy:
    """Expire decisions by request age; replayed replies never renew a lease."""

    def __init__(self, timeout=3.0, min_confidence=0.7):
        if not finite_number(timeout, 0.05) or not finite_number(min_confidence, 0.0, 1.0):
            raise ValueError('Invalid Laya decision thresholds')
        self.timeout, self.min_confidence = timeout, min_confidence
        self.last_request = None
        self.action = None
        self.confidence = 0.0
        self.error = None
        self.seen = deque(maxlen=128)

    def observe(self, report, now):
        try:
            request_id = report['request_id']
            if not isinstance(request_id, str) or not 1 <= len(request_id) <= 64:
                raise ValueError('Invalid request id')
            if request_id in self.seen:
                return
            if report['schema_version'] != 1 or report['backend'] != 'laya':
                raise ValueError('A real Laya response is required')
            action, confidence = report['action'], report['confidence']
            age = report['request_age_s']
            if (action not in ACTIONS or not finite_number(confidence, 0.0, 1.0)
                    or not finite_number(age, 0.0, self.timeout)):
                raise ValueError('Invalid or stale Laya response')
            if not isinstance(report['model_id'], str) or not report['model_id']:
                raise ValueError('Missing Laya model identity')
        except (KeyError, TypeError, ValueError):
            self.error = 'Invalid or stale Laya decision'
            return
        self.seen.append(request_id)
        self.last_request = now - age
        self.action, self.confidence, self.error = action, confidence, None

    def check(self, now):
        if self.error:
            return False, self.error
        if self.last_request is None:
            return False, 'Waiting for a fresh Laya decision'
        if now - self.last_request > self.timeout:
            return False, 'Laya communication or inference timed out'
        if self.action != 'continue':
            return False, f'Laya requested {self.action}'
        if self.confidence < self.min_confidence:
            return False, 'Laya choice probability is below threshold'
        return True, 'Laya allows navigation'
