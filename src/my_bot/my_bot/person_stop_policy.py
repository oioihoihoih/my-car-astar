"""Clock-independent rules for the camera-based motion interlock."""

import math


class PersonStopPolicy:
    def __init__(self, detection_timeout=1.0, max_frame_age=1.0,
                 command_timeout=0.25, clear_time=1.0, confidence=0.35,
                 auto_arm=False, restored_reason=None):
        for value in (detection_timeout, max_frame_age, command_timeout, clear_time):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Safety timeouts must be positive and finite')
        if not 0 < confidence <= 1:
            raise ValueError('Person confidence must be in (0, 1]')
        self.detection_timeout = detection_timeout
        self.max_frame_age = max_frame_age
        self.command_timeout = command_timeout
        self.clear_time = clear_time
        self.confidence = confidence
        self.reason = restored_reason or (None if auto_arm else 'Manual arming required')
        self.last_report = self.last_stamp = self.clear_since = None
        self.clear_count = 0
        self.ever_ready = False
        self.command = None
        self.command_time = None

    def trip(self, reason):
        if self.reason is None or self.reason == 'Manual arming required' or reason == 'Person detected':
            self.reason = reason
        self.command = self.command_time = None

    def observe(self, report, now, ros_now):
        try:
            if report['schema_version'] != 1 or 'person' not in report['monitored_classes']:
                raise ValueError('Detector must include the person class')
            if not 0 < report['confidence_threshold'] <= self.confidence:
                raise ValueError('Detector threshold must include the safety threshold')
            stamp = report['header']['stamp']
            sec, nsec = stamp['sec'], stamp['nanosec']
            if type(sec) is not int or type(nsec) is not int or not 0 <= nsec < 10**9:
                raise ValueError('Invalid capture stamp')
            capture = sec * 10**9 + nsec
            if capture <= 0:
                raise ValueError('Missing capture stamp')
            objects = report['detections']
            if not isinstance(objects, list):
                raise ValueError('Invalid detections')
            person = False
            for obj in objects:
                confidence = obj['confidence']
                if (type(confidence) not in (int, float)
                        or not math.isfinite(confidence) or not 0 <= confidence <= 1):
                    raise ValueError('Invalid detection confidence')
                if (obj['label'] == 'person' or obj['class_id'] == 0) and confidence >= self.confidence:
                    person = True
        except (KeyError, TypeError, ValueError, OverflowError):
            self.clear_since, self.clear_count = None, 0
            self.trip('Invalid perception data or person detection disabled')
            return
        if person:
            self.clear_since, self.clear_count = None, 0
            self.trip('Person detected')
        # Repeated/reordered messages cannot keep the watchdog alive.
        if self.last_stamp is not None and capture <= self.last_stamp:
            return
        age = (ros_now - capture) / 1e9
        if age < -0.2 or age > self.max_frame_age:
            self.clear_since, self.clear_count = None, 0
            if self.ever_ready:
                self.trip('Camera frame is stale or clocks disagree')
            return
        if self.last_report is None or now - self.last_report > self.detection_timeout:
            self.clear_since, self.clear_count = None, 0
        self.last_report, self.last_stamp = now, capture
        if not person:
            if self.clear_since is None:
                self.clear_since = now
            self.clear_count += 1

    def check(self, now):
        if self.last_report is None or now - self.last_report > self.detection_timeout:
            self.clear_since, self.clear_count = None, 0
            if self.ever_ready:
                self.trip('Perception stream lost')
            return False, self.reason or 'Waiting for fresh camera inference'
        if self.reason:
            return False, self.reason
        if not self.ever_ready:
            if not self.clear_ready(now):
                return False, 'Waiting for clear camera observations'
            self.ever_ready = True
        return True, 'Armed'

    def clear_ready(self, now):
        return (self.last_report is not None
                and now - self.last_report <= self.detection_timeout
                and self.clear_since is not None and self.clear_count >= 3
                and now - self.clear_since >= self.clear_time)

    def reset(self, now):
        if not self.clear_ready(now):
            return False, 'Need fresh, continuously clear person detections before reset'
        self.reason = None
        self.ever_ready = True
        self.command = self.command_time = None
        return True, 'Reset accepted; send a new navigation goal'

    def set_command(self, values, now):
        if len(values) != 6 or not all(math.isfinite(v) for v in values):
            self.trip('Invalid velocity command')
            return
        if any(abs(values[i]) > 1e-9 for i in (1, 2, 3, 4)):
            self.trip('Unsupported differential-drive command')
            return
        self.command, self.command_time = tuple(values), now

    def output(self, now):
        allowed, reason = self.check(now)
        if (not allowed or self.command_time is None
                or now - self.command_time > self.command_timeout):
            return (0.0,) * 6, reason
        return self.command, reason
