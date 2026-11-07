"""ROS-free observed drive-command feedback; this is not a vendor controller phase."""
import math
import re

TOPIC = '/fairino_navigation/chassis_motion_state'
STATUS_NAME = 'fairino_slam/chassis_motion'
FIELDS = ('schema_version', 'sequence', 'ready', 'drive_wait', 'hardware_age_sec',
          'drive_command_peak', 'drive_velocity_peak', 'requested_vx', 'requested_vy', 'requested_wz')
DRIVE = (1, 3, 5, 7)
INTENT_EPSILON = 1e-4
DRIVE_EPSILON = 1e-6


def motion_values(hardware, requested, ready, hardware_age):
    """Build finite observations from the last Twist actually sent downstream."""
    requested = tuple(float(v) for v in requested)
    if len(requested) != 3 or not all(math.isfinite(v) for v in requested):
        raise ValueError('invalid requested Twist')
    age = float(hardware_age)
    if not math.isfinite(age) or age < 0:
        raise ValueError('invalid hardware age')
    peaks = []
    for field in ('command_velocity', 'velocity'):
        values = (hardware or {}).get(field, [0.] * 8)
        if len(values) != 8 or not all(math.isfinite(v) for v in values):
            raise ValueError('invalid drive feedback')
        peaks.append(max(abs(float(values[i])) for i in DRIVE))
    ready = bool(ready and hardware and all(hardware.get('enabled', []))
                 and len(hardware.get('enabled', [])) == 8)
    intent = max(map(abs, requested)) > INTENT_EPSILON
    waiting = ready and intent and peaks[0] <= DRIVE_EPSILON
    message = 'not_ready' if not ready else 'idle' if not intent else 'drive_wait' if waiting else 'drive_active'
    values = dict(ready=ready, drive_wait=waiting, hardware_age_sec=age,
                  drive_command_peak=peaks[0], drive_velocity_peak=peaks[1],
                  requested_vx=requested[0], requested_vy=requested[1], requested_wz=requested[2])
    return message, values


def wire_values(values, sequence):
    """The exact KeyValue contract shared by worker, gate and C++ controller."""
    result = {'schema_version': '1', 'sequence': str(sequence)}
    for key, value in values.items():
        result[key] = ('1' if value else '0') if key in ('ready', 'drive_wait') else repr(float(value))
    return result


class FeedbackMonitor:
    def __init__(self, timeout=.4):
        if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('invalid feedback timeout')
        self.timeout = float(timeout)
        self._value = None
        self._received = None
        self._stamp = None
        self._sequences = {}
        self._invalid = None

    def receive(self, message, now_monotonic, now_ros):
        """Reject invalid/replayed messages without refreshing freshness; never raise."""
        try:
            if not all(math.isfinite(v) for v in (now_monotonic, now_ros)) or now_ros <= 0:
                raise ValueError('invalid receiver clock')
            if len(message.status) != 1:
                raise ValueError('expected one diagnostic status')
            status = message.status[0]
            if status.name != STATUS_NAME or not re.fullmatch('[0-9a-f]{32}', status.hardware_id):
                raise ValueError('invalid status name or worker session')
            fields = {}
            for pair in status.values:
                if pair.key in fields:
                    raise ValueError('duplicate field')
                fields[pair.key] = pair.value
            if set(fields) != set(FIELDS) or fields['schema_version'] != '1':
                raise ValueError('unsupported feedback schema')
            if not re.fullmatch(r'[1-9][0-9]*', fields['sequence']):
                raise ValueError('invalid sequence')
            sequence = int(fields['sequence'])
            session = status.hardware_id
            if sequence <= self._sequences.get(session, -1):
                raise ValueError('repeated or regressed sequence')
            for key in ('ready', 'drive_wait'):
                if fields[key] not in ('0', '1'):
                    raise ValueError('invalid boolean')
            value = {key: float(fields[key]) for key in FIELDS[4:]}
            if not all(math.isfinite(v) for v in value.values()):
                raise ValueError('non-finite feedback')
            if any(value[key] < 0 for key in FIELDS[4:7]):
                raise ValueError('negative feedback age or peak')
            ready, waiting = fields['ready'] == '1', fields['drive_wait'] == '1'
            intent = max(abs(value[k]) for k in FIELDS[7:]) > INTENT_EPSILON
            expected_wait = ready and intent and value['drive_command_peak'] <= DRIVE_EPSILON
            expected_message = 'not_ready' if not ready else 'idle' if not intent else 'drive_wait' if waiting else 'drive_active'
            # ROS DiagnosticStatus.level is an IDL octet, represented in Python
            # as one byte (not int, unlike the corresponding C++ uint8_t field).
            if not isinstance(status.level, (bytes, bytearray)) or len(status.level) != 1:
                raise ValueError('invalid diagnostic level byte')
            level = status.level[0]
            if (waiting != expected_wait or level != (0 if ready else 2)
                    or status.message != expected_message):
                raise ValueError('inconsistent feedback state')
            stamp = message.header.stamp
            if (isinstance(stamp.sec, bool) or isinstance(stamp.nanosec, bool)
                    or not isinstance(stamp.sec, int) or not isinstance(stamp.nanosec, int)
                    or stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000):
                raise ValueError('invalid ROS stamp')
            timestamp = stamp.sec + stamp.nanosec * 1e-9
            if timestamp <= 0 or now_ros - timestamp < -.05 or now_ros - timestamp > self.timeout:
                raise ValueError('stale or future ROS stamp')
            if value['hardware_age_sec'] + max(0., now_ros-timestamp) > self.timeout:
                raise ValueError('stale hardware feedback')
            self._value = dict(value, ready=ready, drive_wait=waiting, session=session,
                               sequence=sequence, schema_version=1, message=status.message)
            self._received, self._stamp = float(now_monotonic), timestamp
            self._sequences[session] = sequence
            self._invalid = None
            return True
        except (AttributeError, TypeError, ValueError, OverflowError) as exc:
            self._invalid = '底盘运动反馈无效：' + str(exc)
            return False

    def snapshot(self, now_monotonic, now_ros):
        value = dict(self._value or {})
        reason = self._invalid
        if reason is None:
            if self._value is None:
                reason = '等待底盘运动反馈'
            elif not all(math.isfinite(v) for v in (now_monotonic, now_ros)) or now_ros <= 0:
                reason = '底盘运动反馈时钟无效'
            else:
                elapsed = now_monotonic - self._received
                ros_age = now_ros - self._stamp
                if elapsed < 0 or ros_age < -.05:
                    reason = '底盘运动反馈时钟回退'
                elif elapsed > self.timeout or ros_age > self.timeout:
                    reason = '底盘运动反馈已过期'
                elif value['hardware_age_sec'] + max(0., elapsed, ros_age) > self.timeout:
                    reason = '底盘硬件反馈已过期'
                elif not value['ready']:
                    reason = '底盘运动反馈未就绪'
        value.update(stamp=self._stamp,
                     sample_age_sec=now_ros-self._stamp if self._stamp is not None else None,
                     received_age=now_monotonic-self._received if self._received is not None else None)
        transport_age = max(0., value['sample_age_sec'] or 0., value['received_age'] or 0.)
        value.update(hardware_total_age_sec=(value['hardware_age_sec']+transport_age
                                             if 'hardware_age_sec' in value else None),
                     feedback_timeout_sec=self.timeout)
        value.update(ready=reason is None, reason=reason,
                     session=value.get('session'), drive_wait=bool(value.get('drive_wait')) if reason is None else False)
        return value
