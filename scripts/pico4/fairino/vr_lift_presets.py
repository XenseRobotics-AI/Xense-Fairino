"""One-second stick gestures selecting calibrated high/lower torso targets.

Pure trajectory mathematics: no ROS, device or transport access. The receiver
must enforce its existing input lease and call reset() on pause/disconnection.
Once triggered, neutral does not cancel a trajectory; sample() advances it from
the receiver's monotonic clock, even between new input packets.
"""
import bisect
import math

from body_core import _segment, sample_interval
from vr_lift import DEADBAND, FOLD, LiftPlanner, lift_axis

HOLD_SECONDS = 1.
SPEED_DEG_S = 30.
HIGH_PERCENT = 100.
# Raise the former 50% lower preset by one third of its distance to high.
LOW_PERCENT = 50. + (HIGH_PERCENT - 50.) / 3.
MAX_OBSERVATION_GAP_S = .25
TARGET_TOLERANCE_MM = 1.


class PresetLiftPlanner:
    def __init__(self, config):
        # Reuse calibration, folding-branch and pose validation from continuous
        # lift, but never use its incremental motion integration.
        self._validator = LiftPlanner(config)
        self.model = self._validator.model
        self.reset()

    @property
    def moving(self):
        return self._points is not None

    def reset(self):
        """Forget motion/gesture state; a neutral observation is required next."""
        self._points = None
        self._times = None
        self._started = None
        self._duration = 0.
        self._needs_neutral = True
        self._direction = 0
        self._hold_started = None
        self._last_observed = None
        self._last_gesture = None
        self._last_time = None
        self._target_percent = None
        self._effective_target_percent = None
        self._phase = 'waiting_neutral'
        self._message = '请先将升降摇杆回中'

    def _clock(self, now):
        if type(now) not in (int, float) or not math.isfinite(now):
            self.reset()
            raise ValueError('预设升降需要有限单调时间')
        now = float(now)
        if self._last_time is not None and now < self._last_time - 1e-9:
            self.reset()
            raise ValueError('预设升降时间不能倒退，请回中后重试')
        self._last_time = now
        return now

    def _check_command(self, command):
        try:
            self.model.check_q(command)
        except (ValueError, TypeError):
            self.reset()
            raise ValueError('预设升降需要完整的 20 轴有限指令') from None

    def _clear_candidate(self):
        self._direction = 0
        self._hold_started = None
        self._target_percent = None
        self._effective_target_percent = None
        self._duration = 0.

    def observe(self, axis, command, now, gesture_id=None):
        """Observe a fresh stick sample; hold one direction to trigger once.

        An optional validated gesture id carries direction changes from SDK
        batches even if only their latest axis reaches this observer.
        """
        now = self._clock(now)
        self._check_command(command)
        try:
            axis = lift_axis({'lift': axis})
        except ValueError:
            self.reset()
            raise
        gap = None if self._last_observed is None else now - self._last_observed
        self._last_observed = now
        changed_gesture = gesture_id is not None and gesture_id != self._last_gesture
        if gesture_id is not None:
            self._last_gesture = gesture_id
        if self.moving:
            # Do not queue gestures while moving. A new neutral sample after
            # completion is required, even if neutral was seen during motion.
            return
        if changed_gesture and self._phase == 'holding':
            # E.g. up -> neutral -> up was consumed in one SDK batch. It is a
            # new hold even though the latest scalar axis still points up.
            self._clear_candidate()
        neutral = abs(axis) <= DEADBAND
        if self._needs_neutral:
            if neutral:
                self._needs_neutral = False
                self._clear_candidate()
                self._phase, self._message = 'ready', '上推保持 1 秒到高位，下推保持 1 秒到下位'
            return
        if neutral:
            self._clear_candidate()
            self._phase, self._message = 'ready', '未触发；上推到高位，下推到下位'
            return
        direction = 1 if axis > 0 else -1
        if (direction != self._direction or self._hold_started is None
                or gap is None or gap > MAX_OBSERVATION_GAP_S + 1e-9):
            self._direction = direction
            self._hold_started = now
            self._target_percent = HIGH_PERCENT if direction > 0 else LOW_PERCENT
            self._effective_target_percent = None
            self._duration = 0.
        self._phase = 'holding'
        self._message = f'保持摇杆到满 1 秒：目标 {self._target_percent:.1f}%'
        if now - self._hold_started >= HOLD_SECONDS - 1e-9:
            self._needs_neutral = True
            try:
                self._begin(command, now)
            except (ValueError, ArithmeticError):
                self.reset()
                raise

    def _begin(self, command, now):
        current = self._validator.validated_state(command)
        # Hold this command's horizontal position rather than aligning it to
        # the nominal calibrated vertical line, which could make a sudden step.
        self.model.x, _ = self.model.fold_position(
            math.degrees(command[18]), math.degrees(command[19]))

        def height_at(knee_deg):
            radius2 = (self.model.l1**2 + self.model.l2**2
                       + 2*self.model.l1*self.model.l2*math.cos(math.radians(knee_deg)))
            return math.sqrt(max(0., radius2-self.model.x**2)) - self.model.z_low

        low_mm = max(0., height_at(self.model.low[1]))
        high_mm = min(self.model.travel, height_at(self.model.config['high_knee_deg']))
        if high_mm <= low_mm:
            raise ValueError('当前水平偏移没有有效的标定升降区间')
        target_mm = self.model.travel * self._target_percent / 100.
        target_mm = max(low_mm, min(high_mm, target_mm))
        self._effective_target_percent = target_mm/self.model.travel*100.
        if abs(target_mm-current['height_mm']) <= TARGET_TOLERANCE_MM:
            self._phase = 'at_target'
            self._message = f'已在目标 {self._effective_target_percent:.1f}% 的 1 mm 范围内；回中后重备'
            return
        target = {key: current[key] for key in ('height', 'forward', 'roll', 'yaw')}
        target['height'] = self._effective_target_percent
        # Apply the existing small encoder-noise tolerance to validation only;
        # preserve the commanded pitch/yaw/roll in the actual generated path.
        gate = target.copy()
        for key in ('forward', 'roll', 'yaw'):
            if abs(gate[key]) <= .3:
                gate[key] = 0.
        self.model.validate(gate)
        start = {key: current[key] for key in target}
        points = _segment(self.model, start, target, command, SPEED_DEG_S)
        if max(abs(points[0]['positions'][j]-command[j]) for j in FOLD) > 1e-6:
            raise ValueError('当前折叠角度与标定分支不一致，不能连续开始预设升降')
        self._points = points
        self._times = [point['time'] for point in points]
        self._started = now
        self._duration = self._times[-1]
        self._phase = 'moving'
        limited = abs(self._effective_target_percent-self._target_percent) > 1e-6
        self._message = (f'目标 {self._target_percent:.1f}%；按当前水平偏移限制为 '
                         f'{self._effective_target_percent:.1f}%' if limited
                         else f'前往目标 {self._target_percent:.1f}%')
        self._message += '；回中继续，暂停可中止'

    def sample(self, command, now):
        """Return current trajectory position, replacing only folding joints.

        moving becomes False only after this method has returned the endpoint
        once. The caller must publish that last returned position as well.
        """
        now = self._clock(now)
        self._check_command(command)
        result = list(command)
        if not self.moving:
            return result
        elapsed = max(0., now-self._started)
        if elapsed >= self._duration:
            q = self._points[-1]['positions']
            self._points = self._times = None
            self._hold_started = None
            self._needs_neutral = True
            self._phase = 'completed'
            self._message = '目标轨迹完成；回中后可触发下一目标'
        else:
            index = min(len(self._points)-2, max(0, bisect.bisect_right(self._times,elapsed)-1))
            q, _ = sample_interval(self._points[index], self._points[index+1], elapsed)
        for joint in FOLD:
            result[joint] = q[joint]
        return result

    def snapshot(self, now):
        if type(now) not in (int, float) or not math.isfinite(now):
            raise ValueError('预设升降状态需要有限时间')
        elapsed = (max(0., now-self._hold_started) if self._hold_started is not None else 0.)
        motion_elapsed = max(0., now-self._started) if self._started is not None else 0.
        return dict(phase=self._phase, message=self._message, moving=self.moving,
                    armed=not self._needs_neutral, direction=self._direction,
                    target_percent=self._target_percent,
                    effective_target_percent=self._effective_target_percent,
                    hold_elapsed_s=min(HOLD_SECONDS,elapsed), hold_seconds=HOLD_SECONDS,
                    duration_s=self._duration,
                    remaining_s=max(0.,self._duration-motion_elapsed) if self.moving else 0.,
                    speed_deg_s=SPEED_DEG_S)
