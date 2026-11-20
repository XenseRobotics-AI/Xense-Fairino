"""Physical stick clicks and X/Y double taps, independent of arm routing."""
import math
import time

LEVELS = (.25, .5, 1.)
TAP_MAX_SECONDS = .25
DOUBLE_TAP_GAP_SECONDS = .35
INPUT_GAP_SECONDS = .25


def sensitivity_step(value, direction):
    if direction > 0:
        return next((v for v in LEVELS if v > value+1e-6), max(value, LEVELS[-1]))
    return next((v for v in reversed(LEVELS) if v < value-1e-6), min(value, LEVELS[0]))


class _DoubleTap:
    def __init__(self):
        self.reset()

    def reset(self):
        self.armed = self.down = self.second = False
        self.pressed_at = self.first_release = None

    def update(self, down, now):
        fired = False
        if not self.armed:
            self.armed = not down  # Held at startup/reconnect is not a tap.
        elif down and not self.down:
            self.pressed_at = now
            self.second = (self.first_release is not None
                           and 0 <= now-self.first_release <= DOUBLE_TAP_GAP_SECONDS)
            if not self.second:
                self.first_release = None
        elif not down and self.down:
            short = self.pressed_at is not None and 0 < now-self.pressed_at <= TAP_MAX_SECONDS
            fired = short and self.second
            self.first_release = now if short and not fired else None
            self.pressed_at, self.second = None, False
        elif down and self.pressed_at is not None and now-self.pressed_at > TAP_MAX_SECONDS:
            self.first_release, self.second = None, False
        self.down = down
        return fired


class JoystickControls:
    def __init__(self, *, right_a_home=False):
        self.right_a_home = right_a_home
        self.last_sample = None
        self.reset()

    def reset(self):
        self.armed = False
        self.a_released = False
        self.left_click = self.right_click = False
        self.taps = {'primary': _DoubleTap(), 'secondary': _DoubleTap()}

    def update(self, frame, now=None):
        now = time.monotonic() if now is None else now
        result = {"sensitivity": 0, "home": False, "calibrate_heading": False}
        # Consume real receive timestamps so buffered press/release edges survive
        # a slower IK tick. Repeated 450 Hz reads of one SDK frame add no taps.
        for sample in frame.get('samples') or [frame]:
            stamp = sample.get('received_monotonic', now)
            if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
                self.reset()
                continue
            if self.last_sample is not None and stamp <= self.last_sample:
                continue
            if now-stamp > INPUT_GAP_SECONDS:
                self.reset()
                continue
            if self.last_sample is not None:
                if stamp-self.last_sample > INPUT_GAP_SECONDS:
                    self.reset()
            self.last_sample = stamp
            left, right = sample['left'], sample['right']
            home_edge = False
            if self.right_a_home:
                down = right.get('primary', False)
                home_edge = down and self.a_released
                self.a_released = not down
            lc, rc = left.get('axis_click', False), right.get('axis_click', False)
            if not self.armed:
                self.armed = not lc and not rc
            else:
                home_edge |= rc and not self.right_click
                result['calibrate_heading'] |= lc and not self.left_click
            if home_edge:
                if self.right_a_home and result['home']:
                    result['cancel_home'] = True
                result['home'] = True
            self.left_click, self.right_click = lc, rc
            x, y = left.get('primary', False), left.get('secondary', False)
            if lc or rc or (x and y):
                for tap in self.taps.values():
                    tap.reset()
                continue
            # A different button interrupts the pair; holding both still freezes
            # both Cartesian components but can never change sensitivity.
            if x:
                self.taps['secondary'].reset()
            elif y:
                self.taps['primary'].reset()
            for button, direction in (('primary', -1), ('secondary', 1)):
                if self.taps[button].update(left.get(button, False), stamp):
                    if not result['sensitivity']:
                        result['sensitivity'] = direction
                    for tap in self.taps.values():
                        tap.reset()
        # Calibration wins even if the same batch contains a Home or tap edge.
        if result['calibrate_heading']:
            result['home'] = False
            result.pop('cancel_home', None)
        if result['calibrate_heading'] or result['home']:
            result['sensitivity'] = 0
            for tap in self.taps.values():
                tap.reset()
        return result
