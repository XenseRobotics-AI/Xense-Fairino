"""Joystick geometry and integration use no robot or network."""
import math
import copy
from types import SimpleNamespace as NS
import unittest

from vr_chassis import ChassisTeleop, joystick_axes


def frame(x=0., y=0., yaw=0., lift=0., stamp=10.):
    return dict(received_monotonic=stamp,
                left=dict(axis_x=x, axis_y=y, joystick_available=True, axis_click=False),
                right=dict(axis_x=yaw, axis_y=lift, joystick_available=True, axis_click=False))


class FakeSession:
    def __init__(self, *args, **kwargs):
        self.arguments = args, kwargs
        self.state = dict(ready=True, armed=True, hardware={'enabled': [True]*8})
        self.inputs = []
    def start(self):
        return self
    def snapshot(self):
        return self.state
    def set_axes(self, *args, **kwargs):
        self.inputs.append((args, kwargs))
    def close(self):
        self.closed = True
    def finish(self, timeout):
        self.close()
        return True


class ChassisTests(unittest.TestCase):
    # Expected outputs for physical left-stick up/right and right-stick right.
    # These values express the user convention independently from control_modes.
    DIRECTIONS = {
        'first_person': ((1., 0., 0.), (0., -1., 0.), (0., 0., -1.)),
        'mirror_same_direction': ((-1., 0., 0.), (0., 1., 0.), (0., 0., -1.)),
        'mirror': ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.)),
    }

    def test_all_modes_map_both_directions_of_each_physical_axis_and_analog_scale(self):
        for mode, expected_axes in self.DIRECTIONS.items():
            for axis, expected in zip(('y', 'x', 'yaw'), expected_axes):
                for value, magnitude in ((1., 1.), (-1., -1.), (.6, .5), (-.6, -.5)):
                    with self.subTest(mode=mode, axis=axis, value=value):
                        actual = joystick_axes(frame(**{axis: value}), mode=mode)
                        for received, wanted in zip(actual, expected):
                            self.assertAlmostEqual(received, wanted*magnitude)

    def test_diagonal_is_radially_bounded_and_mixing_can_be_overridden(self):
        for mode, (up, right, turn) in self.DIRECTIONS.items():
            for x, y in ((1., 1.), (-1., 1.), (1., -1.), (-1., -1.)):
                with self.subTest(mode=mode, x=x, y=y):
                    forward, lateral, yaw = joystick_axes(frame(x, y, 1.), mode=mode)
                    self.assertAlmostEqual(math.hypot(forward, lateral), 1.)
                    self.assertAlmostEqual(forward, up[0]*y/math.sqrt(2.))
                    self.assertAlmostEqual(lateral, right[1]*x/math.sqrt(2.))
                    self.assertEqual(yaw, turn[2])
            mixed = joystick_axes(frame(.6, .8, .7), mode=mode)
            for actual, expected in zip(mixed, (up[0]*.8, right[1]*.6, turn[2]*.625)):
                self.assertAlmostEqual(actual, expected)
            self.assertEqual(joystick_axes(frame(1., 1., 1.), rotation_priority=True, mode=mode),
                             (0., 0., turn[2]))
            self.assertEqual(joystick_axes(frame(.1, .1, .1), mode=mode), (0., 0., 0.))

    def test_right_vertical_axis_does_not_change_chassis_command(self):
        for mode in self.DIRECTIONS:
            with self.subTest(mode=mode):
                self.assertEqual(joystick_axes(frame(yaw=.8, lift=-1.), mode=mode),
                                 joystick_axes(frame(yaw=.8, lift=1.), mode=mode))
                self.assertEqual(joystick_axes(frame(lift=1.), mode=mode), (0., 0., 0.))

    def test_unknown_mode_is_rejected_before_a_session_is_created(self):
        for mode in ('unknown', None):
            with self.subTest(mode=mode):
                with self.assertRaisesRegex(ValueError, '未知底盘遥操方向模式'):
                    joystick_axes(frame(), mode=mode)
                created = []
                def factory(*args, **kwargs):
                    created.append((args, kwargs))
                    return FakeSession(*args, **kwargs)
                with self.assertRaisesRegex(ValueError, '未知底盘遥操方向模式'):
                    ChassisTeleop(NS(user='unused', mode=mode), None, factory)
                self.assertEqual(created, [])

    def test_adapter_mode_snapshot_and_fixed_physical_sticks_ignore_arm_hand_mapping(self):
        for mode, (up, right, turn) in self.DIRECTIONS.items():
            for hand_mapping in ('auto', 'same', 'crossed'):
                with self.subTest(mode=mode, hand_mapping=hand_mapping):
                    args = NS(user='unused', mode=mode, hand_mapping=hand_mapping)
                    c = ChassisTeleop(args, None, FakeSession).start()
                    c.update(frame(x=.6, y=.8, yaw=-.7, lift=1., stamp=123.), True)
                    values, source = c.session.inputs[-1][0]
                    for actual, expected in zip(values, (up[0]*.8, right[1]*.6, -turn[2]*.625)):
                        self.assertAlmostEqual(actual, expected)
                    self.assertEqual(source, 123.)
                    self.assertEqual(c.snapshot()['mode'], mode)
                    self.assertTrue(c.finish())

    def test_invalid_or_pressed_joystick_is_not_motion(self):
        for bad in (None, float('nan'), True, 1.1):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                joystick_axes(frame(x=bad))
        missing = frame(); missing['right']['joystick_available'] = False
        pressed = frame(); pressed['left']['axis_click'] = True
        for value in (None, missing, pressed):
            with self.assertRaises(ValueError):
                joystick_axes(value)

    def test_adapter_forwards_original_sample_time_and_pauses_without_io(self):
        c = ChassisTeleop(NS(user='fairino', chassis_host='base.example'), None, FakeSession).start()
        self.assertEqual(c.session.arguments[0][0], 'base.example')
        c.update(frame(y=1., stamp=123.), True)
        self.assertEqual(c.session.inputs[-1], (((1., 0., 0.), 123.), {'active': True}))
        c.update(frame(y=1., stamp=123.), True)
        self.assertEqual(c.session.inputs[-1][0][1], 123.)
        c.update(frame(y=1.), False)
        self.assertEqual(c.session.inputs[-1], ((), {'active': False}))
        self.assertEqual(c.snapshot()['enabled'], 8)
        c.session.state['error'] = '底盘故障'
        with self.assertRaisesRegex(RuntimeError, '底盘控制：底盘故障'):
            c.update(frame(), True)
        self.assertTrue(c.finish())


class StationaryFeedbackTests(unittest.TestCase):
    def adapter(self):
        c = ChassisTeleop(NS(user='unused', mode='mirror_same_direction'), None, FakeSession)
        c.session.state = dict(ready=True, received=10., error=None,
                               hardware={'velocity': [0.]*8}, command=[0.]*3)
        return c

    def test_fresh_stationary_feedback_accepts_boundary_age_and_preserves_state(self):
        c = self.adapter()
        c.session.state['hardware']['velocity'] = [.049, -.049]*4
        c.session.state['command'] = [.001, -.001, 0.]
        before = copy.deepcopy(c.session.state)
        c.require_stationary(10.25)
        self.assertEqual(c.session.state, before)
        self.assertEqual(c.session.inputs, [])
        self.assertEqual(c.mode, 'mirror_same_direction')

    def test_missing_stale_future_or_nonfinite_received_timestamp_is_not_stationary(self):
        for received in (None, 9.749, 10.001, float('nan'), float('inf'), True, '10'):
            with self.subTest(received=received):
                c = self.adapter()
                c.session.state['received'] = received
                with self.assertRaisesRegex(ValueError, '底盘反馈'):
                    c.require_stationary(10.)
                self.assertFalse(c.session.inputs)
        for now in (None, True, float('nan'), float('inf')):
            with self.subTest(now=now), self.assertRaisesRegex(ValueError, '底盘反馈'):
                self.adapter().require_stationary(now)

    def test_not_ready_or_session_error_rejects_calibration_without_commands(self):
        for key, value in (('ready', False), ('ready', None), ('error', '反馈已断开')):
            with self.subTest(key=key):
                c = self.adapter()
                c.session.state[key] = value
                with self.assertRaisesRegex(ValueError, '底盘'):
                    c.require_stationary(10.)
                self.assertFalse(c.session.inputs)

    def test_all_eight_wheel_axes_must_be_finite_and_below_stop_threshold(self):
        for index in range(8):
            for velocity in (.05, -.05, float('nan'), float('inf'), None, True):
                with self.subTest(index=index, velocity=velocity):
                    c = self.adapter()
                    c.session.state['hardware']['velocity'][index] = velocity
                    with self.assertRaisesRegex(ValueError, '底盘'):
                        c.require_stationary(10.)
                    self.assertFalse(c.session.inputs)
        for hardware in (None, {}, {'velocity': []}, {'velocity': [0.]*7}, {'velocity': [0.]*9}):
            with self.subTest(hardware=hardware):
                c = self.adapter()
                c.session.state['hardware'] = hardware
                with self.assertRaisesRegex(ValueError, '速度反馈'):
                    c.require_stationary(10.)

    def test_command_must_be_explicitly_finite_and_near_zero(self):
        for command in (None, [], [0., 0.], [0.]*4, [.0011, 0., 0.], [0., -.0011, 0.],
                        [0., 0., .0011], [float('nan'), 0., 0.], [0., None, 0.], [0., 0., True]):
            with self.subTest(command=command):
                c = self.adapter()
                c.session.state['command'] = command
                with self.assertRaisesRegex(ValueError, '底盘'):
                    c.require_stationary(10.)
                self.assertFalse(c.session.inputs)


if __name__ == '__main__':
    unittest.main()
