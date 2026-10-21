"""Joystick geometry and integration use no robot or network."""
import math
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
    def test_translation_and_yaw_signs_and_analog_scale(self):
        self.assertEqual(joystick_axes(frame(y=1.)), (1., 0., 0.))
        self.assertEqual(joystick_axes(frame(x=1.)), (0., -1., 0.))
        self.assertEqual(joystick_axes(frame(y=-1.)), (-1., 0., 0.))
        self.assertEqual(joystick_axes(frame(x=-1.)), (0., 1., 0.))
        self.assertEqual(joystick_axes(frame(yaw=1.)), (0., 0., -1.))
        self.assertAlmostEqual(joystick_axes(frame(y=.6))[0], .5)

    def test_diagonal_is_radially_bounded_and_mixing_can_be_overridden(self):
        forward, lateral, yaw = joystick_axes(frame(1., 1., 1.))
        self.assertAlmostEqual(math.hypot(forward, lateral), 1.)
        self.assertEqual(yaw, -1.)
        self.assertEqual(joystick_axes(frame(1., 1., 1.), rotation_priority=True), (0., 0., -1.))
        self.assertEqual(joystick_axes(frame(.1, .1, .1)), (0., 0., 0.))

    def test_right_vertical_axis_does_not_change_chassis_command(self):
        self.assertEqual(joystick_axes(frame(yaw=.8, lift=-1.)), joystick_axes(frame(yaw=.8, lift=1.)))
        self.assertEqual(joystick_axes(frame(lift=1.)), (0., 0., 0.))

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


if __name__ == '__main__':
    unittest.main()
