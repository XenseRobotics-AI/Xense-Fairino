"""Navigation owns a separate velocity source; PICO keeps continuous axes."""
import threading
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

import control_core as core
import robot_worker as rw
from test_control import hardware


class NavigationInputTests(unittest.TestCase):
    def worker(self, navigation=True):
        w = rw.Worker.__new__(rw.Worker)
        w.args = NS(navigation=navigation, linear=core.DEFAULT_LINEAR,
                    angular=core.DEFAULT_ANGULAR)
        w.hw = core.normalise_hardware(hardware())
        w.hw_time = w.upper_time = w.received = 10.
        w.upper = 0
        w.stop = threading.Event()
        w.input_lock = threading.Lock()
        w.input_error = ''
        w.homing = w.ready = False
        w.packet = dict(keys=[], ack=10., focused=True, stop=False,
                        linear=core.DEFAULT_LINEAR, angular=core.DEFAULT_ANGULAR)
        return w

    def guard(self, worker):
        with patch('robot_worker.time.monotonic', return_value=10.):
            return worker.guard()

    def test_navigation_accepts_neutral_heartbeat_with_default_limits(self):
        self.assertEqual(self.guard(self.worker()), [])

    def test_navigation_rejects_raw_keyboard_input_even_when_not_focused(self):
        for focused in (True, False):
            with self.subTest(focused=focused):
                w = self.worker()
                w.packet.update(keys=['w'], focused=focused)
                with self.assertRaisesRegex(RuntimeError, '导航'):
                    self.guard(w)

    def test_navigation_rejects_any_axes_packet_including_neutral_or_unfocused(self):
        for axes in ([0., 0., 0.], [.5, 0., 0.]):
            for focused in (True, False):
                with self.subTest(axes=axes, focused=focused):
                    w = self.worker()
                    w.packet.update(axes=axes, focused=focused)
                    with self.assertRaisesRegex(RuntimeError, '导航'):
                        self.guard(w)

    def test_navigation_limits_cannot_be_changed_by_client_or_startup_arguments(self):
        for field, value in (('linear', .07), ('angular', .11)):
            for change_startup in (True, False):
                with self.subTest(field=field, change_startup=change_startup):
                    w = self.worker()
                    w.packet[field] = value
                    if change_startup:
                        setattr(w.args, field, value)
                    with self.assertRaisesRegex(RuntimeError, '导航'):
                        self.guard(w)

    def test_pico_axes_still_reach_scaled_motion_in_non_navigation_mode(self):
        w = self.worker(navigation=False)
        w.packet['axes'] = [.6, -.8, .5]
        value = self.guard(w)
        self.assertIsInstance(value, core.AxisInput)
        vx, vy, wz = core.input_twist(value, w.args.linear, w.args.angular)
        self.assertAlmostEqual(vx, .036)
        self.assertAlmostEqual(vy, -.048)
        self.assertAlmostEqual(wz, .05)
        w.packet['focused'] = False
        self.assertFalse(self.guard(w))
        self.assertEqual(core.input_twist(self.guard(w)), (0., 0., 0.))

    def test_navigation_mode_preserves_heartbeat_and_robot_feedback_checks(self):
        for failure in ('ack', 'input', 'hardware', 'upper', 'stop'):
            with self.subTest(failure=failure):
                w = self.worker()
                if failure == 'ack':
                    w.packet['ack'] -= 1.
                elif failure == 'input':
                    w.received -= 1.
                elif failure == 'hardware':
                    w.hw_time -= 1.
                elif failure == 'upper':
                    w.upper = 1
                else:
                    w.stop.set()
                with self.assertRaises(RuntimeError):
                    self.guard(w)


if __name__ == '__main__':
    unittest.main()
