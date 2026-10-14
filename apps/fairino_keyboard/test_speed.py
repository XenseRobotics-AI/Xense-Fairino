"""Adjustable-speed protocol, combined-wheel envelope and offline GUI checks."""
import copy
import itertools
import json
import math
import queue
import threading
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

import control_core as core
import keyboard_control as desktop
import robot_worker as robot
from test_control import hardware
import test_startup as startup

CONFIG = startup.CONFIG

try:
    import gi
    gi.require_version("Gtk", "3.0")
    from gi.repository import Gtk, Gdk
    GTK = Gtk.init_check()[0]
except (ImportError, ValueError):
    GTK = False


class SpeedProtocolTests(unittest.TestCase):
    def worker(self):
        w = robot.Worker.__new__(robot.Worker)
        w.args = NS(linear=core.DEFAULT_LINEAR, angular=core.DEFAULT_ANGULAR)
        w.hw = core.normalise_hardware(hardware())
        w.hw_time = w.upper_time = w.received = time.monotonic()
        w.upper = 0
        w.stop = threading.Event()
        w.input_lock = threading.Lock()
        w.input_error = ""
        w.homing = w.ready = False
        w.packet = dict(keys=[], ack=w.received, focused=True, stop=False,
                        linear=.20, angular=.40)
        return w

    def test_speed_packet_reaches_receiver_then_scales_real_command(self):
        c = desktop.Connection.__new__(desktop.Connection)
        c.done = threading.Event()
        c.last_status = 0
        c.write_lock = threading.Lock()
        c.speeds = (.06, .10)
        c.latest = {"stamp": time.monotonic()}
        sent = []
        c.child = NS(sendline=lambda value: sent.append(json.loads(value)))
        c.send(speeds=(.20, .40))
        w = self.worker()
        w.packet = sent[-1]
        self.assertEqual(w.guard(), [])
        c.send(keys="wdj", speeds=(.20, .40))
        w.packet = sent[-1]
        vx, vy, wz = core.key_twist(w.guard(), w.args.linear, w.args.angular)
        self.assertAlmostEqual(math.hypot(vx, vy), .20)
        self.assertEqual(wz, .40)
        self.assertLess(vy, 0)
        c.send(speeds=(.20, .40))
        w.packet = sent[-1]
        self.assertEqual(core.key_twist(w.guard(), w.args.linear, w.args.angular), (0, 0, 0))

    def test_bad_packets_are_rejected_on_robot(self):
        for bad in (float("nan"), float("inf"), 0, -.1, .21, "0.1", True):
            w = self.worker()
            w.packet["linear"] = bad
            with self.subTest(bad=bad), self.assertRaises(RuntimeError):
                w.guard()
        w = self.worker()
        w.packet["angular"] = .41
        with self.assertRaises(RuntimeError):
            w.guard()

    def test_new_speed_cannot_be_combined_with_held_motion_key(self):
        w = self.worker()
        w.packet["keys"] = ["w"]
        with self.assertRaisesRegex(RuntimeError, "松开"):
            w.guard()
        self.assertEqual(w.args.linear, core.DEFAULT_LINEAR)

    def test_high_speed_does_not_bypass_expired_heartbeat(self):
        w = self.worker()
        w.packet["ack"] -= 1
        with self.assertRaisesRegex(RuntimeError, "超时"):
            w.guard()

    def test_all_key_combinations_fit_wheel_and_feedback_limits(self):
        params = CONFIG["eight_dof_chassis_controller"]["ros__parameters"]
        core.check_speed_envelope(params)
        for bits in itertools.product((False, True), repeat=6):
            keys = [k for k, pressed in zip("wasdjl", bits) if pressed]
            vx, vy, wz = core.key_twist(keys, core.MAX_LINEAR, core.MAX_ANGULAR)
            for module in params["modules"].values():
                x, y, _ = module["base_to_steering"]["xyz"]
                wheel = math.hypot(vx-wz*y, vy+wz*x)/module["wheel_radius"]
                self.assertLess(wheel, core.DRIVE_COMMAND_LIMIT)
        self.assertLess(core.DRIVE_COMMAND_LIMIT, core.DRIVE_FEEDBACK_LIMIT)
        invalid = copy.deepcopy(params)
        invalid["modules"]["front_left"]["wheel_radius"] = .02
        with self.assertRaisesRegex(RuntimeError, "超过轮组"):
            core.check_speed_envelope(invalid)

    def test_controller_receives_new_limits_but_homing_is_unchanged(self):
        w = startup.StartupTests().exercise()
        self.assertIsNone(w.failure)
        p = w.loaded_params[robot.CHASSIS]["limits"]
        self.assertEqual(p["base"]["linear"]["max_velocity"], .20)
        self.assertEqual(p["base"]["angular"]["max_velocity"], .40)
        self.assertEqual(p["drive"]["max_velocity"], core.DRIVE_COMMAND_LIMIT)
        self.assertEqual(p["steering"]["max_velocity"], .35)
        self.assertEqual(w.commands[-3:], ["enable(85)", "disable(85)", "enable()"])


@unittest.skipUnless(GTK, "GTK/display unavailable")
class SpeedWindowTests(unittest.TestCase):
    def setUp(self):
        class Hidden(Gtk.OffscreenWindow):
            def present(self):
                pass
            def is_active(self):
                return True
        self.args = NS(demo=True, linear=.06, angular=.10)
        with patch.object(Gtk, "Window", Hidden):
            self.w = desktop.Window(self.args)

    def tearDown(self):
        self.w.closing = True
        self.w.window.destroy()

    def event(self, key):
        return NS(keyval=Gdk.keyval_from_name(key))

    def test_adjust_stop_and_entry_typing(self):
        w = self.w
        w.speed_adjustments[0].set_value(.20)
        w.speed_adjustments[1].set_value(.40)
        self.assertEqual((w.args.linear, w.args.angular), (.20, .40))
        w.resume_keyboard()
        w.key_press(None, self.event("w"))
        w.tick()
        self.assertIn("+0.200", w.motion.get_text())
        self.assertTrue(all(not s.get_sensitive() for s in w.speed_widgets))
        w.speed_adjustments[0].set_value(.10)
        self.assertEqual(w.args.linear, .20)
        self.assertEqual(w.speed_adjustments[0].get_value(), .20)
        w.key_release(None, self.event("w"))
        w.tick()
        self.assertFalse(w.keys)
        self.assertTrue(all(s.get_sensitive() for s in w.speed_widgets))
        w.speed_spins[0].grab_focus()
        w.key_press(None, self.event("d"))
        self.assertFalse(w.keys)
        self.assertFalse(w.held)
        w.key_release(None, self.event("d"))
        w.key_press(None, self.event("Escape"))
        self.assertTrue(w.stopped)

    def test_waits_for_robot_speed_ack_and_requires_fresh_press(self):
        w = self.w
        sent = []
        w.connection = NS(send=lambda keys, **kw: sent.append((tuple(keys), kw)),
                          events=queue.Queue(), latest={})
        w.speed_adjustments[0].set_value(.12)
        self.assertTrue(w.speed_pending)
        w.key_press(None, self.event("w"))
        self.assertFalse(w.keys)
        w.connection.events.put(dict(phase="就绪", ready=True, linear_speed=.12, angular_speed=.10))
        w.tick()
        self.assertFalse(w.speed_pending)
        w.key_press(None, self.event("w"))  # Repeat of an already-held key.
        self.assertFalse(w.keys)
        w.key_release(None, self.event("w"))
        w.key_press(None, self.event("w"))
        self.assertEqual(w.keys, {"w"})
        self.assertEqual(sent[-1][1]["speeds"], (.12, .10))
        w.had_focus = True
        w.focus_out()
        self.assertTrue(sent[-1][1]["stop"])


if __name__ == "__main__":
    unittest.main()
