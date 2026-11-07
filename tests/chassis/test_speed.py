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

from fairino_slam.chassis import control_core as core
from fairino_slam.chassis import transport as desktop
from fairino_slam.chassis import robot_worker as robot
from fairino_slam.configuration import load_chassis
from .helpers import hardware
from . import test_startup as startup

CONFIG = startup.CONFIG



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
                        linear=core.DEFAULT_LINEAR, angular=core.DEFAULT_ANGULAR)
        return w

    def test_speed_packet_reaches_receiver_then_scales_real_command(self):
        c = desktop.Connection.__new__(desktop.Connection)
        c.navigation = False
        c.done = threading.Event()
        c.last_status = 0
        c.last_send = 0
        c.write_lock = threading.Lock()
        c.speeds = (core.DEFAULT_LINEAR, core.DEFAULT_ANGULAR)
        c.latest = {"stamp": time.monotonic()}
        sent = []
        c.child = NS(sendline=lambda value: sent.append(json.loads(value)))
        c.send(speeds=(core.MAX_LINEAR, core.MAX_ANGULAR))
        w = self.worker()
        w.packet = sent[-1]
        self.assertEqual(w.guard(), [])
        c.send(keys="wdj", speeds=(core.MAX_LINEAR, core.MAX_ANGULAR))
        w.packet = sent[-1]
        vx, vy, wz = core.key_twist(w.guard(), w.args.linear, w.args.angular)
        self.assertAlmostEqual(math.hypot(vx, vy), core.MAX_LINEAR)
        self.assertEqual(wz, core.MAX_ANGULAR)
        self.assertLess(vy, 0)
        c.send(speeds=(core.MAX_LINEAR, core.MAX_ANGULAR))
        w.packet = sent[-1]
        self.assertEqual(core.key_twist(w.guard(), w.args.linear, w.args.angular), (0, 0, 0))

    def test_bad_packets_are_rejected_on_robot(self):
        for bad in (float("nan"), float("inf"), 0, -.1, core.MAX_LINEAR + .01, "0.1", True):
            w = self.worker()
            w.packet["linear"] = bad
            with self.subTest(bad=bad), self.assertRaises(RuntimeError):
                w.guard()
        w = self.worker()
        w.packet["angular"] = core.MAX_ANGULAR + .01
        with self.assertRaises(RuntimeError):
            w.guard()

    def alternate_speeds(self):
        defaults = (core.DEFAULT_LINEAR, core.DEFAULT_ANGULAR)
        for values in ((core.MAX_LINEAR, core.MAX_ANGULAR), (core.MIN_SPEED, core.MIN_SPEED)):
            if values != defaults:
                return values
        return None

    def test_new_speed_cannot_be_combined_with_held_motion_key(self):
        speeds = self.alternate_speeds()
        if speeds is None:
            self.skipTest('Configured speed interval has no adjustable value')
        w = self.worker()
        w.packet.update(keys=["w"], linear=speeds[0], angular=speeds[1])
        with self.assertRaisesRegex(RuntimeError, "松开"):
            w.guard()
        self.assertEqual(w.args.linear, core.DEFAULT_LINEAR)

    def test_high_speed_does_not_bypass_expired_heartbeat(self):
        w = self.worker()
        w.packet.update(linear=core.MAX_LINEAR, angular=core.MAX_ANGULAR)
        w.packet["ack"] -= 1
        with self.assertRaisesRegex(RuntimeError, "超时"):
            w.guard()

    def test_navigation_worker_rejects_manual_keys_and_speed_changes(self):
        cases = [(['w'], (core.DEFAULT_LINEAR, core.DEFAULT_ANGULAR))]
        if self.alternate_speeds() is not None:
            cases.append(([], self.alternate_speeds()))
        for keys, speeds in cases:
            with self.subTest(keys=keys, speeds=speeds):
                w = self.worker()
                w.args.navigation = True
                w.packet.update(keys=keys, linear=speeds[0], angular=speeds[1])
                with self.assertRaisesRegex(RuntimeError, '导航'):
                    w.guard()

    def test_navigation_worker_accepts_only_default_heartbeat(self):
        w = self.worker()
        w.args.navigation = True
        w.packet.update(keys=[], linear=core.DEFAULT_LINEAR, angular=core.DEFAULT_ANGULAR)
        self.assertEqual(w.guard(), [])

    def test_all_key_combinations_fit_wheel_and_feedback_limits(self):
        params = CONFIG["eight_dof_chassis_controller"]["ros__parameters"]
        core.check_speed_envelope(params)
        for bits in itertools.product((False, True), repeat=6):
            keys = [k for k, pressed in zip("wasdjl", bits) if pressed]
            vx, vy, wz = core.key_twist(keys, core.MAX_LINEAR, core.MAX_ANGULAR)
            for module in params["modules"].values():
                x, y, _ = module["base_to_steering"]["xyz"]
                wheel = math.hypot(vx-wz*y, vy+wz*x)/module["wheel_radius"]
                self.assertLessEqual(wheel, core.DRIVE_COMMAND_LIMIT + 1e-12)
        self.assertLessEqual(core.DRIVE_COMMAND_LIMIT, core.DRIVE_FEEDBACK_LIMIT)
        invalid = copy.deepcopy(params)
        invalid["modules"]["front_left"]["wheel_radius"] = .5 * core.MAX_LINEAR / core.DRIVE_COMMAND_LIMIT
        with self.assertRaisesRegex(RuntimeError, "超过轮组"):
            core.check_speed_envelope(invalid)

    def test_controller_receives_new_limits_but_homing_is_unchanged(self):
        w = startup.StartupTests().exercise()
        self.assertIsNone(w.failure)
        p = w.loaded_params[robot.CHASSIS]["limits"]
        self.assertEqual(p["base"]["linear"]["max_velocity"], core.MAX_LINEAR)
        self.assertEqual(p["base"]["angular"]["max_velocity"], core.MAX_ANGULAR)
        self.assertEqual(p["drive"]["max_velocity"], core.DRIVE_COMMAND_LIMIT)
        self.assertEqual(p["steering"]["max_velocity"], load_chassis()["limits"]["steering"]["max_velocity"])
        self.assertEqual(w.commands[-3:], ["enable(85)", "disable(85)", "enable()"])


if __name__ == "__main__":
    unittest.main()
