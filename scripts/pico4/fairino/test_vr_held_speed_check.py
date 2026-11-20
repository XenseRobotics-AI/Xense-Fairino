"""Held-axis speed opt-out only; ROS, networking and robot motion are never used."""
import ast
import json
import math
import shlex
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import paths
import robot_probe as rp
from probe_core import check_hardware
from test_probe import fake_probe, hardware
from vr_client import remote_command
from vr_robot import VRProbe


class HeldSpeedCheckTests(unittest.TestCase):
    @staticmethod
    def enabled_probe(disabled=False):
        p = fake_probe(VRProbe)
        p.held_speed_check = not disabled
        p.enable_attempted = p.session_enabled = True
        p.hw['enabled'] = [True] * 20
        p.moving_joints = list(range(1, 15))
        return p

    def test_omitted_setting_keeps_vr_one_degree_threshold(self):
        p = fake_probe(VRProbe)
        p.moving_joints = list(range(1, 15))
        p.hw['velocity'][15] = math.radians(-1.0693069306930694)
        with self.assertRaisesRegex(RuntimeError, r'J16 速度 .*1.0°/s'):
            p.guard()
        self.assertIn('fault_snapshot', p.events[-1])

    def test_fairino_qp_raises_only_held_j16_to_ten_degrees(self):
        # 2026-11-20 real-robot stops: J16 -1.842 and -1.010 deg/s while its command stayed fixed.
        p = fake_probe(VRProbe)
        p.held_speed_overrides = {16: 10.}
        p.moving_joints = list(range(1, 15))
        for speed in (-1.842, -1.010, 9.99):
            p.hw['velocity'][15] = math.radians(speed)
            p.guard()
        p.hw['velocity'][15] = math.radians(10.01)
        with self.assertRaisesRegex(RuntimeError, r'J16 速度 .*10.0°/s'):
            p.guard()
        p.hw['velocity'][15] = 0.
        for joint in (15, 17, 18, 19, 20):
            p.hw['velocity'][joint-1] = math.radians(1.01)
            with self.assertRaisesRegex(RuntimeError, rf'J{joint} 速度 .*1.0°/s'):
                p.guard()
            p.hw['velocity'][joint-1] = 0.

    def test_held_j16_override_is_fairino_qp_only(self):
        # Constructor only; TCPProbe.__init__ (ROS node) is replaced.
        with patch('vr_robot.TCPProbe.__init__', lambda self, args: None):
            for body, expected in ((None, None), (object(), {16: 10.})):
                p = VRProbe(NS(body=body, tools_json=None, body_lift=False, no_held_speed_check=False))
                self.assertEqual(p.held_speed_overrides, expected)

    def test_disabled_check_accepts_observed_speed_peaks_without_position_drift(self):
        # Magnitudes from the recorded J16/J17 stops, in the hardware's rad/s.
        for moving in (list(range(1, 15)), None):
            for joint, speed in ((16, -.018662926654988872),
                                 (17, .024883902207)):
                with self.subTest(moving=moving, joint=joint):
                    p = self.enabled_probe(disabled=True)
                    p.moving_joints = moving
                    p.hw['velocity'][joint - 1] = speed
                    p.guard()
                    self.assertEqual(p.hw['position'], p.hold_positions)
                    self.assertFalse(p.events)

    def test_disabled_held_check_does_not_disable_active_axis_speed_limit(self):
        hw = hardware()
        hw['velocity'][15] = math.radians(10.)
        options = dict(moving_joints=[1], moving_speed_limit=2.,
                       held_speed_limit=1., held_speed_check=False)
        check_hardware(hw, 0., [0, 0], **options)
        hw['velocity'][0] = math.radians(2.01)
        with self.assertRaisesRegex(RuntimeError, r'J1 速度 .*2.0°/s'):
            check_hardware(hw, 0., [0, 0], **options)

    def test_disabled_speed_check_keeps_point_three_degree_held_position_limit(self):
        for moving, joints in ((list(range(1, 15)), (15, 16, 17, 18, 19, 20)),
                               (None, (1, 7, 8, 14, 16, 17, 20))):
            for joint in joints:
                with self.subTest(moving=moving, joint=joint):
                    p = self.enabled_probe(disabled=True)
                    p.moving_joints = moving
                    p.hw['velocity'][joint - 1] = math.radians(5.)
                    p.hw['position'][joint - 1] += math.radians(.31)
                    with self.assertRaisesRegex(RuntimeError,
                                                f'J{joint} 偏离保持目标超过 0.3°'):
                        p.guard()

    def test_disabled_speed_check_keeps_fault_estop_feedback_and_link_guards(self):
        cases = (
            ('上半身故障', lambda p: p.hw.__setitem__('error', 1)),
            ('J17=33', lambda p: p.hw['errorcode'].__setitem__(16, 33)),
            ('急停', lambda p: setattr(p, 'gpio', lambda: [1, 0])),
            ('急停', lambda p: setattr(p, 'gpio', lambda: None)),
            ('反馈缺失或超时', lambda p: setattr(p, 'hw_time', time.monotonic() - 1.)),
            ('心跳中断', lambda p: setattr(p, 'received', time.monotonic() - 1.)),
            ('退出使能', lambda p: p.hw['enabled'].__setitem__(16, False)),
        )
        for message, mutate in cases:
            with self.subTest(message=message):
                p = self.enabled_probe(disabled=True)
                p.hw['velocity'][15] = math.radians(5.)
                mutate(p)
                with self.assertRaisesRegex(RuntimeError, message):
                    p.guard()

    def test_disabled_check_cannot_take_over_already_enabled_hardware(self):
        p = fake_probe(VRProbe)
        p.held_speed_check = False
        p.hw['enabled'][15] = True
        p.hw['velocity'][15] = math.radians(5.)
        with self.assertRaisesRegex(RuntimeError, '启动前已有电机使能'):
            p.guard()

    def test_vr_constructor_applies_explicit_opt_out_without_changing_threshold(self):
        for options, expected in (({}, True), ({'no_held_speed_check': False}, True),
                                  ({'no_held_speed_check': True}, False)):
            with self.subTest(options=options), patch('vr_robot.TCPProbe.__init__', return_value=None):
                p = VRProbe(NS(**options))
                self.assertIs(p.held_speed_check, expected)
                self.assertEqual(p.held_speed_limit, 1.)
                self.assertTrue(p.feedback_speed_check)

    def test_report_distinguishes_configured_and_temporarily_active_checks(self):
        for enabled, enabling, active in ((True, False, True), (False, False, False),
                                          (True, True, False), (False, True, False)):
            with self.subTest(enabled=enabled, enabling=enabling):
                p = fake_probe(VRProbe)
                p.held_speed_check = enabled
                p.enabling = enabling
                p.last_status = 0.
                VRProbe.report(p, force=True)
                state = p.events[-1]
                self.assertIs(state['held_speed_check_enabled'], enabled)
                self.assertIs(state['held_speed_check_active'], active)
                self.assertEqual(state['held_speed_limit_deg_s'], 1.)
                # Disabling the check must not leak infinity into strict JSON.
                json.dumps(state, allow_nan=False)

    def test_default_shared_probe_still_rejects_half_degree_held_speed(self):
        p = fake_probe()
        p.hw['velocity'][15] = math.radians(.535)
        with self.assertRaisesRegex(RuntimeError, r'J16 速度 .*0.5°/s'):
            rp.Probe.guard(p)

    def test_disabled_held_speed_check_still_requires_actual_stop_confirmation(self):
        from test_vr_stop import StopConfirmationTests
        harness = StopConfirmationTests()
        with harness.simulation(fresh_after=.02, stopped_after=math.inf) as (p, _, *_):
            p.held_speed_check = False
            with self.assertRaisesRegex(RuntimeError, '释放输入后未确认停止'):
                p.stop_stream('输入已释放')
            self.assertFalse(p.ready)
            self.assertEqual(p.events[-1]['stop_confirmation']['result'], 'failed')

    def test_remote_loader_forwards_opt_out_for_all_real_ik_backends(self):
        for backend in ('native', 'vendor', 'tron2'):
            for options, expected in (({}, False), ({'no_held_speed_check': False}, False),
                                      ({'no_held_speed_check': True}, True)):
                with self.subTest(backend=backend, options=options):
                    command = remote_command(NS(ik_backend=backend, execute=True, **options))
                    setup = shlex.split(command)[2]
                    loader = shlex.split(setup.split('exec python3 -u -c ', 1)[1])[0]
                    tree = ast.parse(loader)
                    argv = next(ast.literal_eval(n.value) for n in tree.body
                                if isinstance(n, ast.Assign)
                                and ast.unparse(n.targets[0]) == 'sys.argv')
                    self.assertEqual(argv.count('--no-held-speed-check'), int(expected))


if __name__ == '__main__':
    unittest.main()
