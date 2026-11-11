"""Actual TRON2 targets through the real receiver with offline ROS doubles."""
import copy
import math
import unittest

import paths
from tcp_core import ARMS
from test_tcp import hardware
import test_vr_native_receiver as native_tests
from vendor_kinematics import VendorKinematics
from vr_core import validate
from wrist_guard import WristGuard, WristBlocked, policy


class TronReceiverTests(unittest.TestCase):
    run_receiver = native_tests.NativeReceiverTests.run_receiver
    translated = native_tests.NativeReceiverTests.translated
    references = staticmethod(native_tests.NativeReceiverTests.references)

    @classmethod
    def setUpClass(cls):
        from tools.ik_experiments.tron2_art7 import dependencies_available
        if not dependencies_available():
            raise unittest.SkipTest('optional TRON2 native dependencies not installed')
        from vr_tron_ik import TronIK
        cls.kin = VendorKinematics()
        cls.native = TronIK(cls.kin.tools)

    def exact(self, hw, left, sequence, right=None):
        sender = copy.deepcopy(hw)
        sender['position'][:7] = sender['command_position'][:7] = left[:]
        goals = {'left': self.kin.forward_tcp(left, 'left')[0]}
        if right is not None:
            sender['position'][7:14] = sender['command_position'][7:14] = right[:]
            goals['right'] = self.kin.forward_tcp(right, 'right')[0]
        return self.native.request(self.kin, sender, goals, sequence, 10.)

    def test_actual_tron_packets_publish_and_released_arm_holds(self):
        native_tests.NativeReceiverTests.test_native_packets_publish_one_point_and_released_arm_holds_last_command(self)

    def test_guard_disabled_rejects_tron_and_native_before_publish(self):
        hw = hardware()
        packet = self.translated(hw, .4, 1)
        for backend in ('tron2', 'native'):
            with self.subTest(backend=backend):
                result = self.run_receiver(hw, [({**packet, 'ik_backend': backend}, None)], guard_enabled=False)
                self.assertEqual(result.probe.accepted_rate.total, 0)
                self.assertEqual(result.messages, [])
                self.assertTrue(any('要求腕部联合限位保护开启' in e.get('jog_rejected', '')
                                    for e in result.probe.events))

    def test_duplicate_id_cannot_skip_active_guard_check(self):
        hw = hardware()
        packet = self.translated(hw, .4, 1)
        def disable(p, step, clock):
            if step == 2:
                # Even an inconsistent internal guard state must fail closed
                # before reusing the previously accepted request ID.
                p.wrist.guard.enabled = False
        result = self.run_receiver(hw, [(packet, None), (packet, None)], mutate=disable)
        self.assertEqual(result.probe.accepted_rate.total, 1)
        self.assertEqual(len(result.messages), 2)  # Target followed by hold only.
        self.assertTrue(any('要求腕部联合限位保护开启' in e.get('jog_rejected', '')
                            for e in result.probe.events))
        self.assertIsNone(result.probe.path)

    def test_backend_change_cannot_reuse_an_accepted_id(self):
        hw = hardware()
        packet = self.translated(hw, .4, 1)
        result = self.run_receiver(hw, [({**packet, 'ik_backend': 'vendor'}, None), (packet, None)])
        self.assertEqual(result.probe.accepted_rate.total, 1)
        self.assertTrue(any('已丢弃旧双臂请求' in e.get('jog_rejected', '')
                            for e in result.probe.events))
        self.assertIsNone(result.probe.path)

    def test_guard_policy_mismatch_discards_target_and_holds(self):
        hw = hardware()
        first = self.translated(hw, .4, 1)
        second = self.translated(hw, .8, 2)
        def mismatch(p, step, clock):
            if step == 2:
                p.packet['request']['_wrist_guard'] = policy(False, 2)
        result = self.run_receiver(hw, [(first, None), (second, None)], mutate=mismatch)
        self.assertEqual(result.probe.accepted_rate.total, 1)
        self.assertEqual(len(result.messages), 2)
        self.assertIn('切换前的旧运动目标', result.probe.wrist.error)
        self.assertIsNone(result.probe.path)

    def test_off_policy_cannot_resume_tron_after_idle_switch(self):
        hw = hardware()
        first = self.translated(hw, .4, 1)
        second = self.translated(hw, .8, 2)
        def switch(p, step, clock):
            if step >= 2:
                p.packet['wrist_guard'] = policy(False, 2)
            if step == 4:
                p.packet['request']['_wrist_guard'] = policy(False, 2)
        # First idle tick stops motion; second confirms the guard transition.
        result = self.run_receiver(hw, [(first, None), (None, None), (None, None), (second, None)], mutate=switch)
        self.assertFalse(result.probe.wrist.guard.enabled)
        self.assertEqual(result.probe.accepted_rate.total, 1)
        self.assertEqual(len(result.messages), 2)
        self.assertTrue(any('要求腕部联合限位保护开启' in e.get('jog_rejected', '')
                            for e in result.probe.events))

    def test_repeated_ids_cannot_refresh_stale_lease(self):
        hw = hardware()
        packet = self.translated(hw, .4, 1)
        steps = [(packet, None)]
        # Every incoming copy lies about a fresh timestamp; an accepted ID
        # retains its original timestamp and must expire anyway.
        steps += [({**packet, 'created': 10.+.005*i}, None) for i in range(1, 60)]
        result = self.run_receiver(hw, steps)
        self.assertEqual(result.probe.accepted_rate.total, 1)
        self.assertTrue(any('0.25 秒' in e.get('jog_rejected', '') for e in result.probe.events))
        self.assertEqual(len(result.messages), 2)
        self.assertIsNone(result.probe.path)

    def test_skipped_packet_checks_published_and_measured_positions(self):
        native_tests.NativeReceiverTests.test_skipped_middle_packet_checks_published_command_and_measured_not_pc_reference(self)

    def test_unsafe_shortcut_rejects_both_arms_atomically(self):
        hw = hardware()
        a, b, c = [hw['position'][:7] for _ in range(3)]
        a[5:7], b[5:7], c[5:7] = (list(map(math.radians, pair)) for pair in ((0., 70.), (0., 0.), (60., 30.)))
        for name in ('position', 'command_position'):
            hw[name][:7] = a[:]
        first = self.exact(hw, a, 1)
        right = hw['position'][7:14]
        right[0] += .01
        last = self.exact(hw, c, 3, right)
        validate(hw, last, 10.)
        WristGuard().arm_segment(b, last['arms']['left']['target'])
        with self.assertRaises(WristBlocked):
            WristGuard().arm_segment(a, last['arms']['left']['target'])
        result = self.run_receiver(hw, [(first, None), (last, None)])
        self.assertEqual(result.probe.accepted_rate.total, 1)
        self.assertTrue(any('联合限位' in e.get('jog_rejected', '') for e in result.probe.events))
        for message in result.messages:
            self.assertEqual(message.points[0].positions[7:14], hw['command_position'][7:14])

    def test_safe_command_path_cannot_hide_unsafe_measured_path(self):
        hw = hardware()
        a, b, c = [hw['position'][:7] for _ in range(3)]
        a[5:7], b[5:7], c[5:7] = (list(map(math.radians, pair)) for pair in ((0., 70.), (0., 0.), (60., 30.)))
        middle, last = self.exact(hw, b, 2), self.exact(hw, c, 3)
        result = self.run_receiver(hw, [(middle, None), (last, a)])
        self.assertEqual(result.probe.accepted_rate.total, 1)
        checks = [row for row in result.checks if row['id'] == 3]
        self.assertEqual(len(checks), 2)
        self.assertEqual(checks[1]['start'], a)
        self.assertIsNone(result.probe.path)

    def test_actual_velocity_reserve_blocks_geometrically_safe_target(self):
        hw = hardware()
        q = hw['position'][:7]
        q[5:7] = list(map(math.radians, (60., 30.)))
        for name in ('position', 'command_position'):
            hw[name][:7] = q[:]
        first, next_packet = self.exact(hw, q, 1), self.exact(hw, q, 2)
        WristGuard().arm_segment(q, q, [0.]*7)
        def moving(p, step, clock):
            if step == 2:
                p.hw['velocity'][5] = math.radians(45.)
        result = self.run_receiver(hw, [(first, None), (next_packet, None)], mutate=moving)
        self.assertEqual(result.probe.accepted_rate.total, 1)
        self.assertTrue(any('联合限位' in e.get('jog_rejected', '') for e in result.probe.events))
        checks = [row for row in result.checks if row['id'] == 2]
        self.assertAlmostEqual(checks[0]['velocity'][5], math.radians(45.))


if __name__ == '__main__':
    unittest.main()
