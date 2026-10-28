"""Real receiver loop with native packets and in-memory ROS/feedback doubles."""
import copy
import math
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np
import paths
import robot_probe as rp
from tcp_core import ARMS
from test_probe import STUBS
from test_tcp import hardware, probe
from vendor_kinematics import VendorKinematics
from vr_core import validate
from vr_native_ik import NativeIK
from vr_robot import VRProbe
from wrist_guard import WristGuard, WristBlocked


class EndSimulation(Exception):
    pass


class NativeReceiverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kin = VendorKinematics()
        cls.native = NativeIK()

    def translated(self, hw, distance, sequence, sides=('left', 'right'), references=None):
        goals = {}
        for side in sides:
            q = [hw['command_position'][j-1] for j in ARMS[side]]
            goal = np.asarray(self.kin.forward_tcp(q, side)[0]).copy()
            goal[:3, 3] += [distance, -.5*distance, .25*distance]
            goals[side] = goal
        return self.native.request(self.kin, hw, goals, sequence, 10., references)

    def run_receiver(self, initial, steps):
        """Each step supplies a latest packet, optionally lagging measured q.

        The real run, validation, wrist checks, publish_positions, stop_stream
        and confirmation paths execute. Only ROS I/O and time are substituted.
        """
        p = probe()
        p.__class__ = VRProbe
        p.hw = copy.deepcopy(initial)
        p.baseline = p.hw['position'][:]
        p.hold_positions = p.hw['command_position'][:]
        p.tools = self.native.tools
        p.args.command_hz, p.args.receiver_hz, p.args.feedback_hz = 100., 200., 100.
        clock, cursor, messages, checks = [10.], [0], [], []
        p.node = NS(create_publisher=lambda *args: NS(
            get_subscription_count=lambda: 1, publish=lambda message: messages.append(copy.deepcopy(message))))

        def feedback(seconds=.005, guarded=True):
            clock[0] += seconds
            if messages:
                # A deliberately ideal follower is sufficient to confirm hold;
                # individual test steps may override measured state afterwards.
                q = messages[-1].points[0].positions[:]
                p.hw['position'], p.hw['command_position'] = q[:], q[:]
                p.hw['velocity'] = [0.]*20
            p.hw_time = p.received = p.packet['ack'] = clock[0]
            if guarded:
                p.guard()

        def prepare():
            p.hw_time = p.received = p.packet['ack'] = clock[0]
            rp.Probe.enable_all(p)
            p.ready = True

        def next_input(seconds):
            feedback(seconds)
            if cursor[0] == len(steps):
                raise EndSimulation()
            request, measured = steps[cursor[0]]
            cursor[0] += 1
            request = copy.deepcopy(request)
            if request is not None:
                request['_wrist_guard'] = p.wrist.active.copy()
            p.packet['request'] = request
            if measured is not None:
                p.hw['position'][:7] = measured[:]
            p.guard()

        original_check = p.wrist.guard.arm_segment

        def record_check(start, target, velocity=None, label='腕部'):
            checks.append(dict(id=(p.packet.get('request') or {}).get('id'), start=start[:],
                               target=target[:], velocity=velocity[:] if velocity else velocity,
                               label=label))
            return original_check(start, target, velocity, label)

        p.prepare, p.pump, p.pump_input = prepare, feedback, next_input
        stubs = {**STUBS, 'JointTrajectory': lambda: NS(joint_names=[], points=[], header=NS(stamp=None))}
        with patch.dict(rp.__dict__, stubs), patch('vr_robot.time.monotonic', lambda: clock[0]), \
                patch.object(p.wrist.guard, 'arm_segment', side_effect=record_check):
            with self.assertRaises(EndSimulation):
                p.run()
        for message in messages:
            self.assertEqual(message.joint_names, rp.JOINTS)
            self.assertEqual(len(message.points), 1)
            point = message.points[0]
            self.assertEqual(point.time_from_start, 0.)
            self.assertEqual(point.velocities, [])
            self.assertEqual(point.accelerations, [])
            self.assertEqual(point.positions[14:], initial['command_position'][14:])
        return NS(probe=p, messages=messages, checks=checks)

    @staticmethod
    def references(packet):
        return {side: value['target'][:] for side, value in packet['arms'].items()}

    def test_native_packets_publish_one_point_and_released_arm_holds_last_command(self):
        hw = hardware()
        both = self.translated(hw, .4, 1)
        right = self.translated(hw, .8, 2, sides=('right',),
                                references={'right': both['arms']['right']['target']})
        result = self.run_receiver(hw, [(both, None), (right, None), (None, None)])
        self.assertEqual(result.probe.accepted_rate.total, 2)
        self.assertEqual(len(result.messages), 3)  # Two targets and the exact hold.
        first, second, hold = [message.points[0].positions for message in result.messages]
        self.assertEqual(first[:7], both['arms']['left']['target'])
        self.assertEqual(second[:7], first[:7])
        self.assertEqual(second[7:14], right['arms']['right']['target'])
        self.assertNotEqual(second[7:14], first[7:14])
        self.assertEqual(hold, second)
        self.assertIsNone(result.probe.path)
        self.assertTrue(result.probe.ready)
        self.assertTrue(result.probe.session_enabled)
        self.assertFalse(any('jog_rejected' in event for event in result.probe.events))

    def test_skipped_middle_packet_checks_published_command_and_measured_not_pc_reference(self):
        hw = hardware()
        first = self.translated(hw, .3, 1, sides=('left',))
        omitted = self.translated(hw, .6, 2, sides=('left',), references=self.references(first))
        latest = self.translated(hw, .9, 3, sides=('left',), references=self.references(omitted))
        published = first['arms']['left']['target']
        planned = omitted['arms']['left']['target']
        measured = hw['position'][:7]
        self.assertNotEqual(published, planned)
        self.assertNotEqual(published, measured)
        result = self.run_receiver(hw, [(first, None), (latest, measured)])
        checks = [row for row in result.checks if row['id'] == 3]
        self.assertEqual(len(checks), 2)
        self.assertEqual(checks[0]['start'], published)
        self.assertEqual(checks[1]['start'], measured)
        self.assertNotEqual(checks[0]['start'], latest['arms']['left']['reference'])
        self.assertNotEqual(checks[1]['start'], latest['arms']['left']['reference'])
        self.assertEqual([row['target'] for row in checks], [latest['arms']['left']['target']]*2)
        self.assertEqual(result.probe.accepted_rate.total, 2)
        self.assertEqual(result.probe.last_request, 3)
        self.assertEqual(len(result.messages), 2)

    def nonconvex_packets(self):
        """All endpoint packets are real native solutions, with a safe B->C leg.

        A->C cuts across the wrist envelope. The receiver must independently
        reject that leg even when the packet's FK and sender-side B->C pass.
        """
        hw = hardware()
        a, b, c = [hw['position'][:7] for _ in range(3)]
        a[5:7] = list(map(math.radians, (0., 70.)))
        b[5:7] = [0., 0.]
        c[5:7] = list(map(math.radians, (60., 30.)))
        for field in ('position', 'command_position'):
            hw[field][:7] = a[:]
        first = self.native.request(self.kin, hw,
            {'left': self.kin.forward_tcp(a, 'left')[0]}, 1, 10., {'left': a})
        middle = self.native.request(self.kin, hw,
            {'left': self.kin.forward_tcp(b, 'left')[0]}, 2, 10., self.references(first))
        planned_hw = copy.deepcopy(hw)
        for field in ('position', 'command_position'):
            planned_hw[field][:7] = middle['arms']['left']['target'][:]
        last = self.native.request(self.kin, planned_hw,
            {'left': self.kin.forward_tcp(c, 'left')[0]}, 3, 10., self.references(middle))
        validate(hw, last, 10.)  # Protocol/endpoint FK is valid even at receiver A.
        WristGuard().arm_segment(middle['arms']['left']['target'], last['arms']['left']['target'])
        with self.assertRaises(WristBlocked):
            WristGuard().arm_segment(a, last['arms']['left']['target'])
        return hw, first, middle, last

    def test_receiver_rejects_unsafe_shortcut_from_real_last_published_target(self):
        hw, first, omitted, last = self.nonconvex_packets()
        result = self.run_receiver(hw, [(first, None), (last, None)])
        checks = [row for row in result.checks if row['id'] == 3]
        self.assertEqual(len(checks), 1)  # First path failed before actual path or publish.
        self.assertEqual(checks[0]['start'], first['arms']['left']['target'])
        self.assertNotEqual(checks[0]['start'], omitted['arms']['left']['target'])
        self.assertEqual(result.probe.accepted_rate.total, 1)
        self.assertEqual(result.probe.last_request, 1)
        self.assertTrue(any('联合限位' in event.get('jog_rejected', '') for event in result.probe.events))
        self.assertEqual(len(result.messages), 2)  # Initial command and its hold only.
        for message in result.messages:
            self.assertEqual(message.points[0].positions[:7], first['arms']['left']['target'])
        self.assertIsNone(result.probe.path)
        self.assertTrue(result.probe.session_enabled)

    def test_receiver_rejects_unsafe_actual_path_even_when_published_path_is_safe(self):
        hw, first, middle, last = self.nonconvex_packets()
        result = self.run_receiver(hw, [(middle, None), (last, first['arms']['left']['target'])])
        checks = [row for row in result.checks if row['id'] == 3]
        self.assertEqual(len(checks), 2)
        self.assertEqual(checks[0]['start'], middle['arms']['left']['target'])
        self.assertEqual(checks[1]['start'], first['arms']['left']['target'])
        self.assertEqual(result.probe.accepted_rate.total, 1)
        self.assertEqual(result.probe.last_request, 2)
        self.assertTrue(any('联合限位' in event.get('jog_rejected', '') for event in result.probe.events))
        self.assertEqual(len(result.messages), 2)
        for message in result.messages:
            self.assertEqual(message.points[0].positions[:7], middle['arms']['left']['target'])
        self.assertIsNone(result.probe.path)
        self.assertTrue(result.probe.session_enabled)


if __name__ == '__main__':
    unittest.main()
