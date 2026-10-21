"""Delayed hardware feedback during release/hold; ROS and hardware are simulated."""
from contextlib import contextmanager
import math
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import paths
import robot_probe as rp
from test_tcp import probe, STUBS
from vr_robot import VRProbe


class StopConfirmationTests(unittest.TestCase):
    @contextmanager
    def simulation(self, joint=4, age=.067, fresh_after=.127, match_after=0.,
                   stopped_after=0., delta=.001, interrupt=None):
        clock = [10.]
        p = probe()
        p.__class__ = VRProbe
        p.hw_time = clock[0]-age
        p.hw['enabled'] = [True]*20
        p.enable_attempted = p.session_enabled = True
        p.path = [(0., p.hw['command_position'][:])]
        p.path[0][1][joint-1] += delta
        target = p.path[0][1][:]
        p.moving_joints = list(range(1, 15))+([18, 19, 20] if joint >= 18 else [])
        p.lift = NS(reset=lambda: None) if joint >= 18 else None
        p.lift_axis_active = joint >= 18
        p.lift_stop_deadline = None
        p.report = lambda *_, **__: None
        messages = []
        p.publisher = NS(publish=messages.append)

        def pump(seconds=.02, guarded=True):
            clock[0] += seconds
            elapsed = clock[0]-10.
            p.received = clock[0]
            p.packet['ack'] = clock[0]
            if elapsed >= fresh_after:
                p.hw_time = clock[0]
                if elapsed >= match_after:
                    p.hw['command_position'] = target[:]
                    p.hw['position'] = target[:]
                p.hw['velocity'][joint-1] = .01 if elapsed < stopped_after else 0.
            if interrupt and elapsed >= .08:
                interrupt(p)
            if guarded:
                p.guard()
        p.pump = pump
        stubs = {**STUBS, 'JointTrajectory': lambda: NS(joint_names=[], points=[])}
        with patch.dict(rp.__dict__, stubs), patch('vr_robot.time.monotonic', lambda: clock[0]):
            yield p, clock, target, messages

    def test_recorded_arm_and_lift_feedback_delays_keep_session_enabled(self):
        # Reproduce the three observed frame gaps: 194, 194 and 149 ms.
        for joint, age, delay in ((4, .067, .127), (19, .09, .104), (19, .003, .146)):
            with self.subTest(joint=joint, age=age), \
                 self.simulation(joint=joint, age=age, fresh_after=delay) as (p, clock, target, messages):
                p.stop_stream('输入已释放')
                self.assertGreaterEqual(clock[0]-10., delay)
                self.assertTrue(p.ready)
                self.assertIsNone(p.path)
                self.assertTrue(all(p.hw['enabled']))
                self.assertFalse(p.commands)  # Hold neither enables nor disables.
                self.assertEqual(len(messages), 1)
                self.assertEqual(messages[0].points[0].positions, target)
                self.assertEqual(p.hold_positions, target)
                evidence = p.events[-1]['stop_confirmation']
                self.assertEqual(evidence['result'], 'confirmed')
                self.assertTrue(evidence['fresh_feedback'])
                self.assertGreaterEqual(evidence['wait_ms'], delay*1000)

    def test_matching_old_feedback_cannot_confirm_a_new_hold(self):
        with self.simulation(delta=0., fresh_after=math.inf) as (p, clock, *_):
            with self.assertRaisesRegex(RuntimeError, '反馈缺失或超时'):
                p.stop_stream('输入已释放')
            self.assertLess(clock[0]-10., .5)
            self.assertFalse(p.ready)
            evidence = p.events[-1]['stop_confirmation']
            self.assertEqual(evidence['result'], 'failed')
            self.assertFalse(evidence['fresh_feedback'])
            self.assertGreater(evidence['hardware_age_ms'], 400.)

    def test_waits_for_command_ack_and_then_measured_stop(self):
        with self.simulation(fresh_after=.02, match_after=.15, stopped_after=.25) as (p, clock, *_):
            p.stop_stream('输入已释放')
            self.assertGreaterEqual(clock[0]-10., .25)
            self.assertTrue(p.ready)

    def test_fresh_but_unacknowledged_target_has_a_bounded_failure(self):
        with self.simulation(fresh_after=.02, match_after=math.inf) as (p, clock, *_):
            with self.assertRaisesRegex(RuntimeError, '控制器未确认停止保持目标'):
                p.stop_stream('输入已释放')
            self.assertGreaterEqual(clock[0]-10., 3.)
            self.assertLess(clock[0]-10., 3.1)
            self.assertFalse(p.ready)
            evidence = p.events[-1]['stop_confirmation']
            self.assertEqual(evidence['result'], 'failed')
            self.assertEqual(evidence['command_error_joint'], 4)
            self.assertAlmostEqual(evidence['max_command_error_rad'], .001)
            self.assertTrue(evidence['fresh_feedback'])

    def test_ongoing_motion_is_not_accepted_as_stopped(self):
        with self.simulation(fresh_after=.02, stopped_after=math.inf) as (p, clock, *_):
            with self.assertRaisesRegex(RuntimeError, '释放输入后未确认停止'):
                p.stop_stream('输入已释放')
            self.assertLess(clock[0]-10., 3.1)
            self.assertFalse(p.ready)

    def test_estop_fault_and_link_guards_still_interrupt_waiting(self):
        cases = (
            ('急停', lambda p: setattr(p, 'gpio', lambda: [1, 1])),
            ('故障', lambda p: p.hw['errorcode'].__setitem__(0, 30)),
            ('心跳中断', lambda p: setattr(p, 'received', 0.)),
        )
        for reason, interrupt in cases:
            with self.subTest(reason=reason), \
                 self.simulation(interrupt=interrupt) as (p, clock, *_):
                with self.assertRaisesRegex(RuntimeError, reason):
                    p.stop_stream('输入已释放')
                self.assertLess(clock[0]-10., .1)
                self.assertFalse(p.ready)


if __name__ == '__main__':
    unittest.main()
