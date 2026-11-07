"""Observed chassis feedback participates in the existing goal stop latch."""
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

from fairino_slam.chassis.motion_feedback import FeedbackMonitor, STATUS_NAME
from fairino_slam.navigation.coordination import GoalCoordination, wait_timeout_from_profile
from fairino_slam.navigation.motion import CommandGate
from fairino_slam.navigation.velocity_gate import gate_output


def message(now_ros, sequence=1, session='a'*32, waiting=False, ready=True, hardware_age=.01):
    fields = dict(schema_version='1', sequence=str(sequence), ready='1' if ready else '0',
                  drive_wait='1' if waiting else '0', hardware_age_sec=str(hardware_age),
                  drive_command_peak='0' if waiting else '.2', drive_velocity_peak='0',
                  requested_vx='.02', requested_vy='0', requested_wz='0')
    status = NS(name=STATUS_NAME, hardware_id=session, level=b'\x00' if ready else b'\x02',
                message='not_ready' if not ready else 'drive_wait' if waiting else 'drive_active',
                values=[NS(key=k, value=v) for k, v in fields.items()])
    sec = int(now_ros)
    return NS(header=NS(stamp=NS(sec=sec, nanosec=round((now_ros-sec)*1e9))), status=[status])


class CoordinationGateTests(unittest.TestCase):
    def setUp(self):
        self.feedback = FeedbackMonitor(.4)
        self.coordination = GoalCoordination(self.feedback, .5)
        self.gate = CommandGate(.1, .2)
        self.now = 10.
        self.sequence = 0
        self.gate.set_goal('goal', self.now)

    def feed(self, *, waiting=False, session='a'*32, ready=True, hardware_age=.01):
        self.sequence += 1
        self.assertTrue(self.feedback.receive(
            message(self.now+90., self.sequence, session, waiting, ready, hardware_age),
            self.now, self.now+90.))

    def output(self, source=True):
        self.gate.receive([.02, 0., 0., 0., 0., 0.], self.now)
        state = self.coordination.check(self.gate.goal, self.now, self.now+90., source)
        return self.gate.output(self.now, state['ready'], ('loc', 99., 'odom'),
                                True, state['reason']), state

    def test_waiting_preserves_nonzero_intent_instead_of_halting_chassis(self):
        self.feed(waiting=True)
        output, state = self.output()
        self.assertEqual(output, (.02, 0., 0.))
        self.assertTrue(state['drive_wait'])
        self.assertEqual(state['goal_session'], 'a'*32)

    def test_wait_timeout_latches_goal_and_recovery_needs_new_goal(self):
        self.feed(waiting=True)
        self.output()
        self.now += .25
        self.feed(waiting=True)
        self.output()
        self.now += .25
        self.feed(waiting=True)
        output, state = self.output()
        self.assertEqual(output, (0., 0., 0.))
        self.assertIn('等待行走指令落实超时', self.gate.failure['reason'])
        self.feed(waiting=False)
        self.assertEqual(self.output()[0], (0., 0., 0.))
        self.gate.set_goal('new-goal', self.now)
        self.assertEqual(self.output()[0], (.02, 0., 0.))

    def test_real_drive_resume_resets_continuous_wait_budget(self):
        self.feed(waiting=True)
        self.output()
        self.now += .3
        self.feed(waiting=False)
        self.output()
        self.now += .3
        self.feed(waiting=True)
        output, state = self.output()
        self.assertEqual(output, (.02, 0., 0.))
        self.assertEqual(state['wait_age'], 0.)

    def test_worker_restart_cannot_resume_old_goal(self):
        self.feed()
        self.output()
        self.now += .05
        self.feed(session='b'*32)
        output, _ = self.output()
        self.assertEqual(output, (0., 0., 0.))
        self.assertIn('底盘执行会话已变化', self.gate.failure['reason'])
        self.gate.set_goal('new-goal', self.now)
        self.assertEqual(self.output()[0], (.02, 0., 0.))

    def test_missing_feedback_fails_closed_even_if_commands_are_fresh(self):
        output, _ = self.output()
        self.assertEqual(output, (0., 0., 0.))
        self.assertIn('等待底盘运动反馈', self.gate.failure['reason'])
        self.feed()
        self.assertEqual(self.output()[0], (0., 0., 0.))

    def test_ambiguous_publisher_is_a_latched_stop(self):
        self.feed()
        output, _ = self.output(source=False)
        self.assertEqual(output, (0., 0., 0.))
        self.assertIn('来源不唯一', self.gate.failure['reason'])

    def test_idle_does_not_pin_next_goal_to_previous_worker(self):
        self.gate.set_goal(None, self.now)
        self.feed(session='a'*32)
        self.output()
        self.feed(session='b'*32)
        self.gate.set_goal('new-goal', self.now)
        output, state = self.output()
        self.assertEqual(output, (.02, 0., 0.))
        self.assertEqual(state['goal_session'], 'b'*32)

    def test_worker_not_ready_stops_immediately(self):
        self.feed(ready=False)
        self.assertEqual(self.output()[0], (0., 0., 0.))
        self.assertIn('底盘运动反馈未就绪', self.gate.failure['reason'])

    def test_post_lifecycle_check_catches_feedback_expiring_during_block(self):
        self.feed(hardware_age=.1)
        self.gate.receive([.02, 0., 0., 0., 0., 0.], self.now)
        def snapshot():
            return dict(now=self.now, now_ros=self.now+90., ready=True,
                        health={'ready': True, 'reason': None}, stale_scans=[])
        def tick(*args):
            self.now += .35
        life = NS(tick=tick, active=lambda now: True)
        current = gate_output(
            self.gate, life, ('loc', 99., 'odom'), True, snapshot, lambda: self.now,
            lambda now, ros: self.coordination.check(self.gate.goal, now, ros, True))
        self.assertEqual(current['output'], (0., 0., 0.))
        self.assertIn('硬件反馈已过期', self.gate.failure['reason'])
        self.assertFalse(current['chassis_motion']['ready'])

    def test_bad_sample_then_healthy_before_tick_cannot_erase_failure(self):
        self.feed()
        self.output()
        broken = message(100.01, 2)
        broken.status[0].values[0].value = 'unsupported'
        self.coordination.receive(self.gate, broken, 10.01, 100.01, True)
        self.coordination.receive(self.gate, message(100.02, 3), 10.02, 100.02, True)
        self.now = 10.02
        self.assertEqual(self.output()[0], (0., 0., 0.))
        self.assertIn('无效', self.gate.failure['reason'])

    def test_worker_a_b_a_before_tick_still_latches_session_failure(self):
        self.feed()
        self.output()
        self.coordination.receive(self.gate, message(100.01, 1, 'b'*32), 10.01, 100.01, True)
        self.coordination.receive(self.gate, message(100.02, 2, 'a'*32), 10.02, 100.02, True)
        self.now = 10.02
        self.assertEqual(self.output()[0], (0., 0., 0.))
        self.assertIn('会话已变化', self.gate.failure['reason'])

    def test_not_ready_then_ready_before_tick_cannot_resume_goal(self):
        self.feed()
        self.output()
        self.coordination.receive(self.gate, message(100.01, 2, ready=False), 10.01, 100.01, True)
        self.coordination.receive(self.gate, message(100.02, 3), 10.02, 100.02, True)
        self.now = 10.02
        self.assertEqual(self.output()[0], (0., 0., 0.))
        self.assertIn('未就绪', self.gate.failure['reason'])

    def test_invalid_wait_limits_rejected(self):
        for value in (0, -1, True, math.nan, math.inf, 61):
            with self.subTest(value=value), self.assertRaises(ValueError):
                GoalCoordination(self.feedback, value)

    def test_deployed_wait_limit_required_and_validated(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'nav2.yaml'
            with self.assertRaisesRegex(RuntimeError, '完整重新部署'):
                wait_timeout_from_profile(path)
            path.write_text('controller_server:\n  ros__parameters:\n    FollowPath:\n      coordination_wait_timeout: 15\n')
            self.assertEqual(wait_timeout_from_profile(path), 15.)
            path.write_text('controller_server:\n  ros__parameters:\n    FollowPath:\n      coordination_wait_timeout: true\n')
            with self.assertRaises(ValueError):
                wait_timeout_from_profile(path)


if __name__ == '__main__':
    unittest.main()


class PublisherDiscoveryTests(unittest.TestCase):
    @staticmethod
    def endpoint(name='fairino_keyboard_guard', namespace='/', serial=1):
        return NS(node_name=name, node_namespace=namespace, endpoint_gid=[serial]*24)

    def test_missing_duplicate_and_wrong_identity_are_distinct(self):
        from fairino_slam.navigation.coordination import publisher_state
        for publishers, kind, ready in [
            ([], 'missing', False),
            ([self.endpoint()], 'valid', True),
            ([self.endpoint(), self.endpoint(serial=2)], 'duplicate', False),
            ([self.endpoint(namespace='/other')], 'unexpected', False),
            ([self.endpoint(name='other')], 'unexpected', False),
        ]:
            with self.subTest(kind=kind):
                result = publisher_state(publishers)
                self.assertEqual(result['source_state'], kind)
                self.assertEqual(result['source_ok'], ready)
                self.assertEqual(result['publisher_count'], len(publishers))
                self.assertEqual(len(result['publishers']), len(publishers))

    def test_healthy_sample_does_not_override_missing_or_duplicate_source(self):
        from fairino_slam.navigation.coordination import FeedbackInbox, publisher_state
        for publishers in ([], [self.endpoint(), self.endpoint(serial=2)]):
            inbox = FeedbackInbox(FeedbackMonitor(.4), 15.)
            details = publisher_state(publishers)
            inbox.set_source(details['source_ok'], details)
            gate = CommandGate(.1, .2)
            gate.set_goal('goal', 10.)
            inbox.set_goal('goal')
            inbox.receive(message(100.), lambda: 10., lambda: 100.)
            state = inbox.check(gate, lambda: 10., lambda: 100., False)
            self.assertFalse(state['ready'])
            self.assertEqual(state['publisher_count'], len(publishers))
            self.assertIsNotNone(gate.failure)
            self.assertIn('已断开' if not publishers else '重复', gate.failure['reason'])
            self.assertEqual(gate.output(10., True, ('loc', 99., 'odom'), True), (0., 0., 0.))

    def test_idle_missing_source_is_pending_without_a_goal_failure(self):
        from fairino_slam.navigation.coordination import publisher_state
        state = GoalCoordination(FeedbackMonitor(.4), 15.).check(
            None, 10., 100., False, publisher_state([]))
        self.assertFalse(state['ready'])
        self.assertIn('等待底盘接入完成', state['reason'])
        self.assertNotIn('不唯一', state['reason'])
