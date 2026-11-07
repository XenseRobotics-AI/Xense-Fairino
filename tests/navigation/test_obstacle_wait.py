"""Obstacle waits must not weaken protective stops or replay old commands."""
import unittest
from types import SimpleNamespace as NS
from fairino_slam.navigation.velocity_gate import receive_execution_state
from fairino_slam.navigation.motion import CommandGate
from fairino_slam.navigation.desktop import navigation_status


class ObstacleWaitTests(unittest.TestCase):
    def gate(self):
        gate = CommandGate(.06, .1)
        gate.set_goal('goal', 10.)
        gate.set_execution('following', 'following', 10.)
        gate.receive([.05, 0., 0., 0., 0., 0.], 10.01)
        self.assertEqual(gate.output(10.02, True, 'session'), (.05, 0., 0.))
        return gate

    def test_blocked_longer_than_progress_timeout_stays_zero_and_recovers_fresh(self):
        gate = self.gate()
        for i in range(1000):
            now = 10.1+i*.1
            gate.set_execution('waiting', '局部障碍，持续规划', now)
            gate.receive([.05, 0., 0., 0., 0., 0.], now)  # late old follower sample
            self.assertEqual(gate.output(now+.01, True, 'session'), (0., 0., 0.))
            self.assertIsNone(gate.failure)
        gate.set_execution('following', '新路径', 110.1)
        self.assertEqual(gate.output(110.11, True, 'session'), (0., 0., 0.))
        gate.receive([.01, 0., 0., 0., 0., 0.], 110.12)
        self.assertEqual(gate.output(110.13, True, 'session'), (.01, 0., 0.))

    def test_pending_initial_plan_has_heartbeat_without_motion(self):
        gate = CommandGate(.06, .1)
        gate.set_goal('goal', 10.)
        for now in (10., 10.5, 11., 11.5):
            gate.set_execution('planning', '正在规划', now)
            self.assertEqual(gate.output(now, True, 'session'), (0., 0., 0.))
        self.assertIsNone(gate.failure)

    def test_missing_wait_heartbeat_is_terminal(self):
        gate = self.gate()
        gate.set_execution('waiting', '障碍', 10.1)
        gate.output(10.8, True, 'session')
        self.assertIn('执行状态断流', gate.failure['reason'])
        gate.set_execution('following', '恢复', 10.9)
        gate.receive([.01, 0., 0., 0., 0., 0.], 10.9)
        self.assertEqual(gate.output(10.9, True, 'session'), (0., 0., 0.))

    def test_resume_requires_new_commands_even_with_fresh_status(self):
        gate = self.gate()
        gate.set_execution('waiting', '障碍', 10.1)
        gate.set_execution('following', '新路径', 10.2)
        gate.set_execution('following', '新路径', 11.)
        gate.output(11., True, 'session')
        self.assertIn('未收到新速度', gate.failure['reason'])

    def test_localization_fault_during_wait_stays_terminal_after_recovery(self):
        gate = self.gate()
        gate.set_execution('waiting', '障碍', 10.1)
        gate.output(10.2, False, 'session', readiness_reason='定位丢失')
        self.assertIn('定位丢失', gate.failure['reason'])
        gate.set_execution('following', '新路径', 10.3)
        gate.receive([.01, 0., 0., 0., 0., 0.], 10.3)
        self.assertEqual(gate.output(10.3, True, 'session'), (0., 0., 0.))

    def test_session_change_and_bad_command_source_still_stop_waiting_goal(self):
        for kwargs in ({'session': 'new-session'}, {'session': 'session', 'source_ok': False}):
            with self.subTest(kwargs=kwargs):
                gate = self.gate()
                gate.set_execution('waiting', '障碍', 10.1)
                gate.output(10.2, True, **kwargs)
                self.assertIsNotNone(gate.failure)

    def test_cancel_then_late_status_cannot_restore_old_goal(self):
        gate = self.gate()
        gate.set_execution('waiting', '障碍', 10.1)
        gate.set_goal(None, 10.2)
        gate.set_execution('following', '迟到状态', 10.3)
        gate.receive([.05, 0., 0., 0., 0., 0.], 10.3)
        self.assertEqual(gate.output(10.3, True, 'session'), (0., 0., 0.))
        self.assertIsNone(gate.execution)
        gate.set_goal('new-goal', 11.)
        self.assertIsNone(gate.command)
        self.assertIsNone(gate.execution)

    def test_controller_fault_reason_survives_later_healthy_states(self):
        gate = self.gate()
        gate.set_execution('failed', '控制器 TF 过期', 10.1)
        gate.set_execution('following', '新路径', 10.2)
        self.assertEqual(gate.failure['reason'], '控制器 TF 过期')

    def test_ui_shows_wait_reason_when_stack_is_ready_and_fault_takes_priority(self):
        state = dict(goal='goal', localization_ready=True, controller_ready=True,
                     execution=dict(goal='goal', phase='waiting', reason='局部障碍，持续规划'))
        self.assertEqual(navigation_status(state, 'goal'), '局部障碍，持续规划')
        state['failure'] = dict(goal='goal', reason='雷达断流')
        self.assertEqual(navigation_status(state, 'goal'), '雷达断流')
        state['failure'] = {}
        state['execution']['phase'] = 'following'
        self.assertEqual(navigation_status(state, 'goal'), '导航中')

    def test_ui_explains_abort_cancel_and_ignores_old_wait(self):
        state = dict(execution=dict(goal='old', phase='waiting', reason='旧障碍'),
                     last_goal=dict(id='new', status=6))
        self.assertIn('导航失败', navigation_status(state, 'new'))
        state['last_goal']['status'] = 5
        self.assertIn('已取消', navigation_status(state, 'new'))


class ExecutionStreamTests(unittest.TestCase):
    def message(self, phase='waiting', sec=100, nanosec=0):
        return NS(header=NS(stamp=NS(sec=sec, nanosec=nanosec)),
                  status=[NS(name=phase, message='原因')])

    def test_rejects_previous_goal_stale_future_and_wrong_source(self):
        for message, now_ros, goal_stamp, source in (
                (self.message(), 100., 101., True),
                (self.message(), 100.7, 99., True),
                (self.message(), 99., 99., True),
                (self.message(), 100., 99., False),
                (self.message(nanosec=1000000000), 101., 99., True)):
            gate = CommandGate(.06, .1)
            gate.set_goal('goal', 10.)
            self.assertFalse(receive_execution_state(gate, message, 10., now_ros, goal_stamp, source))
            self.assertIsNone(gate.execution)

    def test_reordered_following_sample_cannot_release_wait(self):
        gate = CommandGate(.06, .1)
        gate.set_goal('goal', 10.)
        self.assertTrue(receive_execution_state(gate, self.message(), 10., 100., 99., True))
        old = self.message('following', sec=99, nanosec=900000000)
        self.assertFalse(receive_execution_state(gate, old, 10.1, 100.1, 99., True))
        self.assertEqual(gate.execution['phase'], 'waiting')

    def test_transit_age_counts_towards_heartbeat_expiry(self):
        gate = CommandGate(.06, .1)
        gate.set_goal('goal', 10.)
        receive_execution_state(gate, self.message(), 10., 100.5, 99., True)
        gate.output(10.2, True, 'session')
        self.assertIn('执行状态断流', gate.failure['reason'])

    def test_cancel_state_cannot_reopen_before_action_acknowledgement(self):
        gate = CommandGate(.06, .1)
        gate.set_goal('goal', 10.)
        receive_execution_state(gate, self.message('canceled'), 10., 100., 99., True)
        receive_execution_state(gate, self.message('following', sec=101), 11., 101., 99., True)
        self.assertEqual(gate.execution['phase'], 'canceled')
        gate.receive([.05, 0., 0., 0., 0., 0.], 11.)
        self.assertEqual(gate.output(11., True, 'session'), (0., 0., 0.))
