"""Velocity-stream regressions independent of ROS and robot hardware."""
import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from apps.navigation.motion import CommandGate


class GateDiagnosticsTests(unittest.TestCase):
    def ready_gate(self):
        gate = CommandGate(.06, .1)
        gate.set_goal('goal', 1.)
        gate.receive([.03, 0, 0, 0, 0, 0], 1.1)
        self.assertEqual(gate.output(1.11, True, 'session'), (.03, 0., 0.))
        return gate

    def test_late_receive_cannot_hide_timeout_before_next_timer_tick(self):
        gate = self.ready_gate()
        # The executor delivers a late command BEFORE running the stop timer.
        gate.receive([.04, 0, 0, 0, 0, 0], 1.45)
        self.assertEqual(gate.output(1.46, True, 'session'), (0., 0., 0.))
        self.assertEqual(gate.failure['code'], 'command_timeout')
        self.assertAlmostEqual(gate.failure['command_age'], .35)
        self.assertAlmostEqual(gate.failure['command_interval'], .35)
        self.assertEqual(gate.failure['command_timeout'], .3)
        # Subsequent fresh input does not restart that failed goal.
        gate.receive([.04, 0, 0, 0, 0, 0], 1.5)
        self.assertEqual(gate.output(1.51, True, 'session'), (0., 0., 0.))

    def test_timer_timeout_retains_gap_and_count_at_the_first_failure(self):
        gate = self.ready_gate()
        gate.receive([.03, 0, 0, 0, 0, 0], 1.2)
        gate.receive([.03, 0, 0, 0, 0, 0], 1.4)
        gate.output(1.71, True, 'session')
        failure = copy.deepcopy(gate.failure)
        self.assertEqual(failure['command_count'], 3)
        self.assertAlmostEqual(failure['max_command_interval'], .2)
        self.assertAlmostEqual(failure['command_age'], .31)
        gate.receive([.03, 0, 0, 0, 0, 0], 2.)
        self.assertEqual(gate.failure, failure)
        gate.set_goal(None, 2.1)
        gate.output(2.2, False, 'changed-session')
        self.assertEqual(gate.failure, failure)
        self.assertEqual(gate.reason, failure['reason'])

    def test_failure_context_is_immutable_after_cancellation_and_recovery(self):
        gate = self.ready_gate()
        gate.output(1.42, True, 'session')
        context = {'topic_ages': {'/front/scan': {'age': .02}},
                   'lifecycle': {'controller_server': {'state': 3, 'age': .1}},
                   'session_token': ['session', 1., 'epoch']}
        gate.capture_failure_context(context)
        saved = copy.deepcopy(gate.failure)
        context['topic_ages']['/front/scan']['age'] = 4.
        context['session_token'][2] = 'new-epoch'
        gate.set_goal(None, 1.5)
        gate.capture_failure_context(context)
        self.assertEqual(gate.failure, saved)
        gate.set_goal('new-goal', 2.)
        self.assertIsNone(gate.failure)

    def test_fresh_jitter_below_timeout_continues_without_latching(self):
        gate = self.ready_gate()
        now = 1.1
        for interval in [.1, .19, .08, .29, .11]*20:
            now += interval
            gate.receive([.03, -.02, 0, 0, 0, .02], now)
            self.assertEqual(gate.output(now+.005, True, 'session'), (.03, -.02, .02))
            self.assertIsNone(gate.failure)
        self.assertEqual(gate.command_count, 101)
        self.assertAlmostEqual(gate.max_command_interval, .29)

    def test_invalid_idle_command_does_not_poison_the_next_goal(self):
        gate = CommandGate(.06, .1)
        try:
            gate.receive([float('nan'), 0, 0, 0, 0, 0], 1.)
        except ValueError:
            gate.fail('收到无效或非二维速度指令', 1., code='invalid_command')
            gate.command = None
        self.assertIsNone(gate.failure)
        gate.set_goal('new-goal', 2.)
        gate.receive([.03, 0, 0, 0, 0, 0], 2.1)
        self.assertEqual(gate.output(2.11, True, 'session'), (.03, 0., 0.))
        self.assertIsNone(gate.failure)

    def test_new_goal_resets_telemetry_without_old_idle_commands(self):
        gate = self.ready_gate()
        gate.output(2., True, 'session')
        gate.set_goal(None, 2.1)
        gate.receive([.06, 0, 0, 0, 0, 0], 2.2)
        gate.set_goal('new', 20.)
        self.assertEqual(gate.output(25., True, 'session'), (0., 0., 0.))
        self.assertIsNone(gate.failure)
        self.assertEqual(gate.telemetry(25.)['command_count'], 0)
        self.assertIsNone(gate.telemetry(25.)['command_age'])
        gate.receive([.03, 0, 0, 0, 0, 0], 25.1)
        self.assertEqual(gate.output(25.11, True, 'session'), (.03, 0., 0.))
        self.assertEqual(gate.command_count, 1)
        self.assertEqual(gate.max_command_interval, 0.)

    def test_distinct_failures_preserve_machine_readable_first_cause(self):
        for kwargs, expected in [({'session': 'new'}, 'session_changed'),
                                 ({'source_ok': False}, 'command_source'),
                                 ({'ready': False, 'readiness_reason': '后雷达观测过期'}, 'not_ready')]:
            with self.subTest(expected):
                gate = self.ready_gate()
                params = dict(now=1.2, ready=True, session='session')
                params.update(kwargs)
                self.assertEqual(gate.output(**params), (0., 0., 0.))
                self.assertEqual(gate.failure['code'], expected)
                gate.fail('later invalid command', 1.3, code='invalid_command')
                self.assertEqual(gate.failure['code'], expected)


if __name__ == '__main__':
    unittest.main()
