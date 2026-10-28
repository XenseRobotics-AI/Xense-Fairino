"""Offline receiver contracts; no ROS, SSH or physical robot is used."""
import copy
import math
import threading
import unittest
from unittest.mock import patch

import numpy as np

from scripts.pico4.native_ik_test import server
from scripts.pico4.native_ik_test.planner import NativePlanner
from wrist_guard import GuardSession, policy


def hardware():
    left = np.radians([45., 75., -100., 85., 85., 0., 0.])
    right = left*np.array([-1., 1., -1., 1., -1., 1., -1.])
    q = np.r_[left, right, np.zeros(6)].tolist()
    return dict(position=q[:], command_position=q[:], velocity=[0.]*20,
                enabled=[True]*20, error=0, errorcode=[0]*20)


class ServerTests(unittest.TestCase):
    def setUp(self):
        self.clock = [100.]
        self.clock_patch = patch('scripts.pico4.native_ik_test.server.time.monotonic',
                                 lambda: self.clock[0])
        self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)
        self.events = []; self.publications = []; self.order = []
        self.planner = NativePlanner()

        def base_init(p, args):
            p.args = args
            p.tools = self.planner.tools
            p.hw = hardware(); p.hw_time = self.clock[0]
            p.wrist = GuardSession(); p.wrist.active = policy(True)
            p.input_lock = threading.Lock(); p.input_event = threading.Event()
            p.exit = threading.Event(); p.fault = ''
            p.packet = None; p.received = 0.
            p.enable_attempted = p.session_enabled = True; p.enabling = False
            p.selected = p.baseline = p.goal = p.pending_goal = None
            p.hold_positions = p.hw['command_position'][:]
            p.moving_joints = p.path = None
            p.cancel_count = None; p.last_request = 0
            p.ready = False; p.phase = 'test'; p.last_status = 0.
            p.gpio = lambda: [0, 0]
        with patch.object(server.VRProbe, '__init__', base_init):
            self.p = server.NativeProbe(server.arguments([]), self.planner)
        self.p.emit = lambda **data: self.events.append(data)
        self.p.report = lambda *a, **kw: None
        self.p.native_active = True
        self.p.mode = 'idle'; self.p.settled = True
        self.p.stop_completed = self.clock[0]-.01
        self.planner.reset(self.p.hw)

        def publish(points, now):
            self.order.append('publish')
            self.publications.append(copy.deepcopy(points))
            self.p.path, self.p.path_time = copy.deepcopy(points), now
        self.p.publish_positions = publish
        self.real_commit = self.planner.commit

        def commit(proposal):
            self.order.append('commit')
            self.real_commit(proposal)
        self.planner.commit = commit
        self.sequence = 0

    def packet(self, goals=None, **overrides):
        self.sequence += 1
        request = None if goals is None else dict(kind=server.KIND, model=server.MODEL_ID,
            tool_id=self.p.tools['id'], id=self.sequence, created=self.clock[0],
            goals={side: np.asarray(goal).reshape(-1).tolist() for side, goal in goals.items()},
            _wrist_guard=policy(True))
        self.p.packet = dict(request=request, ack=self.clock[0], focused=True,
                             cancel=0, wrist_guard=policy(True), **overrides)
        self.p.received = self.clock[0]

    def goal(self, side='left'):
        goal = self.planner.solvers[side].forward(self.planner.references[side])
        goal[0, 3] += 30.
        return goal

    def follow(self):
        if self.p.path:
            q = self.p.path[-1][1][:]
            self.p.hw['position'] = q[:]; self.p.hw['command_position'] = q[:]
            # Lifecycle tests use an ideal position follower, not inferred
            # motor velocities from the unlimited command difference. Tests
            # that exercise physical settling provide measured speed explicitly.
            self.p.hw['velocity'] = [0.]*20
        self.p.hw_time = self.clock[0]

    def tick(self, goals=None, dt=.01, update=True, follow=True):
        self.clock[0] += dt
        if follow:
            self.follow()
        if update:
            self.packet(goals)
        return self.p.cycle(self.clock[0], dt)

    def arm(self):
        self.tick(None)
        self.assertTrue(self.p.ready)
        self.assertFalse(self.p.need_rearm)
        self.assertIn('已使能就绪', self.p.phase)

    def start(self, count=12):
        self.arm()
        goals = {'left': self.goal(), 'right': self.goal('right')}
        for _ in range(count):
            self.tick(goals)
        self.assertEqual(self.p.mode, 'tracking')
        return goals

    def test_protocol_rejects_invalid_transforms_age_model_and_joint_packets(self):
        self.packet({'left': self.goal()})
        request = self.p.packet['request']
        self.assertEqual(set(server.validate_intent(request, self.clock[0], self.p.tools, .15)), {'left'})
        variants = [dict(request, kind='wrong'), dict(request, model='wrong'),
                    dict(request, tool_id='wrong'), dict(request, id=True),
                    dict(request, created=self.clock[0]-.151), dict(request, created=math.nan),
                    dict(request, goals={}), dict(request, arms={}), dict(request, lift=.5),
                    dict(request, goals={'left': [math.nan]*16}),
                    dict(request, goals={'left': [0.]*16})]
        for invalid in variants:
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                server.validate_intent(invalid, self.clock[0], self.p.tools, .15)

    def test_neutral_handshake_atomic_publish_and_body_hold(self):
        goals = {'left': self.goal(), 'right': self.goal('right')}
        self.tick(goals)
        self.assertFalse(self.publications)
        self.arm(); self.tick(goals)
        self.assertEqual(self.order, ['publish', 'commit'])
        self.assertEqual(self.publications[-1][-1][1][14:], [0.]*6)
        for side, offset in (('left', 0), ('right', 7)):
            np.testing.assert_array_equal(self.planner.references[side],
                                          self.publications[-1][-1][1][offset:offset+7])

    def test_second_arm_validation_failure_publishes_and_commits_nothing(self):
        proposal = self.planner.propose({'left': self.goal()}, self.p.hw, .01)
        proposal['arms']['right']['tcp'][0] = 2.
        before = self.planner.references
        with self.assertRaises(ValueError):
            self.p._publish_proposal(proposal, self.clock[0])
        self.assertFalse(self.order)
        for side in before:
            np.testing.assert_array_equal(before[side], self.planner.references[side])

    def test_publisher_failure_never_commits(self):
        proposal = self.planner.propose({}, self.p.hw, .01)
        self.p.publish_positions = lambda *_: (_ for _ in ()).throw(RuntimeError('publisher failed'))
        with self.assertRaisesRegex(RuntimeError, 'publisher failed'):
            self.p._publish_proposal(proposal, self.clock[0])
        self.assertFalse(self.order)

    def test_equal_hold_is_not_blocked_by_new_dynamic_wrist_reserve(self):
        self.start(3)
        endpoint = self.publications[-1][-1][1][:]
        self.p.hw['velocity'][5] = math.radians(100.)
        proposal = self.planner.propose({}, self.p.hw, .01)
        original = self.p.wrist.guard.arm_segment
        calls = []

        def only_static_hold(start, target, velocity, **kwargs):
            calls.append((start, target, velocity))
            if any(velocity):
                raise ValueError('new measured-speed reserve rejects motion')
            return original(start, target, velocity, **kwargs)

        with patch.object(self.p.wrist.guard, 'arm_segment', side_effect=only_static_hold):
            self.p._publish_proposal(proposal, self.clock[0])
        self.assertEqual(self.publications[-1][-1][1], endpoint)
        self.assertEqual(self.order[-2:], ['publish', 'commit'])
        self.assertTrue(all(start == target and not any(velocity)
                            for start, target, velocity in calls))

    def test_unrequested_arm_cannot_smuggle_motion_as_a_hold(self):
        self.start(3)
        proposal = self.planner.propose({}, self.p.hw, .01)
        arm = proposal['arms']['left']
        arm['target'][0] += .001
        arm['tcp'] = self.planner.solvers['left'].forward(arm['target']).reshape(-1).tolist()
        before = len(self.publications)
        with self.assertRaises(ValueError):
            self.p._publish_proposal(proposal, self.clock[0])
        self.assertEqual(len(self.publications), before)

    def test_release_holds_last_target_then_requires_fresh_stop_and_new_neutral(self):
        goals = self.start()
        target = self.publications[-1][-1][1][:]
        before = len(self.publications)
        self.tick(None)
        self.assertEqual(self.p.mode, 'settling')
        self.assertEqual(len(self.publications), before+1)
        self.assertEqual(self.publications[-1][-1][1], target)
        self.assertTrue(self.p.command_stopped); self.assertFalse(self.p.settled)
        count = len(self.publications)
        self.tick(None, follow=False)
        self.assertFalse(self.p.settled)  # Old feedback cannot confirm stop.
        self.assertEqual(len(self.publications), count)
        for _ in range(15):
            self.tick(goals)
            if self.p.settled:
                break
        self.assertTrue(self.p.settled); self.assertTrue(self.p.need_rearm)
        self.tick(goals)
        self.assertEqual(len(self.publications), count)  # Held grip cannot resume.
        self.tick(None); self.assertFalse(self.p.need_rearm)
        self.tick(goals); self.assertEqual(self.p.mode, 'tracking')
        self.assertTrue(any(e.get('native_event') == 'measured_stop_confirmed' for e in self.events))

    def test_constant_hold_does_not_claim_physically_moving_arm_is_settled(self):
        self.start(2)
        target = self.publications[-1][-1][1][:]
        self.clock[0] += .01; self.follow(); self.packet(None)
        self.p.hw['velocity'][0] = .2
        self.p.cycle(self.clock[0], .01)
        self.assertEqual(self.p.mode, 'settling')
        self.assertTrue(self.p.command_stopped)
        self.assertEqual(self.publications[-1][-1][1], target)
        count = len(self.publications)
        for _ in range(15):
            self.clock[0] += .01; self.follow(); self.packet(None)
            self.p.hw['velocity'][0] = .2
            self.p.cycle(self.clock[0], .01)
        self.assertFalse(self.p.settled)
        self.assertIsNone(self.p.settle_since)
        self.assertEqual(len(self.publications), count)
        for _ in range(15):
            self.tick(None)
        self.assertTrue(self.p.settled)

    def test_hold_timeout_with_fresh_feedback_that_never_stops(self):
        self.start(2)
        self.tick(None)
        self.clock[0] += self.p.args.stop_timeout_s+.01
        self.follow(); self.packet(None)
        self.p.hw['velocity'][0] = .01
        with self.assertRaisesRegex(RuntimeError, 'stop/settle deadline exceeded'):
            self.p.cycle(self.clock[0], .01)
        self.assertFalse(self.p.settled)

    def test_old_feedback_cannot_keep_hold_confirmation_alive(self):
        self.start(2)
        self.tick(None)
        self.clock[0] += .061
        self.packet(None)
        with self.assertRaisesRegex(RuntimeError, 'hardware age'):
            self.p.cycle(self.clock[0], .01)
        self.assertFalse(self.p.settled)

    def test_stop_dwell_requires_new_frames_and_restarts_after_velocity_excursion(self):
        self.start(4)
        while self.p.mode in ('tracking', 'stopping'):
            self.tick(None)
        self.tick(None)
        first = self.p.settle_since
        self.assertIsNotNone(first)
        for _ in range(4):
            self.tick(None)
        self.assertFalse(self.p.settled)
        self.clock[0] += .01; self.follow(); self.packet(None)
        self.p.hw['velocity'][0] = .006
        self.p.cycle(self.clock[0], .01)
        self.assertIsNone(self.p.settle_since)
        self.tick(None)
        self.assertGreater(self.p.settle_since, first)
        for _ in range(9):
            self.tick(None)
        self.assertFalse(self.p.settled)
        self.tick(None)
        self.assertTrue(self.p.settled)
        event = next(e for e in self.events if e.get('native_event') == 'measured_stop_confirmed')
        self.assertGreaterEqual(event['dwell_ms']+1e-9, 100.)

    def test_body_held_speed_trip_is_preserved(self):
        self.start(3)
        self.p.hw['velocity'][15] = math.radians(1.01)
        before = len(self.publications)
        with self.assertRaisesRegex(RuntimeError, 'J16'):
            self.p.cycle(self.clock[0], .01)
        self.assertEqual(len(self.publications), before)

    def test_input_expiry_and_malformed_goal_hold_both_last_targets(self):
        for malformed in (False, True):
            with self.subTest(malformed=malformed):
                self.setUp()
                self.start(4)
                before = self.planner.references
                last_target = self.publications[-1][-1][1][:]
                self.clock[0] += .01; self.follow()
                self.packet({'left': self.goal()})
                if malformed:
                    self.p.packet['request']['goals']['left'][0] = math.nan
                else:
                    self.p.packet['request']['created'] -= .16
                self.p.cycle(self.clock[0], .01)
                self.assertEqual(self.p.mode, 'settling')
                self.assertTrue(self.p.need_rearm)
                self.assertTrue(self.p.command_stopped)
                self.assertEqual(self.publications[-1][-1][1], last_target)
                for side in before:
                    np.testing.assert_array_equal(self.planner.references[side], before[side])
                self.assertEqual(self.order[-2:], ['publish', 'commit'])

    def test_cycle_gap_holds_last_command_instead_of_chasing_new_goal(self):
        self.start(6)
        before = self.planner.references
        self.tick({'left': self.goal()}, dt=.05)
        self.assertEqual(self.p.stop_reason, 'control_cycle_gap')
        self.assertEqual(self.p.mode, 'settling')
        for side in before:
            np.testing.assert_array_equal(self.planner.references[side], before[side])

    def test_packet_arriving_during_guard_is_fresh(self):
        goals = self.start(3)
        self.clock[0] += .01
        self.follow()
        cycle_started = self.clock[0]
        original_guard = self.p.guard
        injected = []

        def guard(*args, **kwargs):
            original_guard(*args, **kwargs)
            if not injected:
                injected.append(True)
                # The input thread receives a new packet after the cycle
                # started, but before the control thread takes its snapshot.
                self.clock[0] += .002
                self.packet(goals)

        with patch.object(self.p, 'guard', guard):
            self.p.cycle(cycle_started, .01)
        self.assertEqual(self.p.mode, 'tracking')
        self.assertFalse(self.p.need_rearm)
        self.assertAlmostEqual(self.p.input_timing['heartbeat_age_ms'], 0.)
        self.assertGreater(self.p.input_timing['received_monotonic'], cycle_started)

    def test_packet_expiring_during_guard_still_holds(self):
        goals = self.start(3)
        self.clock[0] += .01
        self.follow()
        self.packet(goals)
        self.p.received -= .149
        self.p.packet['ack'] -= .149
        self.p.packet['request']['created'] -= .149
        cycle_started = self.clock[0]
        original_guard = self.p.guard
        advanced = []

        def guard(*args, **kwargs):
            original_guard(*args, **kwargs)
            if not advanced:
                advanced.append(True)
                self.clock[0] += .002

        with patch.object(self.p, 'guard', guard):
            self.p.cycle(cycle_started, .01)
        self.assertEqual(self.p.mode, 'settling')
        self.assertEqual(self.p.stop_reason, 'input_heartbeat_timeout')
        event = next(e for e in self.events if e.get('native_event') == 'hold_started')
        self.assertAlmostEqual(event['input_timing']['heartbeat_age_ms'], 151.)

    def test_packet_snapshot_rejects_future_and_expired_times(self):
        self.arm()
        for name, reason in (('received', 'input_heartbeat_timeout'),
                             ('ack', 'input_round_trip_timeout'),
                             ('created', 'native TCP input expired')):
            for delta in (.001, -.151):
                with self.subTest(name=name, delta=delta):
                    self.packet({'left': self.goal()})
                    value = self.clock[0]+delta
                    if name == 'received':
                        self.p.received = value
                    elif name == 'ack':
                        self.p.packet['ack'] = value
                    else:
                        self.p.packet['request']['created'] = value
                    goals, rejected, neutral, _ = self.p._packet_goals(self.clock[0])
                    self.assertEqual(rejected, reason)
                    self.assertFalse(goals)
                    self.assertFalse(neutral)

    def test_idle_cycle_gap_cannot_rearm_in_same_tick_with_new_neutral(self):
        self.arm()
        self.clock[0] += .05
        self.follow()
        cycle_started = self.clock[0]
        self.clock[0] += .002
        self.packet(None)
        self.p.cycle(cycle_started, .05)
        self.assertTrue(self.p.need_rearm)
        self.assertFalse(self.p.ready)
        self.assertEqual(self.p.stop_reason, 'control_cycle_gap')
        self.assertIn('control_cycle_gap', self.p.phase)
        self.tick(None)
        self.assertTrue(self.p.ready)
        self.assertIn('已使能就绪', self.p.phase)

    def test_stale_hardware_fault_and_estop_use_fail_safe(self):
        self.start(3)
        for kind in ('stale', 'fault', 'estop'):
            with self.subTest(kind=kind):
                backup = copy.deepcopy(self.p.hw); old_time = self.p.hw_time
                if kind == 'stale':
                    self.p.hw_time = self.clock[0]-.061
                elif kind == 'fault':
                    self.p.hw['errorcode'][0] = 30
                elif kind == 'estop':
                    self.p.gpio = lambda: [1, 1]
                count = len(self.publications)
                with self.assertRaises(RuntimeError):
                    self.p.cycle(self.clock[0], .01)
                self.assertEqual(len(self.publications), count)
                self.p.hw = backup; self.p.hw_time = old_time
                self.p.gpio = lambda: [0, 0]

    def test_active_arm_has_no_hidden_added_twenty_degree_speed_trip(self):
        self.start(3)
        self.p.hw['velocity'][0] = math.radians(100.)
        # Only the removed synthetic arm trip is at issue here; wrist
        # prediction, held-body limits and fault/estop guards remain separate.
        self.p.guard()
        self.assertAlmostEqual(self.p.actual_max_speed, 100.)

    def test_guard_cannot_be_disabled_or_changed_during_session(self):
        self.arm()
        self.packet(None); self.p.packet['wrist_guard'] = policy(False)
        self.p.cycle(self.clock[0], .01)
        self.assertTrue(self.p.wrist.guard.enabled)
        self.assertTrue(self.p.need_rearm)
        self.packet(None); self.p.packet['wrist_guard'] = policy(True, epoch=2)
        self.p.cycle(self.clock[0], .01)
        self.assertEqual(self.p.wrist.active, policy(True))

    def test_command_feedback_accepts_recent_publication_then_detects_loss(self):
        self.start(5)
        self.clock[0] += .01; self.follow()
        self.p._command_health(self.clock[0])
        self.assertIsNone(self.p.command_mismatch_since)
        self.p.hw['command_position'][0] += .01
        self.p._command_health(self.clock[0])
        self.clock[0] += .061
        with self.assertRaisesRegex(RuntimeError, 'command feedback'):
            self.p._command_health(self.clock[0])
        mismatch = self.events[-1]
        self.assertEqual(mismatch['native_event'], 'command_feedback_mismatch')
        self.assertEqual(mismatch['hardware_command_position'], self.p.hw['command_position'])
        self.assertEqual(mismatch['last_published_target'], self.publications[-1][-1][1])

    def test_shutdown_holds_without_new_input_until_confirmed_then_exits(self):
        self.start(5)
        endpoint = self.publications[-1][-1][1][:]
        self.p.shutdown_requested = True
        for _ in range(100):
            continued = self.tick(update=False)
            if not continued:
                break
        else:
            self.fail('shutdown did not finish')
        self.assertTrue(self.p.settled)
        self.assertTrue(self.p.command_stopped)
        self.assertEqual(self.publications[-1][-1][1], endpoint)

    def test_read_only_and_enable_order_keep_envelope_check_before_enable(self):
        with patch.object(server.VRProbe, 'enable_all', lambda _: self.order.append('enable')), \
                patch.object(self.p, '_feedback_preflight'):
            self.p.enable_all()
        self.assertEqual(self.order, ['enable'])
        self.assertEqual(self.events[-1]['native_event'], 'initial_envelope_checked')
        self.p.hw['command_position'][5:7] = np.radians([69., 80.]).tolist()
        with patch.object(server.VRProbe, 'enable_all', lambda _: self.order.append('bad-enable')), \
                patch.object(self.p, '_feedback_preflight'):
            with self.assertRaises(ValueError):
                self.p.enable_all()
        self.assertNotIn('bad-enable', self.order)

    def test_feedback_preflight_rejects_slow_stream_before_enable(self):
        self.p.native_active = False
        self.p.enable_attempted = self.p.session_enabled = False
        self.p.hw['enabled'] = [False]*20
        last_frame = [self.clock[0]]

        def pump(dt):
            self.clock[0] += dt
            if self.clock[0]-last_frame[0] >= .08:
                last_frame[0] = self.clock[0]
                self.p.hw_time = self.clock[0]
        self.p.pump = pump
        with patch.object(server.VRProbe, 'enable_all', lambda _: self.order.append('enable')):
            with self.assertRaisesRegex(RuntimeError, 'preflight rejected before enable'):
                self.p.enable_all()
        self.assertNotIn('enable', self.order)
        report = self.events[-1]
        self.assertEqual(report['native_event'], 'feedback_preflight')
        self.assertEqual(report['result'], 'rejected')
        self.assertGreater(report['exceed_count'], 0)
        self.assertGreater(report['max_age_ms'], 60.)
        self.assertGreaterEqual(report['duration_ms'], 1000.)

    def test_feedback_preflight_accepts_fresh_stream_then_checks_envelope(self):
        self.p.native_active = False
        self.p.enable_attempted = self.p.session_enabled = False
        self.p.hw['enabled'] = [False]*20
        count = [0]

        def pump(dt):
            self.clock[0] += dt
            count[0] += 1
            if count[0] % 2 == 0:
                self.p.hw_time = self.clock[0]
        self.p.pump = pump
        with patch.object(server.VRProbe, 'enable_all', lambda _: self.order.append('enable')):
            self.p.enable_all()
        self.assertEqual(self.order, ['enable'])
        report = next(e for e in self.events if e.get('native_event') == 'feedback_preflight')
        self.assertEqual(report['result'], 'passed')
        self.assertEqual(report['exceed_count'], 0)
        self.assertGreaterEqual(report['unique_frames'], report['required_frames'])
        self.assertEqual(self.events[-1]['native_event'], 'initial_envelope_checked')

    def test_checked_event_waits_for_native_envelope_validation(self):
        reports = []
        with patch.object(server.rp.Probe, 'report', lambda *a, **kw: reports.append(kw)):
            server.NativeProbe.report(self.p, True, checked=True)
            self.assertNotIn('checked', reports[-1])
            self.p.native_envelope_checked = True
            server.NativeProbe.report(self.p, True, checked=True, native_envelope_checked=True)
            self.assertTrue(reports[-1]['checked'])
        state = reports[-1]['native_test']
        self.assertEqual(state['command_tracking'], 'direct_constrained_ik')
        self.assertEqual(state['stop_strategy'], 'hold_last_published')
        for name in ('speed_deg_s', 'acceleration_deg_s2', 'braking_deg_s2',
                     'measured_speed_trip_deg_s'):
            self.assertIsNone(state[name])

    def test_removed_motion_options_are_not_silently_accepted(self):
        for option in ('--speed-deg-s', '--acceleration-deg-s2', '--braking-deg-s2'):
            with self.subTest(option=option), patch('sys.stderr'), self.assertRaises(SystemExit):
                server.arguments([option, '30'])

    def test_low_rate_gap_default_and_startup_signal(self):
        args = server.arguments(['--command-hz', '20'])
        self.assertEqual(args.max_cycle_gap_ms, 150.)
        self.assertTrue(args.check)
        self.assertFalse(server.arguments(['--execute']).check)
        self.p.native_active = False
        self.p.request_shutdown()
        self.assertTrue(self.p.exit.is_set())
        self.assertTrue(self.p.shutdown_requested)

    def test_startup_failures_are_structured_before_robot_ownership(self):
        with patch.object(server, '_main', side_effect=RuntimeError('missing compiler')), \
                patch('builtins.print') as output:
            self.assertEqual(server.main([]), 1)
        self.assertIn('"error": "missing compiler"', output.call_args.args[0])
        self.assertTrue(output.call_args.args[0].startswith(server.rp.PREFIX))


if __name__ == '__main__':
    unittest.main()
