"""Preset lift in the real 20-axis receiver loop; only clock/ROS I/O are mocked."""
import copy
import math
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import paths
import robot_probe as rp
from body_core import load_config
from tcp_core import MODEL_ID, tcp_forward_matrix
from test_tcp import hardware, probe, STUBS
from vr_core import KIND, LIFT_MODE, validate
from vr_lift import LiftInput
from vr_lift_presets import PresetLiftPlanner
from vr_robot import VRProbe


class EndSimulation(Exception):
    pass


def packet(axis=0., sequence=1, stamp=10., arms=None, epoch=1, gesture=1):
    return dict(kind=KIND, model=MODEL_ID, id=sequence, created=stamp,
                arms={} if arms is None else arms, lift_mode=LIFT_MODE, lift=axis,
                lift_epoch=epoch, lift_gesture=gesture)


class ProtocolTests(unittest.TestCase):
    def test_neutral_preset_heartbeat_and_scalar_directions_are_valid(self):
        for axis in (0., -.2, .2, -1., 1.):
            self.assertEqual(validate(hardware(), packet(axis), 10.), {})

    def test_old_jog_protocol_unknown_mode_and_invalid_heartbeat_are_rejected(self):
        legacy = packet(1.)
        del legacy['lift_mode']
        with self.assertRaisesRegex(ValueError, '旧版身体点动'):
            validate(hardware(), legacy, 10.)
        for change in ({'lift_mode': 'jog'}, {'lift_mode': None}, {'lift': True},
                       {'lift': float('nan')}, {'lift': [0.]*20}, {'lift': 1.1},
                       {'arms': {'body': {}}}, {'created': 9.7}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate(hardware(), {**packet(), **change}, 10.)
        missing = packet()
        del missing['lift']
        with self.assertRaisesRegex(ValueError, '必须包含标量'):
            validate(hardware(), missing, 10.)

    def test_preset_requires_positive_integer_epoch(self):
        for epoch in (None, 0, -1, True, 1., '1'):
            with self.subTest(epoch=epoch), self.assertRaisesRegex(ValueError, 'lift_epoch'):
                validate(hardware(), packet(epoch=epoch), 10.)
        missing = packet()
        del missing['lift_epoch']
        with self.assertRaisesRegex(ValueError, 'lift_epoch'):
            validate(hardware(), missing, 10.)

    def test_optional_gesture_id_is_strict_when_present(self):
        for gesture in (None, 0, -1, True, 1., '1'):
            with self.subTest(gesture=gesture), self.assertRaisesRegex(ValueError, 'lift_gesture'):
                validate(hardware(), packet(gesture=gesture), 10.)
        older_packet = packet()
        del older_packet['lift_gesture']
        self.assertEqual(validate(hardware(), older_packet, 10.), {})


class ReceiverTests(unittest.TestCase):
    def simulate(self, schedule, duration=12., enabled=True, hook=None, start_height=25., receiver_delay=None,
                 report_hook=None):
        p = probe()
        p.__class__ = VRProbe
        model = load_config()
        q = model.pose(dict(height=start_height, forward=0., roll=0., yaw=0.), p.hw['position'])
        p.hw['position'] = q[:]
        p.hw['command_position'] = q[:]
        p.hold_positions = p.baseline = q[:]
        p.lift = PresetLiftPlanner(model.config) if enabled else None
        p.args.command_hz, p.args.receiver_hz, p.args.feedback_hz = 10., 200., 100.
        clock, ticks, messages, topics, states = [10.], [0], [], [], []
        original_report = p.report
        def report(*args, **fields):
            original_report(*args, **fields)
            if report_hook:
                report_hook(clock[0]-10., p)
        p.report = report

        def publish(message):
            messages.append(dict(stamp=clock[0], message=copy.deepcopy(message),
                                 moving_joints=p.moving_joints[:],
                                 request_id=(p.packet.get('request') or {}).get('id')))

        def publisher(kind, topic, depth):
            topics.append(topic)
            return NS(get_subscription_count=lambda:1, publish=publish)

        p.node = NS(create_publisher=publisher)

        def prepare():
            p.hw_time = p.received = p.packet['ack'] = clock[0]
            rp.Probe.enable_all(p)
            p.ready = True

        def feedback(seconds=.005, guarded=True):
            clock[0] += seconds
            if messages:
                target = messages[-1]['message'].points[0].positions[:]
                p.hw['velocity'] = [(b-a)/seconds for a, b in zip(p.hw['position'], target)]
                p.hw['position'] = target[:]
                p.hw['command_position'] = target[:]
            p.hw_time = p.received = p.packet['ack'] = clock[0]
            if hook:
                hook(clock[0]-10., p)
            if guarded:
                p.guard()

        def next_input(seconds):
            delay = receiver_delay(clock[0]-10.) if receiver_delay else 0.
            feedback(seconds+delay)
            elapsed = clock[0]-10.
            states.append(dict(time=elapsed, height=model.state(p.hw['position'])['height'],
                               moving=bool(p.lift and p.lift.moving),
                               lift_epoch=getattr(p, 'last_lift_epoch', 0),
                               armed=bool(p.lift and p.lift.snapshot(clock[0])['armed'])))
            if elapsed >= duration:
                raise EndSimulation()
            ticks[0] += 1
            request = schedule(elapsed, ticks[0], p)
            if request is not None:
                request = copy.deepcopy(request)
                request['_wrist_guard'] = p.wrist.active.copy()
            p.packet['request'] = request

        p.prepare, p.pump, p.pump_input = prepare, feedback, next_input
        stubs = {**STUBS, 'JointTrajectory':lambda:NS(joint_names=[], points=[], header=NS(stamp=None))}
        error = None
        with patch.dict(rp.__dict__, stubs), patch('vr_robot.time.monotonic', lambda:clock[0]):
            try:
                p.run()
            except EndSimulation:
                pass
            except RuntimeError as exc:
                error = exc
        return NS(probe=p, model=model, messages=messages, topics=topics, states=states, error=error)

    @staticmethod
    def regular(axis, arms=None):
        """Only ten new requests per second; the receiver still runs at 200 Hz."""
        def schedule(t, tick, p):
            if tick % 20 != 1:
                return p.packet.get('request')
            return packet(axis(t), sequence=tick, stamp=10.+t,
                          arms=arms(t, p) if arms else {})
        return schedule

    @staticmethod
    def upward(t):
        return 0. if t < .1 or t >= 1.3 else 1.

    def test_neutral_heartbeat_is_read_only_until_a_complete_gesture(self):
        for axis in (lambda t:0., lambda t:1., lambda t:1. if .1 <= t < .9 else 0.):
            result = self.simulate(self.regular(axis), duration=2.)
            self.assertIsNone(result.error)
            self.assertEqual(result.messages, [])
            self.assertAlmostEqual(result.states[-1]['height'], 25.)
            self.assertIsNone(result.probe.path)

    def test_neutral_after_trigger_completes_high_target_at_receiver_rate(self):
        result = self.simulate(self.regular(self.upward))
        self.assertIsNone(result.error)
        self.assertAlmostEqual(result.states[-1]['height'], 100., places=6)
        self.assertEqual(result.topics, [f'/{rp.CONTROLLER}/joint_trajectory'])
        self.assertGreater(len(result.messages), result.probe.accepted_rate.total*3)
        repeated_ids = [a['request_id'] == b['request_id'] and
                        a['message'].points[0].positions[17:20] != b['message'].points[0].positions[17:20]
                        for a, b in zip(result.messages, result.messages[1:])]
        self.assertTrue(any(repeated_ids), 'Body sampling must not wait for a new controller packet.')
        self.assertTrue(any('body_lift_stop_confirmation' in e for e in result.probe.events))
        self.assertTrue(any('stop_confirmation' in e for e in result.probe.events))
        self.assertIsNone(result.probe.path)
        self.assertFalse(result.probe.lift.moving)
        self.assertFalse(any('jog_rejected' in e for e in result.probe.events))
        for record in result.messages:
            message = record['message']
            self.assertEqual(message.joint_names, rp.JOINTS)
            self.assertEqual(len(message.points), 1)
            self.assertEqual(len(message.points[0].positions), 20)
            self.assertEqual(message.points[0].positions[:17], result.probe.baseline[:17])

    def test_gesture_change_detects_overwritten_center_during_candidate(self):
        def schedule(t, tick, p):
            # Raw SDK samples briefly centered at t=.8, but only the latest
            # upward sample survives. Its changed gesture ID carries the break.
            return packet(1. if .1 <= t < 1.6 else 0., tick, 10.+t,
                          gesture=1 if t < .8 else 2)
        result = self.simulate(schedule, duration=2.)
        self.assertIsNone(result.error)
        self.assertEqual(result.messages, [])
        self.assertAlmostEqual(result.states[-1]['height'], 25.)

    def test_gesture_change_does_not_cancel_an_already_triggered_plan(self):
        def schedule(t, tick, p):
            return packet(self.upward(t), tick, 10.+t, gesture=1 if t < 2. else 2)
        result = self.simulate(schedule)
        self.assertIsNone(result.error)
        self.assertAlmostEqual(result.states[-1]['height'], 100., places=6)

    def test_downward_gesture_stops_one_third_above_former_lower_preset(self):
        axis = lambda t: -1. if .1 <= t < 1.3 else 0.
        result = self.simulate(self.regular(axis), start_height=100.)
        self.assertIsNone(result.error)
        self.assertAlmostEqual(result.states[-1]['height'], 200./3., places=6)
        self.assertGreaterEqual(min(row['height'] for row in result.states), 200./3.-1e-7)

    def test_arms_and_lift_share_complete_targets_and_release_still_confirms_hold(self):
        def arm(t, p):
            if t > 9.:
                return {}
            q = p.baseline[:7]
            q[0] += math.radians(.02*math.sin(t))
            matrix = tcp_forward_matrix(q)
            return {'left':dict(reference=q[:], target=q[:], tcp=[v for row in matrix for v in row])}
        result = self.simulate(self.regular(self.upward, arm))
        self.assertIsNone(result.error)
        self.assertAlmostEqual(result.states[-1]['height'], 100., places=6)
        targets = [row['message'].points[0].positions for row in result.messages]
        self.assertGreater(max(q[0] for q in targets)-min(q[0] for q in targets), 1e-4)
        self.assertTrue(all(q[7:17] == result.probe.baseline[7:17] for q in targets))
        self.assertIsNone(result.probe.path)
        self.assertTrue(any('stop_confirmation' in e for e in result.probe.events))

    def test_arm_release_preserves_fresh_untriggered_body_gesture(self):
        def arm(t, p):
            if t >= .4:
                return {}
            q = p.baseline[:7]
            matrix = tcp_forward_matrix(q)
            return {'left':dict(reference=q[:], target=q[:], tcp=[v for row in matrix for v in row])}
        # Model the real client's ready gate and session epoch, using its real
        # LiftInput. A transient ready=False report resets the input gate and
        # would destroy the candidate even if the server preserved it locally.
        input_gate = LiftInput()
        ready, was_active, epoch = [False], [False], [0]
        not_ready_during_release = []
        def on_report(t, p):
            ready[0] = p.ready
            if not p.ready:
                input_gate.reset()
                was_active[0] = False
                if .3 <= t < 1.6:
                    not_ready_during_release.append(t)
        def schedule(t, tick, p):
            if tick % 20 != 1:
                return p.packet.get('request')
            frame = {'right':dict(axis_y=1. if .3 <= t < 1.6 else 0.,
                                   joystick_available=True, axis_click=False)}
            value = input_gate.value(frame, ready[0])
            if ready[0] and not was_active[0]:
                epoch[0] += 1
            was_active[0] = ready[0]
            return (packet(value, tick, 10.+t, arms=arm(t, p), epoch=epoch[0],
                           gesture=input_gate.gesture_id) if ready[0] else None)
        result = self.simulate(schedule, report_hook=on_report)
        self.assertIsNone(result.error)
        self.assertAlmostEqual(result.states[-1]['height'], 100., places=6)
        first_motion = next(row['time'] for row in result.states if row['moving'])
        self.assertGreaterEqual(first_motion, 1.3)
        self.assertLess(first_motion, 1.6)
        self.assertGreaterEqual(sum('stop_confirmation' in e for e in result.probe.events), 2)
        self.assertEqual(not_ready_during_release, [])
        self.assertEqual(epoch[0], 1)

    def test_pause_cancels_without_resuming_from_a_still_held_stick(self):
        regular = self.regular(self.upward)
        def schedule(t, tick, p):
            if t < 2.:
                return regular(t, tick, p)
            if t < 2.3:
                return None  # Same request=None contract as pause/Home.
            return packet(1., tick, 10.+t)
        result = self.simulate(schedule, duration=4.)
        self.assertIsNone(result.error)
        stopped = [r['height'] for r in result.states if r['time'] >= 2.4]
        self.assertGreater(stopped[-1], 25.)
        self.assertLess(stopped[-1], 100.)
        self.assertLess(max(stopped)-min(stopped), 1e-8)
        self.assertIsNone(result.probe.path)
        self.assertFalse(result.probe.lift.moving)

    def test_duplicate_id_cannot_extend_lease_or_resume_after_expiry(self):
        regular = self.regular(self.upward)
        def schedule(t, tick, p):
            if t < 2.:
                return regular(t, tick, p)
            if t < 2.7:
                # Malicious/replayed duplicate with a changed creation stamp.
                old = p.packet['request']
                return {**old, 'created':10.+t}
            # Fresh IDs remain blocked until an explicit None pause/rearm.
            return packet(1., tick, 10.+t)
        result = self.simulate(schedule, duration=4.)
        self.assertIsNone(result.error)
        self.assertTrue(any('超过 0.25' in e.get('jog_rejected', '') for e in result.probe.events))
        stopped = [r['height'] for r in result.states if r['time'] > 2.5]
        self.assertLess(max(stopped)-min(stopped), 1e-8)
        self.assertIsNone(result.probe.path)

    def test_body_mode_requires_enabled_session_even_for_neutral(self):
        result = self.simulate(self.regular(lambda t:0.), enabled=False, duration=.5)
        self.assertIsNone(result.error)
        self.assertEqual(result.messages, [])
        self.assertTrue(any('未启用身体升降' in e.get('jog_rejected', '') for e in result.probe.events))

    def test_new_epoch_neutral_cancels_even_when_none_pause_was_overwritten(self):
        regular = self.regular(self.upward)
        def schedule(t, tick, p):
            if t < 2.:
                return regular(t, tick, p)
            # The None pause never reaches the receiver's latest-packet slot.
            return packet(0., tick, 10.+t, epoch=2)
        result = self.simulate(schedule, duration=4.)
        self.assertIsNone(result.error)
        stopped = [r for r in result.states if r['time'] >= 2.3]
        self.assertGreater(stopped[-1]['height'], 25.)
        self.assertLess(stopped[-1]['height'], 100.)
        self.assertLess(max(r['height'] for r in stopped)-min(r['height'] for r in stopped), 1e-8)
        self.assertTrue(all(r['lift_epoch'] == 2 and r['armed'] and not r['moving'] for r in stopped))
        self.assertTrue(any('body_lift_stop_confirmation' in e for e in result.probe.events))
        self.assertIsNone(result.probe.path)
        self.assertEqual(result.probe.last_lift_epoch, 2)

    def test_new_epoch_held_stick_cancels_but_cannot_trigger_without_neutral(self):
        regular = self.regular(self.upward)
        def schedule(t, tick, p):
            return regular(t, tick, p) if t < 2. else packet(1., tick, 10.+t, epoch=2)
        result = self.simulate(schedule, duration=4.)
        self.assertIsNone(result.error)
        stopped = [r for r in result.states if r['time'] >= 2.3]
        self.assertLess(max(r['height'] for r in stopped)-min(r['height'] for r in stopped), 1e-8)
        self.assertTrue(all(r['lift_epoch'] == 2 and not r['armed'] and not r['moving'] for r in stopped))

    def test_old_epoch_is_rejected_and_none_does_not_roll_epoch_back(self):
        regular = self.regular(self.upward)
        def schedule(t, tick, p):
            if t < 2.:
                return regular(t, tick, p)
            if t < 2.5:
                return packet(0., tick, 10.+t, epoch=2)
            if 3. <= t < 3.2:
                return None
            return packet(1., tick, 10.+t, epoch=1)
        result = self.simulate(schedule, duration=4.)
        self.assertIsNone(result.error)
        self.assertEqual(result.probe.last_lift_epoch, 2)
        self.assertGreaterEqual(sum('旧身体升降会话' in e.get('jog_rejected', '')
                                    for e in result.probe.events), 2)
        stopped = [r['height'] for r in result.states if r['time'] >= 2.3]
        self.assertLess(max(stopped)-min(stopped), 1e-8)
        self.assertIsNone(result.probe.path)

    def test_blocked_recovery_requires_fully_valid_higher_epoch_neutral(self):
        regular = self.regular(self.upward)
        def schedule(t, tick, p):
            if t < 2.:
                return regular(t, tick, p)
            if t < 2.5:
                return p.packet['request']  # Expire without observing None.
            if t < 2.6:
                return packet(1., tick, 10.+t, epoch=2)  # Cannot unblock by motion.
            if t < 2.7:
                return {**packet(0., tick, 10.+t, epoch=2), 'model':'invalid'}
            if t < 3.:
                return packet(0., tick, 10.+t, epoch=2)
            return packet(1. if t < 4.2 else 0., tick, 10.+t, epoch=2)
        result = self.simulate(schedule)
        self.assertIsNone(result.error)
        self.assertTrue(any('超过 0.25' in e.get('jog_rejected', '') for e in result.probe.events))
        self.assertTrue(any('模型不匹配' in e.get('jog_rejected', '') for e in result.probe.events))
        self.assertTrue(all(r['lift_epoch'] == 1 for r in result.states if 2.5 <= r['time'] < 2.7))
        stopped = [r['height'] for r in result.states if 2.5 <= r['time'] < 3.9]
        self.assertLess(max(stopped)-min(stopped), 1e-8)
        self.assertEqual(result.probe.last_lift_epoch, 2)
        self.assertAlmostEqual(result.states[-1]['height'], 100., places=6)
        self.assertTrue(any(r['moving'] and r['time'] > 4. for r in result.states))

    def test_hardware_fault_clears_running_plan(self):
        def hook(t, p):
            if t >= 2.:
                p.hw['errorcode'][18] = 30
        result = self.simulate(self.regular(self.upward), duration=4., hook=hook)
        self.assertIsNotNone(result.error)
        self.assertIn('J19=30', str(result.error))
        self.assertTrue(any(r['moving'] for r in result.states))
        self.assertFalse(result.probe.lift.moving)

    def test_fresh_request_after_receiver_stall_holds_instead_of_skipping_path(self):
        delayed = [False]
        def delay(t):
            if t >= 2. and not delayed[0]:
                delayed[0] = True
                return .6
            return 0.
        def schedule(t, tick, p):
            # Every packet is genuinely fresh, including the first after the
            # receiver's .6-second stall: request LEASE alone cannot catch this.
            return packet(self.upward(t), tick, 10.+t)
        result = self.simulate(schedule, duration=4., receiver_delay=delay)
        self.assertIsNone(result.error)
        self.assertTrue(any('采样间隔超过 50 ms' in e.get('jog_rejected', '') for e in result.probe.events))
        self.assertFalse(any('超过 0.25' in e.get('jog_rejected', '') for e in result.probe.events))
        before = [m for m in result.messages if m['stamp'] < 12.5][-1]
        after = [m for m in result.messages if m['stamp'] >= 12.5][0]
        self.assertEqual(before['message'].points[0].positions, after['message'].points[0].positions)
        stopped = [r for r in result.states if r['time'] >= 2.8]
        self.assertLess(max(r['height'] for r in stopped)-min(r['height'] for r in stopped), 1e-8)
        self.assertTrue(all(not r['moving'] and not r['armed'] for r in stopped))
        self.assertIsNone(result.probe.path)
        self.assertIsNone(result.probe.last_lift_sample_time)


class BodyStopTests(unittest.TestCase):
    def make_probe(self):
        p = probe()
        p.__class__ = VRProbe
        p.enable_attempted = p.session_enabled = True
        p.hw['enabled'] = [True]*20
        p.path = [(0., p.hw['command_position'][:])]
        p.moving_joints = list(range(1, 15))+[18, 19, 20]
        p.lift_stop_target = p.hw['command_position'][17:20]
        p.lift_stop_published, p.lift_stop_deadline = 10., 13.
        return p

    def test_completion_waits_for_new_fold_feedback_but_does_not_stop_arms(self):
        p = self.make_probe()
        p.hw_time = p.received = p.packet['ack'] = 10.
        p.hw['velocity'][0] = .3  # Arms may still move during body completion.
        with patch('vr_robot.time.monotonic', return_value=10.):
            p.guard()
        self.assertIsNotNone(p.lift_stop_deadline)
        p.hw_time = p.received = p.packet['ack'] = 10.1
        with patch('vr_robot.time.monotonic', return_value=10.1):
            p.guard()
        self.assertIsNone(p.lift_stop_deadline)
        self.assertEqual(p.moving_joints, list(range(1, 15)))
        self.assertEqual(p.events[-1]['body_lift_stop_confirmation']['result'], 'confirmed')

    def test_unacknowledged_final_fold_target_cannot_report_completion(self):
        p = self.make_probe()
        p.lift_stop_target = [q+.001 for q in p.lift_stop_target]
        for now in (10.1, 13.1):
            p.hw_time = p.received = p.packet['ack'] = now
            with patch('vr_robot.time.monotonic', return_value=now):
                if now < 13.:
                    p.guard()
                    self.assertIsNotNone(p.lift_stop_deadline)
                else:
                    with self.assertRaisesRegex(RuntimeError, '预设轨迹结束后未确认停止'):
                        p.guard()

    def test_failed_preserved_arm_hold_revokes_ready_and_clears_gesture(self):
        p = self.make_probe()
        p.lift_stop_deadline = None
        p.lift = PresetLiftPlanner(load_config().config)
        q = p.lift.model.pose(dict(height=25., forward=0., roll=0., yaw=0.), p.hw['position'])
        p.hw['position'] = p.hw['command_position'] = q[:]
        p.hold_positions = q[:]
        p.path = [(0., q[:])]
        p.lift.observe(0., q, 10.)
        p.lift.observe(1., q, 10.1)
        p.publisher = NS(publish=lambda message:None)
        readiness = []
        p.report = lambda *args, **fields:readiness.append(p.ready)
        p.confirm_hold = lambda *args:(_ for _ in ()).throw(RuntimeError('hold failed'))
        stubs = {**STUBS, 'JointTrajectory':lambda:NS(joint_names=[], points=[], header=NS(stamp=None))}
        with patch.dict(rp.__dict__, stubs), patch('vr_robot.time.monotonic', return_value=10.2):
            with self.assertRaisesRegex(RuntimeError, 'hold failed'):
                p.stop_stream('普通握把释放', preserve_lift_gesture=True)
        self.assertTrue(readiness[0])
        self.assertFalse(readiness[-1])
        self.assertFalse(p.ready)
        self.assertFalse(p.lift.snapshot(10.2)['armed'])


if __name__ == '__main__':
    unittest.main()
