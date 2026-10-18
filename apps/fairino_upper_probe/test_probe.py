import json
import math
import threading
import time
import unittest
from types import SimpleNamespace as NS
from pathlib import Path
from unittest.mock import patch

import robot_probe as rp
from probe_core import (ALL_MASK, JOINTS, JOINT_GROUPS, check_hardware, check_packet,
                        fresh_request, plan_step, snapshot, step_duration, StepCancelled)


def hardware():
    pose = [0.0]*14+[.1, -.1, .0, -1.39, 2.59, -1.17]
    return snapshot({"joint_names": JOINTS[:], "hexerrorcode_curerrcode": 0,
        "position": pose[:], "command_position": pose[:], "velocity": [0.0]*20,
        "stateword": [0x2240]*20, "errorcode": [0]*20})


class CoreTests(unittest.TestCase):
    def test_adjustable_speed_changes_duration_not_distance_or_hold_targets(self):
        hw = hardware()
        reference = plan_step(hw, 19, 1)
        for speed in (.1, .3, .5, 1.0):
            plan = plan_step(hw, 19, 1, speed_deg_s=speed)
            self.assertEqual(plan['target'], reference['target'])
            self.assertEqual(plan['speed_deg_s'], speed)
            self.assertAlmostEqual(1.875*.5/plan['duration'], speed)
        self.assertAlmostEqual(step_duration(.1)+.1, 9.475)
        self.assertAlmostEqual(step_duration(1)+.1, 1.0375)

    def test_invalid_speed_rejected_before_motion(self):
        for speed in (0, -.1, .09, float('nan'), float('inf'), True, '0.5', None):
            with self.assertRaises(ValueError):
                plan_step(hardware(), 19, 1, speed_deg_s=speed)

    def test_only_selected_axis_changes_for_all_twenty_joints(self):
        hw = hardware()
        held = hw["position"][:]
        held[0] += .0001
        for joint in range(1, 21):
            plan = plan_step(hw, joint, 1, held)
            self.assertEqual(len(plan["target"]), 20)
            self.assertEqual([i+1 for i, (a, b) in enumerate(zip(plan['start'], plan['target'])) if a != b], [joint])
            self.assertAlmostEqual(math.degrees(plan['target'][joint-1]-plan['start'][joint-1]), .5)
            for i in range(20):
                if i+1 != joint:
                    self.assertEqual(plan['target'][i], held[i])
            self.assertAlmostEqual(1.875*.5/plan['duration'], .5)

    def test_body_can_jog_past_old_three_degree_range(self):
        hw = hardware(); baseline = hw['position'][:]
        for joint in range(15, 21):
            hw['position'][joint-1] += math.radians(25)
            plan = plan_step(hw, joint, 1)
            self.assertAlmostEqual(math.degrees(plan['target'][joint-1]-baseline[joint-1]), 25.5)
        check_hardware(hw, 0, [0, 0], baseline=baseline)

    def test_invalid_joints_and_directions_rejected(self):
        for joint in (0, -1, 21, 15.0, True, '1', None):
            with self.assertRaises(ValueError): plan_step(hardware(), joint, 1)
        for direction in (0, 2, True):
            with self.assertRaises(ValueError): plan_step(hardware(), 19, direction)

    def test_groups_cover_twenty_axes_without_overlap(self):
        self.assertEqual(JOINT_GROUPS['left'], tuple(range(1, 8)))
        self.assertEqual(JOINT_GROUPS['right'], tuple(range(8, 15)))
        self.assertEqual(JOINT_GROUPS['body'], tuple(range(15, 21)))
        self.assertEqual([j for group in JOINT_GROUPS.values() for j in group], list(range(1, 21)))

    def test_arm_jog_allows_active_axis_but_monitors_all_other_hold_targets(self):
        for selected in (1, 7, 8, 14, 19):
            hw = hardware(); held = hw['position'][:]
            hw['position'][selected-1] += math.radians(.5)
            options = dict(selected=selected, held_positions=held, check_speed=False)
            check_hardware(hw, 0, [0, 0], **options)
            for other in (1, 8, 15, 16, 17, 18, 19, 20):
                if other == selected: continue
                hw['position'][other-1] += math.radians(.31)
                with self.assertRaisesRegex(RuntimeError, f'J{other} 偏离保持目标'):
                    check_hardware(hw, 0, [0, 0], **options)
                hw['position'][other-1] = held[other-1]
            held[selected-1] = hw['position'][selected-1]
            check_hardware(hw, 0, [0, 0], held_positions=held)

    def test_initial_enable_is_rejected_but_session_all_enable_allowed(self):
        hw = hardware(); hw['enabled'] = [True]*20
        with self.assertRaisesRegex(RuntimeError, '已有电机'):
            check_hardware(hw, 0, [0, 0])
        check_hardware(hw, 0, [0, 0], allow_enabled=True)

    def test_speed_suspend_does_not_suspend_estop_feedback_fault_or_arm_hold(self):
        hw = hardware(); baseline = hw['position'][:]
        hw['velocity'][18] = math.radians(5)
        options = dict(allow_enabled=True, check_speed=False, baseline=baseline)
        check_hardware(hw, 0, [0, 0], **options)
        for age, gpio in ((.5, [0, 0]), (0, [1, 1]), (0, None)):
            with self.assertRaises(RuntimeError): check_hardware(hw, age, gpio, **options)
        hw['errorcode'][18] = 5
        with self.assertRaisesRegex(RuntimeError, 'J19=5'): check_hardware(hw, 0, [0, 0], **options)
        hw['errorcode'][18] = 0
        hw['position'][0] += math.radians(.31)
        with self.assertRaisesRegex(RuntimeError, 'J1'): check_hardware(hw, 0, [0, 0], **options)

    def test_running_speed_protection_restored_with_measured_value(self):
        hw = hardware(); hw['velocity'][18] = math.radians(2.1)
        with self.assertRaisesRegex(RuntimeError, 'J19 速度 .*2.100.*2.0'):
            check_hardware(hw, 0, [0, 0], selected=19, allow_enabled=True)
        hw['velocity'][18] = math.radians(.6)
        check_hardware(hw, 0, [0, 0], selected=19, allow_enabled=True)
        with self.assertRaisesRegex(RuntimeError, '0.5'):
            check_hardware(hw, 0, [0, 0], allow_enabled=True)

    def test_delayed_command_focus_loss_and_stale_request_stop(self):
        packet = {'ack': 10.0, 'focused': True, 'exit': False}
        check_packet(packet, 10, 10.1)
        for p, now in (({**packet, 'focused': False}, 10.1), (packet, 11)):
            with self.assertRaises(RuntimeError): check_packet(p, 10, now)
        with self.assertRaises(ValueError): fresh_request({'id': 1, 'created': 8}, 10)

    def test_focus_pause_opt_in_preserves_exit_and_communication_guards(self):
        packet = {'ack': 10.0, 'focused': False, 'exit': False}
        check_packet(packet, 10, 10.1, allow_unfocused=True)
        for p, received, now in (({**packet, 'exit': True}, 10, 10.1),
                                  (packet, 9, 10.1), (packet, 11, 11)):
            with self.subTest(packet=p, received=received, now=now), self.assertRaises(RuntimeError):
                check_packet(p, received, now, allow_unfocused=True)

    def test_calibration_reset_and_unspecified_masks_are_not_allowed(self):
        p = rp.Probe.__new__(rp.Probe)
        for command in ('enable()', 'enable(1)', 'disable(262144)', 'zeroing()', 'reseterror()'):
            with self.assertRaisesRegex(RuntimeError, '拒绝'): p.command(command)


class Future:
    def __init__(self, value): self.value = value
    def done(self): return True
    def result(self): return self.value


def fake_probe(probe_type=rp.Probe):
    p = probe_type.__new__(probe_type)
    p.args = NS(check=False, prepare_check=False)
    p.hw = hardware(); p.hw_time = time.monotonic()
    p.baseline = p.hw['position'][:]; p.hold_positions = p.hw['position'][:]
    p.selected = None; p.ready = False
    p.enable_attempted = p.enabling = p.session_enabled = False
    p.enable_deadline = 0
    p.owned = p.loaded = p.active = True
    p.pending_enable_goal = p.pending_enable_result = None
    p.pending_goal = p.goal = None
    p.params_dir = None
    p.phase = 'test'; p.fault = ''; p.exit = threading.Event()
    p.input_lock = threading.Lock(); p.cancel_count = None; p.last_request = 0
    p.packet = {'ack': time.monotonic(), 'focused': True, 'exit': False, 'cancel': 0}
    p.received = time.monotonic()
    p.commands = []; p.events = []; p.reports = []; p.sent = []
    p.emit = lambda **data: p.events.append(data)
    p.report = lambda *a, **data: p.reports.append(data)
    p.gpio = lambda: [0, 0]
    def pump(seconds=.02, guarded=True):
        p.hw_time = time.monotonic(); p.received = p.hw_time; p.packet['ack'] = p.hw_time
        if guarded: p.guard()
    p.pump = pump
    p.wait = lambda future, *a, **k: future.result()
    p.controllers = lambda *a: {rp.CONTROLLER: 'active' if p.active else 'inactive'}
    p.switch = lambda active, *a: setattr(p, 'active', active)
    def command(text, guarded=True):
        p.commands.append(text)
        if text == f'enable({ALL_MASK})': p.hw['enabled'] = [True]*20
        elif text == f'disable({ALL_MASK})': p.hw['enabled'] = [False]*20
        p.hw_time = time.monotonic()
    p.command = command
    p.service = lambda *a, **k: NS(ok=True)
    p.confirm_disabled = lambda: not any(p.hw['enabled'])
    return p


def goal():
    return NS(trajectory=NS(joint_names=[], points=[]), path_tolerance=[],
              goal_tolerance=[], goal_time_tolerance=NS(sec=0))


class FocusHoldTests(unittest.TestCase):
    def test_joint_focus_loss_cancels_motion_without_ending_enabled_session(self):
        p = fake_probe(rp.JointProbe)
        p.session_enabled = p.enable_attempted = True
        p.hw['enabled'] = [True]*20
        p.packet['focused'] = False
        p.cancel_count = 0
        with self.assertRaises(StepCancelled):
            p.guard()
        self.assertTrue(p.session_enabled)
        self.assertEqual(p.commands, [])
        p.cancel_count = None
        p.guard()  # Position hold remains valid after trajectory cancellation.

    def test_focus_loss_alone_does_not_cancel_an_inflight_enable(self):
        p = fake_probe(rp.JointProbe)
        p.packet['focused'] = False
        p.enabling = True
        p.enable_deadline = time.monotonic()+12
        p.cancel_count = 0
        p.guard()
        self.assertEqual(p.commands, [])

    def test_disconnect_still_ends_the_session_while_unfocused(self):
        p = fake_probe(rp.JointProbe)
        p.packet['focused'] = False
        p.received -= 1
        with self.assertRaisesRegex(RuntimeError, '心跳'):
            p.guard()


STUBS = {'FollowJointTrajectory': NS(Goal=goal), 'JointTrajectoryPoint': NS,
         'JointTolerance': NS, 'Duration': lambda seconds: NS(to_msg=lambda: seconds),
         'UnloadController': NS(Request=NS)}


class LifecycleTests(unittest.TestCase):
    def test_initial_target_check_still_rejects_stale_target_before_enable(self):
        p = fake_probe()
        p.hw['position'][14] = math.radians(-7.873)
        p.hw['command_position'][14] = math.radians(-7.812)
        with self.assertRaisesRegex(RuntimeError, r'J15.*目标 -7.8120°.*实测 -7.8730°.*0.0610° > 0.0573°'):
            p.check_initial_target()
        self.assertEqual(p.commands, [])
        p.hw['command_position'] = p.hw['position'][:]
        p.check_initial_target()

    def test_enable_ignores_speed_then_restores_guard_and_all_axis_monitoring(self):
        p = fake_probe()
        original = p.command
        def enable(text, guarded=True):
            self.assertTrue(p.enabling)
            p.hw['velocity'][18] = math.radians(5)
            p.guard()
            original(text, guarded)
            p.hw['velocity'][18] = 0
        p.command = enable
        p.enable_all()
        self.assertEqual(p.commands, [f'enable({ALL_MASK})'])
        self.assertTrue(p.session_enabled)
        self.assertFalse(p.enabling)
        p.hw['velocity'][18] = math.radians(5)
        with self.assertRaisesRegex(RuntimeError, 'J19 速度'): p.guard()
        self.assertIn('fault_snapshot', p.events[-1])
        p.hw['velocity'][18] = 0; p.hw['enabled'][0] = False
        with self.assertRaisesRegex(RuntimeError, '退出使能'): p.guard()

    def test_estop_and_link_failure_during_enable_restore_flag_and_disable_all(self):
        for failure in ('estop', 'link', 'timeout'):
            p = fake_probe()
            original = p.command
            def command(text, guarded=True):
                if text.startswith('enable('):
                    original(text)
                    if failure == 'estop': p.gpio = lambda: [1, 1]
                    elif failure == 'link': p.packet['ack'] -= 1
                    else: p.enable_deadline = time.monotonic()-1
                    p.guard()
                else: original(text, guarded)
            p.command = command
            with self.assertRaises(RuntimeError): p.enable_all()
            self.assertFalse(p.enabling)
            with patch.dict(rp.__dict__, STUBS): self.assertTrue(p.cleanup())
            self.assertEqual(p.commands[-1], f'disable({ALL_MASK})')
            self.assertFalse(any(p.hw['enabled']))

    def exercise_step(self, fail=False, speed=.5, joint=19):
        p = fake_probe(); p.enable_all()
        def send(g):
            self.assertEqual(g.trajectory.joint_names, JOINTS)
            self.assertTrue(all(p.hw['enabled']))
            self.assertAlmostEqual(g.trajectory.points[1].time_from_start-
                                   g.trajectory.points[0].time_from_start, step_duration(speed))
            p.sent.append(g)
            if not fail:
                p.hw['position'] = g.trajectory.points[-1].positions[:]
                p.hw['command_position'] = p.hw['position'][:]
            return Future(NS(accepted=True, get_result_async=lambda: Future(NS(
                status=6 if fail else 4, result=NS(error_code=-4 if fail else 0, error_string='test fault'))),
                cancel_goal_async=lambda: Future(NS(goals_canceling=[1]))))
        p.trajectory_client = NS(send_goal_async=send)
        with patch.dict(rp.__dict__, STUBS):
            if fail:
                with self.assertRaisesRegex(RuntimeError, '轨迹执行失败'):
                    p.perform_step(plan_step(p.hw, joint, 1, p.hold_positions, speed_deg_s=speed))
                self.assertTrue(p.cleanup())
            else:
                for _ in range(8):
                    p.perform_step(plan_step(p.hw, joint, 1, p.hold_positions, speed_deg_s=speed))
                    p.guard()
                    p.selected = None
                    p.guard()
                self.assertEqual(p.commands, [f'enable({ALL_MASK})'])
                self.assertTrue(p.active)
                self.assertTrue(all(p.hw['enabled']))
                self.assertAlmostEqual(math.degrees(p.hw['position'][joint-1]-p.baseline[joint-1]), 4)
                for i in range(20):
                    if i != joint-1: self.assertEqual(p.hw['position'][i], p.baseline[i])
                self.assertTrue(p.cleanup())
        return p

    def test_worker_replanning_keeps_requested_speed_and_logs_it(self):
        for speed in (.1, 1.0):
            p = self.exercise_step(speed=speed)
            steps = [e['step'] for e in p.events if 'step' in e]
            self.assertEqual(len(steps), 8)
            self.assertTrue(all(s['speed_deg_s'] == speed for s in steps))

    def test_repeated_steps_keep_all_enabled_and_hold_other_nineteen_axes(self):
        p = self.exercise_step()
        self.assertEqual(p.commands, [f'enable({ALL_MASK})', f'disable({ALL_MASK})'])
        self.assertFalse(any(p.hw['enabled']))
        self.assertFalse(p.active)

    def test_repeated_left_and_right_arm_steps_preserve_entire_body_pose(self):
        for joint in (1, 7, 8, 14):
            p = self.exercise_step(joint=joint)
            for trajectory in p.sent:
                for point in trajectory.trajectory.points:
                    self.assertEqual(point.positions[14:], p.baseline[14:])

    def test_guard_checks_body_drift_even_while_arm_is_selected(self):
        p = fake_probe(); p.enable_all(); p.selected = 1
        p.hw['position'][0] += math.radians(.5)
        p.guard()
        p.hw['position'][18] += math.radians(.31)
        with self.assertRaisesRegex(RuntimeError, 'J19 偏离保持目标'): p.guard()
        self.assertIn('fault_snapshot', p.events[-1])
        with patch.dict(rp.__dict__, STUBS): self.assertTrue(p.cleanup())
        self.assertEqual(p.commands[-1], f'disable({ALL_MASK})')

    def test_trajectory_fault_disables_all_twenty_axes(self):
        p = self.exercise_step(fail=True)
        self.assertEqual(p.commands[-1], f'disable({ALL_MASK})')
        self.assertFalse(any(p.hw['enabled']))

    def test_space_cancels_trajectory_but_keeps_all_axes_enabled(self):
        p = fake_probe(); p.enable_all()
        p.goal = NS(accepted=True, get_result_async=lambda: Future(NS(status=5)),
                    cancel_goal_async=lambda: Future(NS(goals_canceling=[1])))
        p.cancel_step()
        self.assertTrue(all(p.hw['enabled']))
        self.assertTrue(p.active)
        self.assertEqual(p.commands, [f'enable({ALL_MASK})'])
        self.assertEqual(p.hold_positions, p.hw['command_position'])

    def test_late_enable_completion_is_followed_by_another_disable(self):
        p = fake_probe(); p.enable_attempted = True
        p.pending_enable_goal = Future(NS(accepted=True))
        def settle():
            p.commands.append('late-enable-completed')
            p.hw['enabled'] = [True]*20
            p.pending_enable_goal = None
        p.settle_enable_request = settle
        with patch.dict(rp.__dict__, STUBS): self.assertTrue(p.cleanup())
        self.assertEqual(p.commands, [f'disable({ALL_MASK})', 'late-enable-completed', f'disable({ALL_MASK})'])
        self.assertFalse(any(p.hw['enabled']))

    def test_unknown_enable_completion_never_claims_disabled(self):
        p = fake_probe(); p.enable_attempted = True; p.pending_enable_goal = Future(None)
        def settle(): raise RuntimeError('request timeout')
        p.settle_enable_request = settle
        with patch.dict(rp.__dict__, STUBS): self.assertFalse(p.cleanup())
        self.assertFalse(p.reports[-1]['disabled_confirmed'])
        self.assertTrue(p.active)

    def test_check_and_prepare_check_never_enable_or_send_a_trajectory(self):
        for mode in ('check', 'prepare_check'):
            p = fake_probe(); setattr(p.args, mode, True)
            p.owned = p.loaded = p.active = False
            p.controllers = lambda *a: {}
            p.trajectory_client = NS(server_is_ready=lambda: True)
            captured = []
            def service(typ, path, request, guarded=True):
                if path.endswith('/set_parameters'):
                    params = {x.name: x.value for x in request.parameters}
                    config = json.loads(Path(params[rp.CONTROLLER+'.params_file'][0]).read_text())
                    captured.append(config[rp.CONTROLLER]['ros__parameters'])
                    return NS(results=[NS(successful=True), NS(successful=True)])
                return NS(ok=True)
            p.service = service
            stubs = {**STUBS, 'yaml': NS(safe_dump=json.dumps),
                'SetParameters': NS(Request=NS), 'LoadController': NS(Request=NS),
                'ConfigureController': NS(Request=NS),
                'Parameter': lambda name, value: NS(to_parameter_msg=lambda: NS(name=name, value=value))}
            with patch.dict(rp.__dict__, stubs):
                p.prepare()
                self.assertTrue(p.reports[-1]['checked'])
                self.assertEqual(p.commands, [])
                self.assertFalse(any(p.hw['enabled']))
                self.assertTrue(p.cleanup())
            if mode == 'prepare_check':
                self.assertEqual(len(captured), 1)
                self.assertEqual(captured[0]['joints'], JOINTS)
                self.assertIs(captured[0]['set_last_command_interface_value_as_state_on_activation'], False)
                self.assertIs(captured[0]['open_loop_control'], False)
            else:
                self.assertEqual(captured, [])


if __name__ == '__main__':
    unittest.main()
