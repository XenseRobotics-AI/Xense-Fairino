"""Absolute joint goals: bounded trajectory, server validation and cancellation."""
import math
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

import robot_probe as rp
from probe_core import ALL_MASK, JOINTS, StepCancelled, plan_request, plan_target
from test_probe import hardware, fake_probe, Future, STUBS


class TargetPlanningTests(unittest.TestCase):
    def test_absolute_goals_keep_all_other_axes_and_peak_speed_bounded(self):
        hw = hardware(); held = hw['position'][:]
        held[0] += .0001
        for joint in range(1, 21):
            for delta in (-90., -10., -.01, 0., .01, 10., 90.):
                for speed in (.1, 1., 3., 12., 90.):
                    goal = math.degrees(hw['position'][joint-1]) + delta
                    plan = plan_target(hw, joint, goal, held, speed)
                    self.assertAlmostEqual(math.degrees(plan['target'][joint-1]), goal)
                    start = plan['start'][joint-1]; end = plan['target'][joint-1]
                    previous = start
                    # Sample both position and the analytic derivative of the
                    # controller's zero-velocity/acceleration quintic segment.
                    for k in range(101):
                        s = k/100
                        q = start + (end-start)*(10*s**3-15*s**4+6*s**5)
                        v = math.degrees(end-start)*30*s*s*(1-s)**2/plan['duration']
                        self.assertLessEqual(abs(v), speed+1e-9)
                        self.assertGreaterEqual(q, min(start, end)-1e-10)
                        self.assertLessEqual(q, max(start, end)+1e-10)
                        self.assertGreaterEqual((q-previous)*delta, -1e-9)
                        previous = q
                    for i in range(20):
                        if i != joint-1:
                            self.assertEqual(plan['start'][i], held[i])
                            self.assertEqual(plan['target'][i], held[i])

    def test_worker_rejects_invalid_target_speed_joint_and_request_kind(self):
        valid = {'kind': 'target', 'joint': 16, 'target_deg': -45., 'speed_deg_s': 3.}
        invalid = ([{'target_deg': v} for v in (None, '45', True, math.nan, math.inf, -math.inf, 1e308)]
                   + [{'speed_deg_s': v} for v in (0, .09, True, math.nan, math.inf, -math.inf)]
                   + [{'joint': v} for v in (0, 21, True, 16., '16')]
                   + [{'kind': 'unknown'}])
        for change in invalid:
            with self.subTest(change=change), self.assertRaises(ValueError):
                plan_request(hardware(), {**valid, **change})
        # Client-supplied durations cannot override the worker's peak limit.
        plan = plan_request(hardware(), {**valid, 'duration': .001})
        self.assertGreater(plan['duration'], 1.)

    def test_original_jog_request_remains_relative(self):
        hw = hardware()
        p = plan_request(hw, {'joint': 16, 'direction': -1, 'speed_deg_s': .5})
        self.assertAlmostEqual(math.degrees(p['target'][15]-hw['position'][15]), -.5)

    def test_speed_above_old_caps_is_used_for_both_motion_types(self):
        for speed in (3.01, 12., 99., 1000.):
            for request in ({'joint': 15, 'direction': 1},
                            {'kind': 'target', 'joint': 15, 'target_deg': 90.}):
                plan = plan_request(hardware(), {**request, 'speed_deg_s': speed})
                self.assertEqual(plan['speed_deg_s'], speed)
                self.assertGreater(plan['duration'], 0.)
                peak = 1.875*abs(math.degrees(plan['target'][14]-plan['start'][14]))/plan['duration']
                self.assertLessEqual(peak, speed+1e-9)
                if plan.get('kind') != 'target': self.assertAlmostEqual(peak, speed)
        with self.assertRaisesRegex(ValueError, '纳秒分辨率'):
            plan_request(hardware(), {'joint': 15, 'direction': 1, 'speed_deg_s': 1e308})


class TargetExecutionTests(unittest.TestCase):
    def execute(self, cancel=False, failure=False):
        p = fake_probe(); p.enable_all()
        plan = plan_target(p.hw, 16, 20., p.hold_positions)
        # Simulate a fresher start arriving between planning and execution.
        p.hw['position'][15] += math.radians(.1)
        p.hold_positions[15] = p.hw['command_position'][15] = p.hw['position'][15]
        initial = p.hw['position'][:]
        def send(goal):
            p.sent.append(goal)
            self.assertEqual(goal.trajectory.joint_names, JOINTS)
            points = goal.trajectory.points
            self.assertEqual(points[0].positions, initial)
            self.assertAlmostEqual(math.degrees(points[1].positions[15]), 20.)
            for point in points:
                self.assertEqual(point.velocities, [0.]*20)
                self.assertEqual(point.accelerations, [0.]*20)
            duration = points[1].time_from_start-points[0].time_from_start
            peak = 1.875*abs(20.-math.degrees(initial[15]))/duration
            self.assertLessEqual(peak, 3.+1e-9)
            p.hw['velocity'][15] = math.radians(2.9)
            p.guard()  # The previous 2 deg/s guard must not abort a valid target.
            p.hw['velocity'][15] = math.radians(3.1)
            with self.assertRaisesRegex(RuntimeError, 'J16 速度 .*3.0°/s'): p.guard()
            p.hw['velocity'][15] = 0.
            if not cancel and not failure:
                p.hw['position'] = points[-1].positions[:]
                p.hw['command_position'] = p.hw['position'][:]
            return Future(NS(accepted=True,
                get_result_async=lambda: Future(NS(status=5 if cancel else 6 if failure else 4,
                    result=NS(error_code=-4 if failure else 0, error_string='test fault'))),
                cancel_goal_async=lambda: Future(NS(goals_canceling=[1]))))
        p.trajectory_client = NS(send_goal_async=send)
        with patch.dict(rp.__dict__, STUBS):
            if cancel:
                original = p.wait
                count = 0
                def wait(future, *args, **kwargs):
                    nonlocal count
                    count += 1
                    if count == 2: raise StepCancelled('space')
                    return original(future, *args, **kwargs)
                p.wait = wait
                with self.assertRaises(StepCancelled): p.perform_step(plan)
                p.cancel_step()
                self.assertEqual(p.hold_positions, initial)
            elif failure:
                with self.assertRaisesRegex(RuntimeError, '轨迹执行失败'): p.perform_step(plan)
                self.assertTrue(p.cleanup())
                self.assertFalse(any(p.hw['enabled']))
            else:
                p.perform_step(plan)
                self.assertAlmostEqual(math.degrees(p.hold_positions[15]), 20.)
            if not failure:
                self.assertTrue(all(p.hw['enabled']))
                self.assertEqual(p.commands, [f'enable({ALL_MASK})'])
            for i in range(20):
                if i != 15: self.assertEqual(p.hold_positions[i], initial[i])
        return p

    def test_absolute_goal_survives_replanning_and_holds_other_axes(self): self.execute()
    def test_space_cancels_absolute_goal_without_disabling(self): self.execute(cancel=True)
    def test_absolute_trajectory_failure_disables_all_axes(self): self.execute(failure=True)


class JointAdjustmentTests(unittest.TestCase):
    def test_manual_goal_continues_past_recorded_tracking_error_and_old_speed_trips(self):
        p = fake_probe(rp.JointProbe); p.enable_all()
        plan = plan_target(p.hw, 15, 50., p.hold_positions, speed_deg_s=12.)
        def send(goal):
            p.sent.append(goal)
            # Replay the J15 condition that aborted the real 20261012 goal.
            p.hw['position'][14] += .2
            p.hw['command_position'][14] = p.hw['position'][14] + .004001
            p.hw['velocity'] = [math.radians(1.5)]*20
            p.hw['velocity'][14] = math.radians(15.)
            p.guard()
            # A nonzero controller default must be explicitly erased too.
            for tolerance in goal.path_tolerance:
                self.assertEqual((tolerance.position, tolerance.velocity, tolerance.acceleration),
                                 (-1., -1., -1.))
            self.assertEqual([t.name for t in goal.path_tolerance], JOINTS)
            self.assertTrue(all(t.position == .001 for t in goal.goal_tolerance))
            p.hw['velocity'] = [0.]*20
            p.hw['position'] = goal.trajectory.points[-1].positions[:]
            p.hw['command_position'] = p.hw['position'][:]
            return Future(NS(accepted=True, get_result_async=lambda: Future(
                NS(status=4, result=NS(error_code=0, error_string='')))))
        p.trajectory_client = NS(send_goal_async=send)
        with patch.dict(rp.__dict__, STUBS): p.perform_step(plan)
        self.assertAlmostEqual(math.degrees(p.hold_positions[14]), 50.)
        self.assertTrue(all(p.hw['enabled']))
        self.assertEqual(p.commands, [f'enable({ALL_MASK})'])

    def test_speed_trip_stays_off_after_enabling_and_status_reports_it(self):
        p = fake_probe(rp.JointProbe); original = p.command
        def command(text, guarded=True):
            p.hw['velocity'] = [math.radians(8.)]*20
            original(text, guarded)
        p.command = command
        p.enable_all(); p.guard()
        rp.Probe.report(p, True)
        self.assertFalse(p.events[-1]['speed_protection'])
        self.assertFalse(p.events[-1]['path_tracking_check'])
        self.assertTrue(rp.Probe.feedback_speed_check)
        self.assertTrue(rp.Probe.path_tracking_check)

    def test_hardware_estop_feedback_link_and_held_position_checks_remain(self):
        for failure in ('hardware', 'estop', 'feedback', 'link', 'hold'):
            p = fake_probe(rp.JointProbe); p.enable_all(); p.selected = 15
            p.hw['velocity'] = [math.radians(8.)]*20
            p.guard()
            if failure == 'hardware': p.hw['errorcode'][14] = 1
            elif failure == 'estop': p.gpio = lambda: [1, 0]
            elif failure == 'feedback': p.hw_time -= 1.
            elif failure == 'link': p.received -= 1.
            else: p.hw['position'][15] += math.radians(.31)
            with self.subTest(failure=failure), self.assertRaises(RuntimeError): p.guard()


if __name__ == '__main__': unittest.main()
