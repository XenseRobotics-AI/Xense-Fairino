"""Age checks use output-time clocks even when synchronous gate work stalls."""
from types import SimpleNamespace as NS
import unittest

from fairino_slam.navigation.motion import CommandGate
from fairino_slam.navigation.velocity_gate import (
    gate_output, localization_tf_health, readiness_snapshot)


class Clock:
    def __init__(self):
        self.elapsed = 0.

    def advance(self, seconds):
        self.elapsed += seconds

    def monotonic(self):
        return 100.+self.elapsed

    def ros(self):
        return 1000.+self.elapsed

    def wall(self):
        return 2000.+self.elapsed


class Buffer:
    def __init__(self, stamp, clock, delay=0.):
        self.stamp, self.clock, self.delay = stamp, clock, delay

    def lookup_transform(self, parent, child, query_time):
        self.clock.advance(self.delay)
        self.delay = 0.
        sec = int(self.stamp)
        return NS(header=NS(stamp=NS(sec=sec, nanosec=round((self.stamp-sec)*1e9))))


class Monitor:
    """Only distinguish inputs relevant to these clock tests."""
    def status(self, now_ros, odometry, ages, tf_ready, active, source_ok):
        if not odometry.get('running'):
            return {'ready': False, 'reason': '里程计未就绪'}
        return {'ready': tf_ready, 'reason': None if tf_ready else '定位 TF 未就绪或已过期'}


class Lifecycle:
    def __init__(self, clock, delay=0., state_age=0., on_tick=None):
        self.clock, self.delay = clock, delay
        self.updated = clock.monotonic()-state_age
        self.before_ready = None
        self.on_tick = on_tick

    def tick(self, now, healthy, token, busy):
        self.before_ready = healthy
        self.clock.advance(self.delay)
        if self.on_tick:
            self.on_tick()

    def active(self, now):
        return now-self.updated < 3.


class GateTimingTests(unittest.TestCase):
    def run_cycle(self, *, lifecycle_delay=0., lookup_delay=0., before_delay=0.,
                  tf_age=.1, scan_age=.1, command_age=.1, odometry_age=0., lifecycle_age=0.,
                  update_tf_during_lifecycle=False):
        clock = Clock()
        gate = CommandGate(.06, .1)
        gate.set_goal('goal', clock.monotonic()-.5)
        gate.receive([.03, 0, 0, 0, 0, 0], clock.monotonic()-command_age)
        state = {'id': 'session', 'initial_pose_stamp': 990., 'active': True}
        odometry = {'running': True, 'epoch': 'epoch', 'time': clock.wall()-odometry_age}
        topics = {name: (clock.monotonic(), scan_age)
                  for name in ('/scan', '/odom', '/front/scan', '/back/scan')}
        buffer = Buffer(1000.-tf_age, clock, lookup_delay)
        def refresh_tf():
            buffer.stamp = clock.ros()-.01
        lifecycle = Lifecycle(clock, lifecycle_delay, lifecycle_age,
                              refresh_tf if update_tf_during_lifecycle else None)
        # These delays represent graph/file work and TF calls before decisions.
        clock.advance(before_delay)
        read_tf = lambda: localization_tf_health(buffer, clock.ros(), 990., object())
        snapshot = lambda: readiness_snapshot(
            Monitor(), state, odometry, topics, read_tf, True,
            clock.monotonic, clock.ros, clock.wall)
        current = gate_output(gate, lifecycle, ('session', 990., 'epoch'), True,
                              snapshot, clock.monotonic)
        return current, gate, lifecycle

    def test_fresh_inputs_still_forward(self):
        current, gate, _ = self.run_cycle()
        self.assertEqual(current['output'], (.03, 0., 0.))
        self.assertIsNone(gate.failure)
        self.assertAlmostEqual(current['tf']['transforms']['map_base']['age'], .1)

    def test_450ms_command_age_is_within_new_timeout(self):
        current, gate, _ = self.run_cycle(lifecycle_delay=.35)
        self.assertTrue(current['ready'])
        self.assertEqual(current['output'], (.03, 0., 0.))
        self.assertIsNone(gate.failure)

    def test_350ms_lifecycle_stall_cannot_forward_expired_command(self):
        current, gate, lifecycle = self.run_cycle(lifecycle_delay=.35, command_age=.3)
        self.assertTrue(lifecycle.before_ready)
        self.assertTrue(current['ready'])
        self.assertEqual(current['output'], (0., 0., 0.))
        self.assertAlmostEqual(gate.failure['command_age'], .65)
        self.assertAlmostEqual(current['now'], 100.35)
        self.assertAlmostEqual(current['lifecycle_seconds'], .35)
        self.assertIn('速度指令断流', gate.failure['reason'])

    def test_350ms_graph_or_file_stall_is_included_in_command_age(self):
        current, gate, _ = self.run_cycle(before_delay=.35, command_age=.3)
        self.assertEqual(current['output'], (0., 0., 0.))
        self.assertAlmostEqual(gate.failure['command_age'], .65)

    def test_tf_lookup_delay_is_included_in_final_tf_age(self):
        current, gate, lifecycle = self.run_cycle(lookup_delay=.35, tf_age=.7)
        self.assertFalse(lifecycle.before_ready)
        self.assertFalse(current['tf']['ready'])
        self.assertAlmostEqual(current['tf']['transforms']['map_base']['age'], 1.05)
        self.assertEqual(current['output'], (0., 0., 0.))
        self.assertIn('定位 TF 已过期 1.050 秒', gate.failure['reason'])

    def test_lifecycle_delay_rechecks_tf_after_initial_healthy_snapshot(self):
        current, gate, lifecycle = self.run_cycle(lifecycle_delay=.35, tf_age=.7)
        self.assertTrue(lifecycle.before_ready)
        self.assertFalse(current['ready'])
        self.assertAlmostEqual(current['tf']['transforms']['map_base']['age'], 1.05)
        self.assertEqual(current['output'], (0., 0., 0.))
        self.assertIn('定位 TF 已过期 1.050 秒', gate.failure['reason'])

    def test_lifecycle_delay_rechecks_lidar_receipt_and_stamp_age(self):
        current, gate, lifecycle = self.run_cycle(lifecycle_delay=.35, scan_age=.2)
        self.assertTrue(lifecycle.before_ready)
        self.assertTrue(current['tf']['ready'])
        self.assertAlmostEqual(current['topic_ages']['/front/scan']['age'], .55)
        self.assertEqual(current['output'], (0., 0., 0.))
        self.assertIn('双雷达观测过期', gate.failure['reason'])

    def test_lifecycle_delay_rechecks_odometry_status_wall_clock_age(self):
        current, gate, lifecycle = self.run_cycle(lifecycle_delay=.35, odometry_age=1.8)
        self.assertTrue(lifecycle.before_ready)
        self.assertFalse(current['ready'])
        self.assertEqual(current['output'], (0., 0., 0.))
        self.assertIn('里程计未就绪', gate.failure['reason'])

    def test_lifecycle_feedback_age_uses_time_after_lifecycle_work(self):
        current, gate, lifecycle = self.run_cycle(lifecycle_delay=.35, lifecycle_age=2.8)
        self.assertTrue(lifecycle.before_ready)
        self.assertTrue(current['ready'])
        self.assertFalse(current['stack_ready'])
        self.assertEqual(current['output'], (0., 0., 0.))
        self.assertIn('生命周期反馈过期', gate.failure['reason'])


    def test_final_snapshot_reads_tf_received_during_blocked_lifecycle(self):
        current, gate, lifecycle = self.run_cycle(
            lifecycle_delay=.35, command_age=.3, tf_age=.7,
            update_tf_during_lifecycle=True)
        self.assertTrue(lifecycle.before_ready)
        self.assertTrue(current['tf']['ready'])
        self.assertAlmostEqual(current['tf']['transforms']['map_base']['age'], .01)
        self.assertTrue(current['ready'])
        # Receiving fresh TF cannot hide an unprocessed command stream.
        self.assertEqual(current['output'], (0., 0., 0.))
        self.assertIn('速度指令断流', gate.failure['reason'])

    def test_fresh_independent_tf_cannot_hide_unprocessed_lidar(self):
        current, gate, _ = self.run_cycle(
            lifecycle_delay=.35, tf_age=.7, scan_age=.2,
            update_tf_during_lifecycle=True)
        self.assertTrue(current['tf']['ready'])
        self.assertFalse(current['ready'])
        self.assertEqual(current['output'], (0., 0., 0.))
        self.assertIn('双雷达观测过期', gate.failure['reason'])


if __name__ == '__main__':
    unittest.main()


class InputAgeEvidenceTests(unittest.TestCase):
    def snapshot(self, received, stamp_age):
        clock = Clock()
        return readiness_snapshot(
            Monitor(), dict(active=True), dict(running=True, time=clock.wall()),
            {'/odom': (received, stamp_age)}, lambda: dict(
                transforms={'map_base': dict(stamp=clock.ros(), age=0., parent='map', child='base_footprint')},
                initial_pose_stamp=990., max_combined_age=1.),
            True, clock.monotonic, clock.ros, clock.wall)['topic_ages']['/odom']

    def test_old_sample_received_now_keeps_its_age_and_exposes_backlog(self):
        state = self.snapshot(100., .81)
        self.assertAlmostEqual(state['age'], .81)
        self.assertEqual(state['received_age'], 0.)
        self.assertAlmostEqual(state['stamp_age_at_receive'], .81)
        self.assertAlmostEqual(state['stamp_age'], .81)

    def test_actual_stream_gap_still_exceeds_watchdog(self):
        state = self.snapshot(99.2, .01)
        self.assertGreater(state['received_age'], .75)
        self.assertAlmostEqual(state['age'], .81)

    def test_future_stamp_cannot_hide_a_gap(self):
        state = self.snapshot(99.2, -.8)
        self.assertAlmostEqual(state['stamp_age'], 0.)
        self.assertAlmostEqual(state['age'], .8)
