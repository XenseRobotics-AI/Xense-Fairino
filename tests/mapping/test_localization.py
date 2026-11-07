from fairino_slam.paths import CONFIG_ROOT
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import fairino_slam.mapping.localization as loc
from fairino_slam.mapping.remote_stream import LocalizationMonitor


class FakeROS:
    def __init__(self):
        self.node = Mock()
        self.node.get_node_names_and_namespaces.return_value = [('amcl', '/')]
        self.current = 1
        self.events = []
        self.slam = False

    def slam_present(self):
        return self.slam

    def state(self, _):
        return self.current

    def transition(self, name, value):
        self.events.append(('transition', value))
        self.current = {1: 2, 3: 3, 4: 2, 2: 1}[value]

    def require_good_input(self, wait=0):
        self.events.append(('check', wait))

    def set_params(self, name, values):
        self.events.append((name, values))

    def map_server(self, active):
        self.events.append(('map', active))
        return active


class LifecycleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.map = root/'map.yaml'
        self.map.write_text('image: map.pgm')
        self.epoch = root/'epoch'
        self.epoch.write_text('A')
        for name, value in [('SESSION', root/'localization.json'), ('EPOCH', self.epoch),
                            ('PROFILE', CONFIG_ROOT/'localization.yaml')]:
            patcher = patch.object(loc, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        self.ros = FakeROS()
        self.control = loc.Localization(self.ros)

    def test_empty_map_and_slam_are_rejected_without_lifecycle_mutation(self):
        self.map.unlink()
        with self.assertRaisesRegex(RuntimeError, '尚无'):
            self.control.start(self.map)
        self.assertEqual(self.ros.events, [])
        self.ros.slam = True
        with self.assertRaisesRegex(RuntimeError, '结束建图'):
            self.control.start(self.map)
        self.assertEqual(self.ros.events, [])

    def test_start_waits_for_operator_pose_and_uses_omni_model(self):
        state = self.control.start(self.map)
        self.assertIsNone(state['initial_pose_stamp'])
        self.assertEqual(state['epoch'], 'A')
        self.assertEqual(self.ros.current, 3)
        params = next(v for k, v in self.ros.events if k == '/amcl')
        self.assertEqual(params['robot_model_type'], 'nav2_amcl::OmniMotionModel')
        self.assertFalse(params['set_initial_pose'])
        self.assertTrue(params['always_reset_initial_pose'])
        self.assertLess(self.ros.events.index(('map', True)), self.ros.events.index(('transition', 3)))

    def test_reentry_is_explicit_and_stop_invalidates_before_deactivation(self):
        self.control.start(self.map)
        with self.assertRaisesRegex(RuntimeError, '已运行'):
            self.control.start(self.map)
        transition = self.ros.transition
        def checking(name, value):
            self.assertFalse(loc.read_session()['active'])
            transition(name, value)
        self.ros.transition = checking
        self.control.stop()
        self.assertEqual(self.ros.current, 1)

    def test_failed_activation_cleans_up_and_does_not_claim_running(self):
        transition = self.ros.transition
        def failing(name, value):
            if value == 3:
                raise RuntimeError('activation failed')
            transition(name, value)
        self.ros.transition = failing
        with self.assertRaisesRegex(RuntimeError, 'activation failed'):
            self.control.start(self.map)
        self.assertFalse(loc.read_session()['active'])
        self.assertEqual(self.ros.current, 1)

    def test_pose_from_an_old_ui_session_is_rejected(self):
        self.control.start(self.map)
        with self.assertRaisesRegex(RuntimeError, '会话已改变'):
            self.control.set_pose(1., 2., 0., session_id='old-map-session')
        self.assertTrue(loc.read_session()['active'])
        self.assertIsNone(loc.read_session()['initial_pose_stamp'])

    def test_changed_odometry_epoch_requires_new_session(self):
        self.control.start(self.map)
        self.epoch.write_text('B')
        with self.assertRaisesRegex(RuntimeError, '会话已变化'):
            self.control.set_pose(1., 2., 0.)
        self.assertFalse(loc.read_session()['active'])

    def test_pose_units_covariance_and_invalid_values(self):
        pose = loc.initial_pose(1., -2., math.pi/2)
        self.assertAlmostEqual(pose['qz'], math.sqrt(.5))
        self.assertEqual(pose['covariance'][0], .25)
        self.assertAlmostEqual(pose['covariance'][35], math.radians(15)**2)
        for args in ((float('nan'), 0, 0), (0, 0, float('inf')), (0, 0, 0, 0), (0, 0, 0, .5, -1)):
            with self.assertRaises(ValueError):
                loc.initial_pose(*args)


class MonitoringTests(unittest.TestCase):
    def setUp(self):
        self.monitor = LocalizationMonitor()
        self.session = {'id': 'A', 'active': True, 'initial_pose_stamp': 100.,
                        'epoch': 'E', 'watchdog_time': 101.}
        self.monitor.set_session(self.session)
        self.cov = [0.] * 36
        self.cov[0] = self.cov[7] = self.cov[35] = .01
        self.odom = {'running': True, 'epoch': 'E'}
        self.topics = {'/odom': {'age': .01}, '/scan': {'age': .01}}

    def accept(self, stamp=101., cov=None, frame='map'):
        return self.monitor.accept(stamp, 0., [0., 0., 0.], cov or self.cov, frame)

    def status(self, now=101.1, tf=True, active=True):
        return self.monitor.status(now, self.odom, self.topics, tf, active, True)

    def test_first_estimate_requires_explicit_pose_and_correct_session(self):
        self.assertFalse(self.status()['ready'])
        self.assertFalse(self.accept(99.))
        self.assertFalse(self.accept(frame='odom'))
        self.assertTrue(self.accept())
        self.assertTrue(self.status()['ready'])
        self.monitor.set_session(dict(self.session, initial_pose_stamp=102.))
        self.assertFalse(self.status()['ready'])
        self.assertFalse(self.accept())

    def test_stationary_latched_pose_is_retained_but_motion_needs_new_match(self):
        self.assertTrue(self.monitor.accept(101., 60., [0., 0., 0.], self.cov, 'map'))
        self.monitor.on_odometry([0., 0., 0.], [0., 0., 0.])
        self.monitor.session['watchdog_time'] = 180.
        self.assertTrue(self.status(180.)['ready'])
        for index, x in enumerate((.06, .12, .06, 0.)):
            self.monitor.on_odometry([x, 0., 0.], [.06, 0., 0.])
            # The old stationary sample does not consume the matching deadline.
            self.assertTrue(self.status(180.1+index*.1)['ready'])
        self.assertEqual(self.status(182.21)['state'], 'stale')
        self.assertTrue(self.accept(182.3))
        self.assertTrue(self.status(182.4)['ready'])

    def test_reset_dropout_tf_and_inactive_node_invalidate_estimate(self):
        self.accept()
        self.assertFalse(self.status(tf=False)['ready'])
        self.assertFalse(self.status(active=False)['ready'])
        self.odom['epoch'] = 'OTHER'
        self.assertFalse(self.status()['ready'])
        self.odom['epoch'] = 'E'
        self.topics['/scan']['age'] = 5
        self.assertFalse(self.status()['ready'])

    def test_stationary_estimate_is_explicitly_normal_and_navigable(self):
        self.accept()
        self.monitor.session['watchdog_time'] = 180.
        state = self.status(180.)
        self.assertTrue(state['ready'])
        self.assertIn('定位正常，可导航', state['reason'])
        self.assertIsNone(state['update_wait_age'])

    def test_input_failure_identifies_topic_and_age_without_relaxing_limit(self):
        self.accept()
        self.topics['/scan']['age'] = .751
        state = self.status()
        self.assertFalse(state['ready'])
        self.assertIn('融合雷达', state['reason'])
        self.assertIn('0.751', state['reason'])
        self.topics['/scan']['age'] = .1
        self.topics['/odom']['age'] = None
        self.assertIn('里程计未收到有效数据', self.status()['reason'])

    def test_uncertainty_and_invalid_covariance_are_not_ready(self):
        cov = self.cov.copy()
        cov[0] = 1.
        self.assertTrue(self.accept(cov=cov))
        self.assertEqual(self.status()['state'], 'uncertain')
        cov[7] = -1.
        self.assertFalse(self.accept(102., cov))
        cov[7] = float('nan')
        self.assertFalse(self.accept(102., cov))

    def test_dead_watchdog_does_not_leave_ready_status(self):
        self.accept()
        self.assertFalse(self.status(110.)['ready'])


    def fresh_status(self, now):
        self.monitor.session['watchdog_time'] = now
        return self.status(now)

    def test_diagonal_distance_is_not_amcl_axis_displacement(self):
        self.monitor.on_odometry([0., 0., 0.], [0., 0., 0.])
        self.assertTrue(self.accept())
        self.monitor.on_odometry([.071, .071, 0.], [.04, .04, 0.])
        state = self.fresh_status(180.)
        self.assertGreater(state['travel_since_match'], .1)
        self.assertTrue(state['ready'])
        self.assertIsNone(state['update_wait_age'])
        self.monitor.on_odometry([.1, .071, 0.], [.04, .04, 0.])
        self.assertIsNone(self.fresh_status(180.1)['update_wait_age'])
        self.monitor.on_odometry([.1001, .071, 0.], [.04, .04, 0.])
        self.assertEqual(self.fresh_status(180.2)['update_wait_age'], 0.)

    def test_new_match_resets_wait_but_duplicate_cannot_extend_deadline(self):
        self.monitor.on_odometry([0., 0., 0.], [0., 0., 0.])
        self.assertTrue(self.accept())
        self.monitor.on_odometry([.11, 0., 0.], [.06, 0., 0.])
        self.assertTrue(self.fresh_status(180.)['ready'])
        self.assertFalse(self.accept())  # A latched duplicate cannot reset waiting.
        self.assertTrue(self.fresh_status(181.999)['ready'])
        self.assertEqual(self.fresh_status(182.)['state'], 'stale')
        self.assertTrue(self.accept(182.1))
        state = self.fresh_status(182.2)
        self.assertTrue(state['ready'])
        self.assertIsNone(state['update_wait_age'])
        self.assertEqual(state['odometry_delta'], [0., 0., 0.])

    def test_timestamped_delayed_match_preserves_motion_after_match(self):
        self.monitor.on_odometry([0., 0., 0.], [0., 0., 0.], 101.)
        self.assertTrue(self.accept())
        self.monitor.on_odometry([.22, 0., 0.], [.06, 0., 0.], 103.)
        self.assertTrue(self.monitor.accept(102., 1., [0., 0., 0.], self.cov, 'map'))
        state = self.fresh_status(103.)
        self.assertTrue(state['baseline_covered'])
        self.assertEqual(state['baseline_stamp'], 102.)
        self.assertAlmostEqual(state['odometry_delta'][0], .11)
        self.assertAlmostEqual(state['travel_since_match'], .11)
        self.assertEqual(state['update_wait_age'], 0.)

    def test_uncovered_latched_pose_uses_observation_baseline(self):
        self.assertTrue(self.monitor.accept(101., 99., [0., 0., 0.], self.cov, 'map'))
        self.monitor.on_odometry([20., 30., 0.], [0., 0., 0.], 200.)
        self.assertTrue(self.fresh_status(200.)['ready'])
        self.monitor.on_odometry([20.11, 30., 0.], [.06, 0., 0.], 201.)
        state = self.fresh_status(201.)
        self.assertFalse(state['baseline_covered'])
        self.assertEqual(state['baseline_stamp'], 200.)
        self.assertAlmostEqual(state['odometry_delta'][0], .11)
        self.assertEqual(state['update_wait_age'], 0.)

    def test_incrementing_old_matches_cannot_reset_motion_or_wait(self):
        self.monitor.on_odometry([0., 0., 0.], [0., 0., 0.], 200.)
        self.assertTrue(self.monitor.accept(101., 99., [0., 0., 0.], self.cov, 'map'))
        self.monitor.on_odometry([.11, 0., 0.], [.06, 0., 0.], 200.1)
        baseline = self.fresh_status(200.1)
        for stamp in (102., 103., 104.):
            self.assertFalse(self.monitor.accept(stamp, 201.-stamp, [0., 0., 0.], self.cov, 'map'))
        state = self.fresh_status(202.2)
        self.assertEqual(state['state'], 'stale')
        self.assertEqual(state['stamp'], 101.)
        self.assertEqual(state['baseline_stamp'], baseline['baseline_stamp'])
        self.assertEqual(state['travel_since_match'], baseline['travel_since_match'])
        self.assertGreaterEqual(state['update_wait_age'], 2.)

    def test_legacy_untimed_odometry_rejects_subsequent_aged_match(self):
        self.monitor.on_odometry([0., 0., 0.], [0., 0., 0.])
        self.assertTrue(self.accept())
        self.monitor.on_odometry([.11, 0., 0.], [.06, 0., 0.])
        self.fresh_status(180.)
        self.assertFalse(self.monitor.accept(102., 78., [0., 0., 0.], self.cov, 'map'))
        self.assertEqual(self.fresh_status(182.)['state'], 'stale')

    def test_subthreshold_loops_still_hit_cumulative_hard_limit(self):
        self.monitor.on_odometry([0., 0., 0.], [0., 0., 0.])
        self.assertTrue(self.accept())
        for index in range(13):
            self.monitor.on_odometry([.04 if index % 2 == 0 else 0., 0., 0.], [.04, 0., 0.])
        state = self.fresh_status(180.)
        self.assertIsNone(state['update_wait_age'])
        self.assertGreater(state['travel_since_match'], state['hard_distance_limit'])
        self.assertEqual(state['state'], 'stale')

    def test_heading_wrap_and_rotational_loops(self):
        self.monitor.on_odometry([0., 0., math.radians(179)], [0., 0., 0.])
        self.assertTrue(self.accept())
        self.monitor.on_odometry([0., 0., math.radians(-179)], [0., 0., .1])
        self.assertTrue(self.fresh_status(180.)['ready'])
        self.assertAlmostEqual(self.fresh_status(180.)['odometry_delta'][2], math.radians(2))
        for index in range(8):
            angle = 179 if index % 2 == 0 else -179
            self.monitor.on_odometry([0., 0., math.radians(angle)], [0., 0., .1])
        state = self.fresh_status(180.1)
        self.assertIsNone(state['update_wait_age'])
        self.assertEqual(state['state'], 'stale')

    def test_session_parameters_set_update_thresholds_and_invalid_values_fail_closed(self):
        self.monitor.set_session(dict(self.session, amcl_parameters={
            'update_min_d': .2, 'update_min_a': math.radians(10)}))
        self.monitor.on_odometry([0., 0., 0.], [0., 0., 0.])
        self.assertTrue(self.accept())
        self.monitor.on_odometry([.15, 0., 0.], [.06, 0., 0.])
        self.assertIsNone(self.fresh_status(180.)['update_wait_age'])
        self.monitor.on_odometry([.21, 0., 0.], [.06, 0., 0.])
        self.assertEqual(self.fresh_status(180.1)['update_wait_age'], 0.)
        for parameters in (None, {}, {'update_min_d': True, 'update_min_a': .1},
                           {'update_min_d': float('nan'), 'update_min_a': .1},
                           {'update_min_d': -.1, 'update_min_a': .1},
                           {'update_min_d': .1, 'update_min_a': float('inf')}):
            with self.subTest(parameters=parameters):
                self.monitor.set_session(dict(self.session, amcl_parameters=parameters))
                state = self.fresh_status(180.2)
                self.assertFalse(state['ready'])
                self.assertIn('参数', state['reason'])

    def test_zero_update_thresholds_have_grace_and_nonzero_hard_limits(self):
        self.monitor.set_session(dict(self.session, amcl_parameters={
            'update_min_d': 0., 'update_min_a': 0.}))
        self.monitor.on_odometry([0., 0., 0.], [0., 0., 0.])
        self.assertTrue(self.accept())
        self.assertTrue(self.fresh_status(180.)['ready'])
        self.monitor.on_odometry([.001, 0., 0.], [.006, 0., 0.])
        state = self.fresh_status(180.1)
        self.assertTrue(state['ready'])
        self.assertGreater(state['hard_distance_limit'], 0.)
        self.assertGreater(state['hard_angle_limit'], 0.)
        self.assertEqual(self.fresh_status(182.2)['state'], 'stale')

    def test_session_reset_clears_history_reference_and_wait(self):
        self.monitor.on_odometry([0., 0., 0.], [0., 0., 0.], 101.)
        self.accept()
        self.monitor.on_odometry([.11, 0., 0.], [.06, 0., 0.], 102.)
        self.fresh_status(102.)
        self.monitor.set_session(dict(self.session, id='B', initial_pose_stamp=103.))
        self.assertEqual(len(self.monitor.history), 0)
        state = self.fresh_status(103.)
        self.assertFalse(state['ready'])
        self.assertIsNone(state['baseline_stamp'])
        self.assertIsNone(state['update_wait_age'])
        self.assertEqual(state['travel_since_match'], 0.)

    def test_odometry_history_is_bounded_and_rejects_out_of_order_messages(self):
        for index in range(4000):
            self.monitor.on_odometry([index*.00001, 0., 0.], [.01, 0., 0.], 101.+index*.001)
        self.assertLessEqual(len(self.monitor.history), self.monitor.ODOM_HISTORY_LIMIT)
        latest = self.monitor.odometry
        self.monitor.on_odometry([100., 0., 0.], [.01, 0., 0.], 101.)
        self.assertEqual(self.monitor.odometry, latest)



    def test_late_matches_inside_history_cannot_postpone_existing_crossing(self):
        self.monitor.on_odometry([0., 0., 0.], [0., 0., 0.], 101.)
        self.assertTrue(self.accept())
        self.monitor.on_odometry([0., 0., 0.], [0., 0., 0.], 101.5)
        self.monitor.on_odometry([.11, 0., 0.], [.06, 0., 0.], 101.6)
        self.assertEqual(self.fresh_status(101.6)['update_wait_age'], 0.)
        self.monitor.on_odometry([.11, 0., 0.], [0., 0., 0.], 103.5)
        for stamp, now in ((101.2, 103.5), (101.3, 103.7), (101.4, 103.8)):
            self.assertTrue(self.monitor.accept(stamp, now-stamp, [0., 0., 0.], self.cov, 'map'))
            state = self.fresh_status(now)
            self.assertTrue(state['baseline_covered'])
            self.assertAlmostEqual(state['update_wait_age'], now-101.6)
            self.assertEqual(state['ready'], now < 103.6)

    def test_new_reference_rebuilds_a_later_genuine_motion_deadline(self):
        for stamp, x in ((101., 0.), (101.5, .11), (102., .2), (102.5, .31)):
            self.monitor.on_odometry([x, 0., 0.], [.06, 0., 0.], stamp)
        self.assertTrue(self.monitor.accept(101., 1.5, [0., 0., 0.], self.cov, 'map'))
        self.assertAlmostEqual(self.fresh_status(102.5)['update_wait_age'], 1.)
        self.assertTrue(self.monitor.accept(102., .5, [0., 0., 0.], self.cov, 'map'))
        state = self.fresh_status(103.)
        self.assertTrue(state['ready'])
        self.assertEqual(state['update_wait_age'], .5)
        self.assertAlmostEqual(state['odometry_delta'][0], .11)

    def test_invalid_watchdog_and_topic_ages_never_claim_ready(self):
        self.accept()
        for value in (None, float('nan'), float('inf'), True, '101', -1.):
            with self.subTest(watchdog=value):
                self.monitor.session['watchdog_time'] = value
                self.assertFalse(self.status()['ready'])
        self.monitor.session['watchdog_time'] = 101.
        for topic in ('/scan', '/odom'):
            for value in (None, float('nan'), float('inf'), True, '0.01', -.01, .751):
                with self.subTest(topic=topic, age=value):
                    self.topics[topic]['age'] = value
                    self.assertFalse(self.status()['ready'])
            self.topics[topic]['age'] = .01
        self.assertTrue(self.status()['ready'])

    def test_clock_rollback_beyond_existing_future_tolerance_is_not_ready(self):
        self.assertTrue(self.monitor.accept(102., -.4, [0., 0., 0.], self.cov, 'map'))
        self.assertTrue(self.fresh_status(101.6)['ready'])
        self.assertFalse(self.fresh_status(101.4)['ready'])
        for value in (None, float('nan'), float('inf'), True, '102'):
            with self.subTest(now=value):
                self.assertFalse(self.monitor.status(
                    value, self.odom, self.topics, True, True, True)['ready'])
        self.assertTrue(self.fresh_status(102.)['ready'])



class WatchdogTests(unittest.TestCase):
    def test_lost_input_deactivates_and_does_not_auto_resume(self):
        from contextlib import nullcontext
        clock = [0.]
        ros = FakeROS()
        ros.source = '/wheel/odom'
        ros.current = 3
        ros.spin = lambda duration: clock.__setitem__(0, clock[0]+duration)
        ros.input_reason = lambda: 'scan lost'
        with tempfile.TemporaryDirectory() as directory, \
                patch.object(loc, 'SESSION', Path(directory)/'session.json'), \
                patch.object(loc, 'epoch', return_value='E'), \
                patch.object(loc.time, 'monotonic', side_effect=lambda: clock[0]), \
                patch.object(loc.time, 'time', side_effect=lambda: 100+clock[0]):
            loc.write_session({'id': 'A', 'epoch': 'E', 'active': True})
            loc.watch(ros, nullcontext)
            self.assertFalse(loc.read_session()['active'])
            self.assertEqual(loc.read_session()['reason'], 'scan lost')
            self.assertEqual(ros.current, 1)
            self.assertNotIn(('transition', 3), ros.events)


class TransportAndInstallationTests(unittest.TestCase):
    def test_initial_pose_reaches_remote_command_in_radians(self):
        import sys
        import shlex
        from fairino_slam.mapping import client as map_mode
        with patch.object(map_mode, 'run', return_value='MAP_CONTROL {"message": "ok"}') as run:
            map_mode.execute('initial-pose', password='unused',
                             pose={'x': 1.2, 'y': -.4, 'yaw': math.pi/2, 'session_id': 'A'})
        command = shlex.split(run.call_args.args[1].splitlines()[-1])
        for key, value in (('--x', '1.2'), ('--y', '-0.4'), ('--yaw', str(math.pi/2)), ('--session-id', 'A')):
            self.assertEqual(command[command.index(key)+1], value)

    def test_installer_embedded_program_compiles_and_delivers_dependencies(self):
        import sys
        import shlex
        from fairino_slam.deployment import install_map_mode
        with patch.object(sys, 'argv', ['deploy', '--execute']), \
                patch.object(install_map_mode, 'run', side_effect=RuntimeError('captured')) as run:
            with self.assertRaisesRegex(RuntimeError, 'captured'):
                install_map_mode.main(password='unused')
        program = shlex.split(run.call_args.args[1])[3]
        compile(program, '<robot-installer>', 'exec')
        self.assertIn('fairino-localization.service', program)
        self.assertIn('watch-localization', program)
        import ast
        tree = ast.parse(program)
        payload = ast.literal_eval(tree.body[0].value)
        self.assertEqual(set(payload), {'localization.py', 'localization.yaml'})

    def test_map_operations_share_nonblocking_lock(self):
        import fairino_slam.mapping.map_control_remote as manager
        with tempfile.TemporaryDirectory() as directory, patch.object(manager, 'RUNTIME', Path(directory)):
            with manager.mode_lock():
                with self.assertRaises(BlockingIOError):
                    with manager.mode_lock():
                        pass


if __name__ == '__main__':
    unittest.main()
