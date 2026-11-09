import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import localization as loc
from remote_stream import LocalizationMonitor


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
                            ('PROFILE', Path(__file__).with_name('localization.yaml'))]:
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
        for x in (.06, .12, .06, 0.):
            self.monitor.on_odometry([x, 0., 0.], [.06, 0., 0.])
        self.assertEqual(self.status(180.)['state'], 'stale')
        self.assertTrue(self.accept(180.))
        self.assertTrue(self.status(180.1)['ready'])

    def test_reset_dropout_tf_and_inactive_node_invalidate_estimate(self):
        self.accept()
        self.assertFalse(self.status(tf=False)['ready'])
        self.assertFalse(self.status(active=False)['ready'])
        self.odom['epoch'] = 'OTHER'
        self.assertFalse(self.status()['ready'])
        self.odom['epoch'] = 'E'
        self.topics['/scan']['age'] = 5
        self.assertFalse(self.status()['ready'])

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
        sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'tools'))
        import map_mode
        with patch.object(map_mode, 'run', return_value='MAP_CONTROL {"message": "ok"}') as run:
            map_mode.execute('initial-pose', password='unused',
                             pose={'x': 1.2, 'y': -.4, 'yaw': math.pi/2, 'session_id': 'A'})
        command = shlex.split(run.call_args.args[1].splitlines()[-1])
        for key, value in (('--x', '1.2'), ('--y', '-0.4'), ('--yaw', str(math.pi/2)), ('--session-id', 'A')):
            self.assertEqual(command[command.index(key)+1], value)

    def test_installer_embedded_program_compiles_and_delivers_dependencies(self):
        import sys
        import shlex
        sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'tools'))
        import install_map_mode
        with patch.object(install_map_mode, 'run', side_effect=RuntimeError('captured')) as run, \
                patch.object(sys, 'argv', ['install_map_mode.py', '--execute']):
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
        import map_control_remote as manager
        with tempfile.TemporaryDirectory() as directory, patch.object(manager, 'RUNTIME', Path(directory)):
            with manager.mode_lock():
                with self.assertRaises(BlockingIOError):
                    with manager.mode_lock():
                        pass


if __name__ == '__main__':
    unittest.main()
