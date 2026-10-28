"""Real-observation display rules; no ROS, SSH or motor connections."""
import json
import math
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import remote_stream

from remote_stream import (DisplayTelemetry, LocalizationMonitor, bounded_points,
                           display_session, observation_yaw, path_observation)


class TelemetryTests(unittest.TestCase):
    def setUp(self):
        self.cache = DisplayTelemetry()
        self.session = ['saved', '/maps/office/map.yaml', None, 'loc-A', 100., 'odom-A']
        self.navigation_session = ['loc-A', 100., 'odom-A']
        self.cache.set_session(self.session, True, 100., self.navigation_session)
        self.points = [[1., 2., .3, .7], [1.1, 2.1, .4, .3]]
        self.state = {'time': 101., 'goal': 'goal-A', 'run_id': 'run-A',
                      'controller_ready': True, 'localization_ready': True,
                      'session_token': self.navigation_session}

    def snapshot(self, now=11., visible=True, source=True, path_sources=None):
        return self.cache.snapshot(now, visible, '定位不可用', source,
                                   path_sources or {'global': True, 'local': True})

    def particle(self, **changes):
        options = dict(points=self.points, stamp=101., frame='map', now_ros=101.1,
                       now=10., source_confirmed=True)
        options.update(changes)
        return self.cache.accept_particles(**options)

    def navigation(self, **changes):
        self.cache.set_navigation(dict(self.state, **changes), 101., 101.)

    def path(self, kind='global', **changes):
        options = dict(points=[[1., 2.], [3., 4.]], stamp=101.1, frame='map',
                       now_ros=101.2, now=10., source_confirmed=True)
        options.update(changes)
        return self.cache.accept_path(kind, **options)

    def test_sample_limit_preserves_real_points_and_endpoints(self):
        cloud = [[i*.01, float(i), .1, 1./2000] for i in range(2000)]
        self.assertTrue(self.particle(points=cloud))
        output, _ = self.snapshot()
        self.assertTrue(output['valid'])
        self.assertEqual(output['count'], 2000)
        self.assertEqual(len(output['points']), 600)
        self.assertEqual(output['points'][0], cloud[0])
        self.assertEqual(output['points'][-1], cloud[-1])
        self.assertTrue(all(point in cloud for point in output['points']))
        self.assertEqual(len(bounded_points([[i, i] for i in range(2000)], 400, 2)), 400)

    def test_stationary_particles_retained_without_faking_timestamp(self):
        self.particle()
        output, _ = self.snapshot(now=120.)
        self.assertTrue(output['valid'])
        self.assertEqual(output['state'], 'retained')
        self.assertEqual(output['stamp'], 101.)
        self.assertGreater(output['age'], 100.)
        self.cache.on_odometry([0., 0., 0.], [0., 0., 0.])
        self.cache.on_odometry([.15, 0., 0.], [.06, 0., 0.])
        output, _ = self.snapshot(now=121.)
        self.assertFalse(output['valid'])
        self.assertEqual(output['state'], 'stale')
        self.assertEqual(output['points'], [])

    def test_rotation_also_invalidates_old_particle_cloud(self):
        self.particle()
        self.cache.on_odometry([0., 0., 0.], [0., 0., .1])
        self.cache.on_odometry([0., 0., .12], [0., 0., .1])
        self.assertFalse(self.snapshot(now=14.)[0]['valid'])
        self.assertTrue(self.particle(stamp=105., now_ros=105.1, now=14.))
        self.assertTrue(self.snapshot(now=14.1)[0]['valid'])

    def test_particle_rejects_wrong_frame_history_future_invalid_weights(self):
        for changes in ({'frame': 'odom'}, {'stamp': 99.}, {'stamp': 102.},
                        {'source_confirmed': False}, {'points': []},
                        {'points': [[1., 2., .1, -.1]]},
                        {'points': [[float('nan'), 2., .1, .1]]}):
            with self.subTest(changes=changes):
                self.assertFalse(self.particle(**changes))
                self.assertFalse(self.snapshot()[0]['valid'])
                self.assertEqual(self.snapshot()[0]['points'], [])
                json.dumps(self.snapshot(), allow_nan=False)

    def test_source_or_localization_loss_hides_retained_cloud(self):
        self.particle()
        self.assertFalse(self.snapshot(source=False)[0]['valid'])
        self.assertFalse(self.snapshot(visible=False)[0]['valid'])
        self.assertEqual(self.snapshot(visible=False)[0]['points'], [])

    def test_every_display_identity_change_clears_old_overlays(self):
        for index, replacement in enumerate(['mapping', '/maps/other/map.yaml', 123, 'loc-B', 102., 'odom-B']):
            with self.subTest(index=index):
                self.cache.set_session(self.session, True, 100., self.navigation_session)
                self.particle()
                self.navigation()
                self.path()
                changed = list(self.session)
                changed[index] = replacement
                self.cache.set_session(changed, True, 100., self.navigation_session)
                cloud, paths = self.snapshot()
                self.assertEqual(cloud['points'], [])
                self.assertEqual(paths['global']['points'], [])
                self.assertFalse(paths['global']['valid'])

    def test_paths_need_active_matching_navigation_session_and_new_observation(self):
        self.assertFalse(self.path())
        for changes in ({'goal': None}, {'run_id': None}, {'time': 90.},
                        {'session_token': ['loc-OLD', 100., 'odom-A']},
                        {'controller_ready': False}, {'localization_ready': False}):
            with self.subTest(changes=changes):
                self.navigation(**changes)
                self.assertFalse(self.path())
        self.navigation()
        self.assertFalse(self.path(stamp=100.9))
        self.assertTrue(self.path())
        self.assertEqual(self.snapshot()[1]['global']['goal_id'], 'goal-A')

    def test_new_goal_new_process_and_termination_clear_execution_paths(self):
        for change in ({'goal': 'goal-B'}, {'run_id': 'run-B'}, {'goal': None}, {'time': 90.}):
            with self.subTest(change=change):
                self.navigation()
                self.path()
                self.cache.set_navigation(dict(self.state, **change), 102., 102.)
                self.assertEqual(self.snapshot()[1]['global']['points'], [])
                self.assertFalse(self.path(stamp=101.5, now_ros=102.1))

    def test_global_and_local_have_independent_actual_age_limits(self):
        self.navigation()
        self.path()
        self.path('local')
        cloud, paths = self.snapshot(now=11.)
        self.assertTrue(paths['global']['valid'])
        self.assertFalse(paths['local']['valid'])
        self.assertEqual(paths['local']['points'], [])
        self.assertFalse(self.snapshot(now=14.)[1]['global']['valid'])
        self.assertFalse(self.snapshot(now=10.1, visible=False)[1]['global']['valid'])

    def test_old_path_timestamp_does_not_become_fresh_by_republication(self):
        self.navigation()
        self.path()
        self.assertFalse(self.path(stamp=101.1, now_ros=105., now=14.))
        self.assertEqual(self.snapshot(now=14.)[1]['global']['points'], [])

    def test_odom_path_transforms_to_map_and_missing_tf_clears_it(self):
        self.navigation()
        self.assertTrue(self.path(frame='odom', points=[[1., 0.], [2., 0.]],
                                  transform=[5., 6., math.pi/2]))
        path = self.snapshot(now=10.1)[1]['global']
        self.assertEqual(path['frame'], 'map')
        self.assertEqual(path['source_frame'], 'odom')
        self.assertAlmostEqual(path['points'][0][0], 5.)
        self.assertAlmostEqual(path['points'][0][1], 7.)
        self.assertFalse(self.path(frame='odom', transform=None))
        self.assertFalse(self.snapshot()[1]['global']['valid'])
        self.assertFalse(self.path(points=[[float('inf'), 0.]]))

    def test_covariance_includes_oblique_xy_term_without_changing_readiness(self):
        monitor = LocalizationMonitor()
        monitor.set_session({'active': True, 'initial_pose_stamp': 100., 'id': 'loc-A'})
        covariance = [0.]*36
        covariance[0], covariance[7], covariance[1], covariance[6], covariance[35] = .04, .09, .02, .02, .01
        self.assertTrue(monitor.accept(101., .1, [1., 2., .3], covariance, 'map'))
        self.assertEqual(monitor.sample['covariance_xy'], [.04, .02, .09])
        self.assertAlmostEqual(monitor.sample['yaw_std'], .1)

    def test_session_token_ignores_poll_times_and_preserves_reinitialization(self):
        loc = {'id': 'A', 'initial_pose_stamp': 100., 'watchdog_time': 101.}
        first = display_session('saved', 'map.yaml', 12, loc, 'epoch')
        loc['watchdog_time'] = 200.
        self.assertEqual(first, display_session('saved', 'map.yaml', 15, loc, 'epoch'))
        loc['initial_pose_stamp'] = 102.
        self.assertNotEqual(first, display_session('saved', 'map.yaml', 12, loc, 'epoch'))
        self.assertNotEqual(display_session('mapping', 'map.yaml', 12, loc, 'epoch'),
                            display_session('mapping', 'map.yaml', 13, loc, 'epoch'))


class PathMessageTests(unittest.TestCase):
    def message(self, frame='odom', stamp=None):
        return NS(header=NS(frame_id=frame, stamp=stamp or NS(sec=123, nanosec=500000000)),
                  poses=[NS(header=NS(frame_id=frame), pose=NS(position=NS(x=1., y=2.)))])

    def test_lookup_uses_original_path_time_and_never_latest_tf(self):
        message = self.message()
        lookup = Mock(return_value=NS(transform=NS(translation=NS(x=3., y=4.),
                          rotation=NS(x=0., y=0., z=0., w=1.))))
        points, stamp, frame, transform = path_observation(message, lookup)
        lookup.assert_called_once_with('map', 'odom', message.header.stamp)
        self.assertEqual((stamp, frame, points, transform), (123.5, 'odom', [[1., 2.]], [3., 4., 0.]))
        lookup.reset_mock()
        path_observation(self.message('map'), lookup)
        lookup.assert_not_called()

    def test_invalid_frame_mixed_frames_zero_stamp_or_missing_tf_rejected(self):
        mixed = self.message()
        mixed.poses[0].header.frame_id = 'map'
        lookup = Mock(side_effect=RuntimeError('extrapolation'))
        for msg in (self.message('laser'), mixed, self.message(stamp=NS(sec=0, nanosec=0)), self.message()):
            with self.subTest(msg=msg), self.assertRaises(ValueError):
                path_observation(msg, lookup)

    def test_invalid_quaternion_is_not_an_invented_heading(self):
        for q in (NS(x=0., y=0., z=0., w=0.), NS(x=0., y=0., z=float('nan'), w=1.)):
            with self.assertRaises(ValueError):
                observation_yaw(q)
        self.assertAlmostEqual(observation_yaw(NS(x=0., y=0., z=2., w=2.)), math.pi/2)


class StreamWiringTests(unittest.TestCase):
    def test_first_static_cloud_survives_initial_mode_poll(self):
        """A one-off real cloud before the first graph poll must not disappear."""
        callbacks = {}
        output = []
        q = NS(x=0., y=0., z=0., w=1.)
        pose = NS(position=NS(x=1., y=2., z=0.), orientation=q)
        covariance = [0.]*36
        covariance[0], covariance[7], covariance[35] = .01, .01, .01

        def stamp():
            ns = time.time_ns()
            return NS(sec=ns//1000000000, nanosec=ns%1000000000)

        class FakeTime:
            def __init__(self, nanoseconds=0):
                self.nanoseconds = nanoseconds

            @classmethod
            def from_msg(cls, msg):
                return cls(msg.sec*1000000000+msg.nanosec)

            def __sub__(self, other):
                return FakeTime(self.nanoseconds-other.nanoseconds)

        def publishers(topic):
            names = {'/map': 'map_server', '/amcl_pose': 'amcl', '/particle_cloud': 'amcl'}
            return [NS(node_name=names[topic], node_namespace='/')] if topic in names else []

        def client(_kind, topic):
            return NS(service_is_ready=lambda: topic == '/amcl/get_state',
                      call_async=lambda request: NS(done=lambda: True,
                                                   result=lambda: NS(current_state=NS(id=3))))

        node = NS(create_subscription=lambda kind, topic, callback, qos: callbacks.update({topic: callback}),
                  create_client=client, get_publishers_info_by_topic=publishers,
                  get_clock=lambda: NS(now=lambda: FakeTime(time.time_ns())), destroy_node=lambda: None)
        sent = False

        def spin_once(_node, timeout_sec):
            nonlocal sent
            if not sent:
                sent = True
                header = NS(frame_id='map', stamp=stamp())
                callbacks['/particle_cloud'](NS(header=header, particles=[NS(pose=pose, weight=1.)]))
                callbacks['/amcl_pose'](NS(header=header, pose=NS(pose=pose, covariance=covariance)))
            # Fresh read-only sensor messages; particle cloud is deliberately not repeated.
            callbacks['/scan'](NS())
            callbacks['/odom'](NS(pose=NS(pose=pose, covariance=covariance),
                                  twist=NS(twist=NS(linear=NS(x=0., y=0.), angular=NS(z=0.)))))
            time.sleep(timeout_sec)

        transform = NS(lookup_transform=lambda *args: NS(
            header=NS(stamp=stamp()), transform=NS(translation=pose.position, rotation=q)))
        modules = {
            'rclpy': NS(init=lambda: None, shutdown=lambda: None, ok=lambda: True,
                        create_node=lambda name: node, spin_once=spin_once),
            'rclpy.qos': NS(QoSProfile=lambda **kwargs: kwargs,
                            DurabilityPolicy=NS(TRANSIENT_LOCAL=1), qos_profile_sensor_data=object()),
            'rclpy.time': NS(Time=FakeTime),
            'nav_msgs.msg': NS(OccupancyGrid=object, Odometry=object, Path=object),
            'sensor_msgs.msg': NS(LaserScan=object),
            'geometry_msgs.msg': NS(PoseWithCovarianceStamped=object),
            'rcl_interfaces.srv': NS(GetParameters=NS(Request=lambda: NS())),
            'lifecycle_msgs.srv': NS(GetState=NS(Request=lambda: NS())),
            'nav2_msgs.msg': NS(ParticleCloud=object),
            'tf2_ros': NS(Buffer=lambda: transform, TransformListener=lambda *args: None),
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            now = time.time()
            (root/'epoch').write_text('odom-A')
            (root/'status.json').write_text(json.dumps({'running': True, 'time': now}))
            (root/'localization.json').write_text(json.dumps({
                'active': True, 'id': 'loc-A', 'initial_pose_stamp': now-1,
                'watchdog_time': now, 'epoch': 'odom-A'}))
            with patch.dict('sys.modules', modules), patch('sys.argv', ['remote_stream', '--duration', '.4']), \
                    patch.object(remote_stream, 'ODOMETRY_STATUS', root/'status.json'), \
                    patch.object(remote_stream, 'MAPPING_EPOCH', root/'odometry-epoch'), \
                    patch.object(remote_stream, 'NAVIGATION_STATUS', root/'navigation.json'), \
                    patch.object(remote_stream.signal, 'signal'), patch.object(remote_stream, 'emit', output.append):
                remote_stream.main()
        frames = [frame for frame in output if frame['type'] == 'frame']
        self.assertTrue(frames)
        self.assertEqual(frames[0]['map_mode'], 'saved')
        self.assertTrue(frames[0]['localization']['estimate_visible'])
        self.assertTrue(frames[0]['particles']['valid'])
        self.assertEqual(frames[0]['particles']['points'], [[1., 2., 0., 1.]])
        self.assertEqual(frames[0]['localization']['covariance_xy'], [.01, 0., .01])
        self.assertEqual(frames[0]['display_session'][3], 'loc-A')
        json.dumps(frames[0], allow_nan=False)


if __name__ == '__main__':
    unittest.main()
