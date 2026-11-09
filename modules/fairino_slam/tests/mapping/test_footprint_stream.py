"""Actual Nav2 polygons are transformed for display without inventing a footprint."""
from contextlib import redirect_stdout
import io
import json
import math
import struct
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from fairino_slam.deployment.build_navigation import build
from fairino_slam.configuration import load_navigation
from fairino_slam.mapping.remote_stream import FOOTPRINT_TOPICS, PREFIX, FootprintObserver, emit
from fairino_slam.mapping.viewer import Stream


def build_circle_profile():
    settings = load_navigation()
    settings['nav2']['controller_server']['ros__parameters']['FollowPath']['heading_mode'] = 'hold_initial'
    with patch('fairino_slam.navigation.profile.load_navigation', return_value=settings):
        return build()


def polygon(stamp=1000., frame='map', points=None):
    points = points if points is not None else ((1., 2., 0.), (3., 2., 0.), (3., 4., 0.), (1., 4., 0.))
    sec = int(stamp)
    return NS(header=NS(stamp=NS(sec=sec, nanosec=round((stamp-sec)*1e9)), frame_id=frame),
              polygon=NS(points=[NS(x=p[0], y=p[1], z=p[2]) for p in points]))


def transform(x=0., y=0., angle=0.):
    return NS(transform=NS(translation=NS(x=x, y=y, z=0.),
                           rotation=NS(x=0., y=0., z=math.sin(angle/2), w=math.cos(angle/2))))


class FootprintStreamTests(unittest.TestCase):
    def setUp(self):
        self.observer = FootprintObserver()
        self.observer.set_context(('saved', 'map', 1, 'session', 990., 'epoch'))

    def snapshot(self, **kwargs):
        args = dict(now=100.2, now_ros=1000.2, map_frame='map', pose_visible=True,
                    cutoff=990., lookup=lambda *args: transform())
        args.update(kwargs)
        return self.observer.snapshot(**args)

    def test_map_polygon_keeps_actual_padded_world_coordinates(self):
        self.assertTrue(self.observer.accept('global', polygon(), 100.))
        def forbidden_lookup(*args):
            self.fail('Map-frame footprint must not be transformed by robot pose again')
        result = self.snapshot(lookup=forbidden_lookup)
        entry = result['global']
        self.assertEqual(entry['points'], [[1., 2.], [3., 2.], [3., 4.], [1., 4.]])
        self.assertEqual(entry['source'], FOOTPRINT_TOPICS['global'])
        self.assertEqual(entry['source_frame'], 'map')
        self.assertEqual(entry['frame_id'], 'map')
        self.assertEqual(entry['stamp'], 1000.)
        self.assertAlmostEqual(entry['age'], .2)

    def test_generated_circle_survives_map_frame_and_json_transport_at_vertex_limit(self):
        config = build_circle_profile()['global_costmap']['global_costmap']['ros__parameters']
        circle = json.loads(config['footprint'])
        self.assertEqual(len(circle), 64)
        self.assertEqual(config['footprint_padding'], 0.)
        self.assertNotEqual(circle[0], circle[-1])
        c, s = math.cos(.37), math.sin(.37)
        # Nav2 publishes already padded/oriented Point32 vertices, without a
        # repeated closing point. Simulate that ROS wire precision explicitly.
        point32 = lambda value: struct.unpack('f', struct.pack('f', value))[0]
        world = [[point32(1.2+c*x-s*y), point32(-.7+s*x+c*y)] for x, y in circle]
        message = polygon(points=[(*p, 0.) for p in world])
        self.assertTrue(self.observer.accept('global', message, 100.))
        def forbidden_lookup(*args):
            self.fail('A published map-frame circle must not receive a second pose transform')
        footprints = self.snapshot(lookup=forbidden_lookup)
        self.assertEqual(footprints['global']['points'], world)
        wire = io.StringIO()
        with redirect_stdout(wire):
            emit({'type': 'frame', 'footprints': footprints})
        line = wire.getvalue().strip()
        self.assertTrue(line.startswith(PREFIX))
        received = json.loads(line[len(PREFIX):])
        # Constructing Stream does not start its SSH thread.
        stream = Stream(NS(), None)
        self.assertFalse(stream.thread.is_alive())
        with patch('fairino_slam.mapping.viewer.time.monotonic', return_value=100.2):
            stream.put(received)
        cached = stream.snapshot()['frame']['footprints']['global']['points']
        self.assertEqual(len(cached), 64)
        self.assertEqual(cached, world)

    def test_generated_circle_is_transformed_once_without_padding_or_vertex_loss(self):
        circle = json.loads(build_circle_profile()['global_costmap']['global_costmap']['ros__parameters']['footprint'])
        c, s = math.cos(.31), math.sin(.31)
        source = [[1.+c*x-s*y, 2.+s*x+c*y] for x, y in circle]
        message = polygon(frame='odom', points=[(*p, 0.) for p in source])
        self.assertTrue(self.observer.accept('global', message, 100.))
        calls = []
        angle = -.42
        def lookup(target, frame, stamp):
            calls.append((target, frame, stamp))
            return transform(.3, -1., angle)
        result = self.snapshot(lookup=lookup)['global']['points']
        c, s = math.cos(angle), math.sin(angle)
        expected = [[.3+c*x-s*y, -1.+s*x+c*y] for x, y in source]
        self.assertEqual(len(result), 64)
        self.assertEqual(calls, [('map', 'odom', message.header.stamp)])
        for actual, wanted in zip(result, expected):
            self.assertAlmostEqual(actual[0], wanted[0], places=12)
            self.assertAlmostEqual(actual[1], wanted[1], places=12)

    def test_odom_polygon_uses_transform_at_message_stamp(self):
        msg = polygon(frame='odom')
        self.observer.accept('local', msg, 100.)
        calls = []
        def lookup(target, source, stamp):
            calls.append((target, source, stamp))
            return transform(10., -2., math.pi/2)
        result = self.snapshot(lookup=lookup)['local']
        expected = [[8., -1.], [8., 1.], [6., 1.], [6., -1.]]
        for got, want in zip(result['points'], expected):
            self.assertAlmostEqual(got[0], want[0])
            self.assertAlmostEqual(got[1], want[1])
        self.assertEqual(calls, [('map', 'odom', msg.header.stamp)])
        self.assertEqual(result['source_frame'], 'odom')

    def test_replaces_shape_when_robot_moves_in_world(self):
        self.observer.accept('global', polygon(), 100.)
        moved = ((2., 3., 0.), (4., 3., 0.), (4., 5., 0.), (2., 5., 0.))
        self.observer.accept('global', polygon(1000.1, points=moved), 100.1)
        self.assertEqual(self.snapshot()['global']['points'], [list(p[:2]) for p in moved])

    def test_both_stamp_and_receipt_age_must_be_fresh(self):
        self.observer.accept('global', polygon(), 100.)
        for kwargs in ({'now_ros': 1001.0001}, {'now': 101.0001},
                       {'now_ros': 999.94}, {'now': 99.99},
                       {'now_ros': float('nan')}, {'cutoff': float('inf')}):
            with self.subTest(kwargs=kwargs):
                self.assertEqual(self.snapshot(**kwargs), {})
        # Inclusive display TTL is independent of stricter motion gate limits.
        self.assertIn('global', self.snapshot(now=101., now_ros=1001.))

    def test_hidden_pose_and_session_cutoff_do_not_show_old_polygon(self):
        self.observer.accept('global', polygon(), 100.)
        self.assertEqual(self.snapshot(pose_visible=False), {})
        self.assertEqual(self.snapshot(cutoff=1000.01), {})
        for token in (('saved', 'map', 1, 'new-session', 1000.1, 'epoch'),
                      ('saved', 'map', 2, 'new-session', 1000.1, 'epoch'),
                      ('mapping', 'map', 2, None, None, 'new-epoch')):
            self.observer.set_context(token)
            self.assertEqual(self.snapshot(), {})
            self.observer.accept('global', polygon(), 100.)
        self.observer.clear()
        self.assertEqual(self.snapshot(), {})

    def test_unchanged_context_preserves_fresh_polygon(self):
        self.observer.accept('global', polygon(), 100.)
        self.observer.set_context(('saved', 'map', 1, 'session', 990., 'epoch'))
        self.assertIn('global', self.snapshot())

    def test_latest_missing_tf_can_use_fresh_previous_sample(self):
        first = polygon(frame='odom')
        second = polygon(1000.1, frame='odom')
        self.observer.accept('local', first, 100.)
        self.observer.accept('local', second, 100.1)
        calls = []
        def lookup(target, source, stamp):
            calls.append(stamp)
            if stamp is second.header.stamp:
                raise RuntimeError('extrapolation into future')
            return transform()
        result = self.snapshot(lookup=lookup)
        self.assertEqual(result['local']['stamp'], 1000.)
        self.assertEqual(calls, [second.header.stamp, first.header.stamp])
        def missing(*args):
            raise RuntimeError('no transform')
        self.assertEqual(self.snapshot(lookup=missing), {})

    def test_invalid_or_degenerate_polygons_are_rejected(self):
        bad = [polygon(frame=''), polygon(frame='/map'), polygon(stamp=0),
               polygon(points=((0, 0, 0), (1, 1, 0), (2, 2, 0))),
               polygon(points=((0, 0, 0), (1, 0, 0))),
               polygon(points=((0, 0, 0), (1, 0, 0), (0, float('nan'), 0))),
               polygon(points=((0, 0, 0), (1, 0, 0), (0, 1, float('inf')))),
               polygon(points=((0, 0, 0), (1, 0, 0), (0, 1, True)))]
        for msg in bad:
            with self.subTest(msg=msg):
                self.assertFalse(self.observer.accept('local', msg, 100.))
        self.assertFalse(self.observer.accept('unrecognised', polygon(), 100.))
        self.assertFalse(self.observer.accept('global', polygon(), float('nan')))
        self.assertEqual(self.snapshot(), {})

    def test_closed_polygon_accepted_and_replay_does_not_replace_newer_sample(self):
        msg = polygon(points=((0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 0)))
        self.assertTrue(self.observer.accept('global', msg, 100.))
        self.assertEqual(len(self.snapshot()['global']['points']), 3)
        self.assertFalse(self.observer.accept('global', polygon(999.9), 100.1))
        self.assertEqual(self.snapshot()['global']['stamp'], 1000.)

    def test_buffers_and_polygon_payloads_are_bounded(self):
        many = tuple((math.cos(t/100), math.sin(t/100), 0.) for t in range(65))
        self.assertFalse(self.observer.accept('global', polygon(points=many), 100.))
        for i in range(20):
            self.assertTrue(self.observer.accept('global', polygon(999.+i*.05), 99.+i*.05))
        self.assertEqual(len(self.observer.pending['global']), 4)

    def test_invalid_transform_and_degenerate_projection_are_hidden(self):
        self.observer.accept('local', polygon(frame='odom'), 100.)
        cases = [transform(), transform()]
        cases[0].transform.translation.x = float('nan')
        cases[1].transform.rotation.w = 2.
        edge_on = transform()
        edge_on.transform.rotation = NS(x=0., y=math.sin(math.pi/4), z=0., w=math.cos(math.pi/4))
        cases.append(edge_on)
        for tf in cases:
            with self.subTest(tf=tf):
                self.assertEqual(self.snapshot(lookup=lambda *args: tf), {})


if __name__ == '__main__':
    unittest.main()
