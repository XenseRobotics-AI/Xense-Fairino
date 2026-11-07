"""Named goals remain map-bound, persistent and explicit navigation drafts."""
import copy
import json
import math
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from fairino_slam.navigation.waypoints import WaypointStore, capture, fresh_frame, map_binding
from fairino_slam.navigation.desktop import Target


def snapshot():
    return {'received': 10., 'frame': {
        'map_mode': 'saved', 'saved_map_source': '/maps/room/map.yaml',
        'saved_map_modified': 100., 'frame': 'map', 'pose': [1.25, -2.5, math.pi/2],
        'localization': {'active': True, 'ready': True, 'id': 'A'}}}


def point(name='取料位'):
    return capture(name, snapshot(), 'robot', 'fairino', now=10.1)


class WaypointTests(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.store = WaypointStore(Path(folder.name)/'points.json')
        self.binding = point()['map']

    def test_multiple_points_survive_reload_and_localization_restart(self):
        self.store.save(point())
        second = point('放料位')
        second['pose'] = {'x': 3., 'y': 2., 'yaw': -1.2}
        self.store.save(second)
        restored = WaypointStore(self.store.path).list(self.binding)
        self.assertEqual([p['name'] for p in restored], ['取料位', '放料位'])
        frame = snapshot()['frame']
        frame['localization']['id'] = 'B'
        target = Target()
        target.set_saved(restored[1], frame)
        req = target.request(frame)
        self.assertEqual(req['session_id'], 'B')
        self.assertEqual(req['yaw'], -1.2)
        self.assertEqual(req['saved_pose'], 1)
        self.assertEqual((req['x'], req['y']), (3., 2.))
        self.assertIsNone(target.goal_id)

    def test_maps_versions_and_robots_have_separate_lists(self):
        self.store.save(point())
        for field, value in [('host', 'other'), ('user', 'other'), ('source', '/maps/new/map.yaml'),
                             ('modified', 101.)]:
            changed = dict(self.binding, **{field: value})
            self.assertEqual(self.store.list(changed), [])
            other = point()
            other['map'] = changed
            self.store.save(other)
            self.assertEqual(len(self.store.list(changed)), 1)

    def test_duplicate_names_do_not_overwrite_pose(self):
        original = point()
        self.store.save(original)
        duplicate = point()
        duplicate['pose']['x'] = 99.
        with self.assertRaisesRegex(ValueError, '同名'):
            self.store.save(duplicate)
        self.assertEqual(self.store.list(self.binding), [original])

    def test_bad_file_and_failed_replace_preserve_saved_data(self):
        self.store.save(point())
        before = self.store.path.read_bytes()
        with patch('fairino_slam.navigation.waypoints.os.replace', side_effect=OSError('disk')):
            with self.assertRaises(OSError):
                self.store.save(point('新点'))
        self.assertEqual(self.store.path.read_bytes(), before)
        self.assertEqual(list(self.store.path.parent.glob('.waypoints-*')), [])
        for text in ('invalid', '{"version":9,"points":[]}', '{"version":1,"points":[{}]}'):
            self.store.path.write_text(text)
            with self.assertRaises(ValueError):
                self.store.save(point('新点'))
            self.assertEqual(self.store.path.read_text(), text)

    def test_concurrent_panels_do_not_lose_saved_points(self):
        errors = []
        def save(index):
            try:
                WaypointStore(self.store.path).save(point(str(index)))
            except Exception as exc:
                errors.append(exc)
        threads = [threading.Thread(target=save, args=(i,)) for i in range(8)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        self.assertEqual(errors, [])
        self.assertEqual(len(self.store.list(self.binding)), 8)

    def test_capture_requires_live_ready_valid_pose(self):
        changes = [
            lambda s: s.update(received=7.),
            lambda s: s.update(received=11.),
            lambda s: s.update(error='disconnected'),
            lambda s: s['frame']['localization'].update(ready=False),
            lambda s: s['frame']['localization'].update(active=False),
            lambda s: s['frame'].update(pose=[float('nan'), 0., 0.]),
            lambda s: s['frame'].update(pose=None),
            lambda s: s['frame'].update(saved_map_modified=None),
            lambda s: s['frame'].update(map_mode='mapping'),
        ]
        for change in changes:
            state = snapshot()
            change(state)
            with self.subTest(state=state), self.assertRaises(ValueError):
                capture('点', state, 'robot', 'fairino', now=10.1)
        for name in ('', '  ', 'x'*65, 'x\ny'):
            with self.assertRaises(ValueError):
                point(name)

    def test_saved_target_rejects_session_or_map_version_change_before_send(self):
        for changed in ({'saved_map_modified': 101.},
                        {'localization': {'active': True, 'id': 'B'}},
                        {'saved_map_source': '/maps/other/map.yaml'}):
            frame = snapshot()['frame']
            target = Target()
            target.set_saved(point(), frame)
            with self.assertRaises(ValueError):
                target.request(dict(frame, **changed))
            self.assertIsNone(target.pose)

    def test_manual_edit_clears_saved_heading_and_map_metadata(self):
        frame = snapshot()['frame']
        target = Target()
        target.set_saved(point(), frame)
        target.set(['3', '4', '30'], frame)
        request = target.request(frame)
        self.assertNotIn('saved_pose', request)
        self.assertAlmostEqual(request['yaw'], math.pi/6)
        self.assertEqual(request['arrival_heading'], 1)

    def test_delete_only_selected_record_in_selected_map(self):
        first, second = point(), point('另一个点')
        self.store.save(first)
        self.store.save(second)
        with self.assertRaises(ValueError):
            self.store.delete(first['id'], dict(self.binding, host='other'))
        self.assertEqual(self.store.delete(first['id'], self.binding), [second])
