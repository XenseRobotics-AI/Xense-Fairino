"""Console keeps its renderer while consuming the bundled SLAM protocol."""
import copy
import time
import unittest

from apps.map_viewer.demo import DemoStream
from apps.map_viewer.viewer import MapPlot, Stream, map_action
from apps.navigation.desktop import DriveSession, Target
from fairino_slam.mapping.viewer import Stream as SlamStream
from fairino_slam.mapping.client import execute as slam_map_action
from fairino_slam.navigation.desktop import DriveSession as SlamDrive


class BundledSlamIntegrationTests(unittest.TestCase):
    def test_console_uses_bundled_observer_actions_and_chassis_session(self):
        self.assertIs(Stream, SlamStream)
        self.assertIs(map_action, slam_map_action)
        self.assertIs(DriveSession, SlamDrive)
        frame = DemoStream().snapshot()['frame']
        target = Target()
        target.set([1., 2., 90.], frame)
        request = target.request(frame)
        self.assertEqual(request['heading_mode'], 'hold_initial')
        self.assertEqual(request['arrival_heading'], 1)
        self.assertEqual(request['session_id'], frame['localization']['id'])

    def test_actual_collision_contours_require_current_frame_and_map_revision(self):
        plot = MapPlot()
        state = copy.deepcopy(DemoStream().snapshot())
        state['received'] = time.monotonic()
        state['frame']['map_revision'] = state['map']['map_revision'] = 5
        state['frame']['frame'] = state['map']['frame'] = 'map'
        points = [[1., 1.], [2., 1.], [2., 2.], [1., 2.]]
        state['frame']['footprints'] = {
            'local': {'frame_id': 'map', 'age': 0., 'points': points}}
        plot.update(state)
        artist = plot.footprint_artists['local']
        self.assertTrue(artist.get_visible())
        self.assertEqual(list(artist.get_xdata()), [1., 2., 2., 1., 1.])
        plot.set_layer('footprints', False)
        self.assertFalse(artist.get_visible())
        plot.set_layer('footprints', True)
        state['frame']['map_revision'] = 6
        plot.update(state)
        self.assertFalse(artist.get_visible())
        self.assertEqual(len(artist.get_xdata()), 0)
        state['frame']['map_revision'] = 5
        state['received'] -= 3.
        plot.update(state)
        self.assertFalse(artist.get_visible())


if __name__ == '__main__':
    unittest.main()
