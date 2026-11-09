"""Map overlays use live footprint coordinates and disappear when evidence expires."""
import base64
import copy
import json
import math
import unittest
from unittest.mock import patch
import zlib

import matplotlib
matplotlib.use('Agg')
from fairino_slam.deployment.build_navigation import build
from fairino_slam.configuration import load_navigation
from fairino_slam.mapping.viewer import MapPlot, setup_plotting


def build_circle_profile():
    settings = load_navigation()
    settings['nav2']['controller_server']['ros__parameters']['FollowPath']['heading_mode'] = 'hold_initial'
    with patch('fairino_slam.navigation.profile.load_navigation', return_value=settings):
        return build()


class FootprintPlotTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        setup_plotting()

    def setUp(self):
        self.plot = MapPlot()
        self.points = [[.6, 1.7], [1.4, 1.7], [1.4, 2.3], [.6, 2.3]]
        item = {'points': self.points, 'age': .1, 'stamp': 99.9, 'frame_id': 'map'}
        self.state = {
            'received': 100.,
            'map': {'width': 80, 'height': 80, 'resolution': .1,
                    'origin': [-2., -2., .3], 'frame': 'map', 'map_revision': 1,
                    'data': base64.b64encode(zlib.compress(bytes(80*80))).decode()},
            'frame': {'time': 100., 'map_mode': 'saved', 'frame': 'map', 'map_revision': 1,
                      'pose': [1., 2., math.pi/2], 'localization': {'ready': True},
                      'footprints': {'local': copy.deepcopy(item),
                                     'global': copy.deepcopy(item)}}}

    def update(self, state=None, now=100.):
        with patch('fairino_slam.mapping.viewer.time.monotonic', return_value=now):
            self.plot.update(self.state if state is None else state)

    def visible(self):
        return {name for name, artist in self.plot.footprint_artists.items()
                if artist.get_visible()}

    def test_world_points_are_not_transformed_again_by_robot_or_map_origin(self):
        self.update()
        artist = self.plot.footprint_artists['local']
        self.assertEqual(self.visible(), {'local', 'global'})
        self.assertEqual(list(zip(artist.get_xdata(), artist.get_ydata())),
                         [tuple(p) for p in self.points+self.points[:1]])

    def test_generated_circle_and_legacy_square_draw_without_extra_pose_or_padding(self):
        self.update()
        global_artist = self.plot.footprint_artists['global']
        circle = json.loads(build_circle_profile()['global_costmap']['global_costmap']['ros__parameters']['footprint'])
        self.assertEqual(len(circle), 64)
        c, s = math.cos(.41), math.sin(.41)
        world = [[1.+c*x-s*y, 2.+s*x+c*y] for x, y in circle]
        self.state['frame']['footprints']['global']['points'] = world
        self.update()
        self.assertEqual(self.visible(), {'local', 'global'})
        self.assertIs(self.plot.footprint_artists['global'], global_artist)
        actual = list(zip(global_artist.get_xdata(), global_artist.get_ydata()))
        self.assertEqual(len(actual), 65)  # 64 vertices plus only the closing point.
        self.assertEqual(actual, [tuple(p) for p in world+world[:1]])
        local_artist = self.plot.footprint_artists['local']
        self.assertEqual(list(zip(local_artist.get_xdata(), local_artist.get_ydata())),
                         [tuple(p) for p in self.points+self.points[:1]])
        # Exercise the renderer as well as its stored data; no display or robot.
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        FigureCanvasAgg(self.plot.figure).draw()
        # Older four-vertex global footprints remain compatible and replace
        # the circle completely, without leaving any of its vertices behind.
        self.state['frame']['footprints']['global']['points'] = self.points
        self.update()
        self.assertIs(self.plot.footprint_artists['global'], global_artist)
        self.assertEqual(list(zip(global_artist.get_xdata(), global_artist.get_ydata())),
                         [tuple(p) for p in self.points+self.points[:1]])

    def test_motion_updates_existing_outline_without_leaving_old_vertices(self):
        self.update()
        artist = self.plot.footprint_artists['local']
        moved = [[3.3, 3.6], [3.3, 4.4], [2.7, 4.4], [2.7, 3.6]]
        self.state['frame']['pose'] = [3., 4., math.pi/2]
        self.state['frame']['footprints']['local']['points'] = moved
        self.update()
        self.assertIs(self.plot.footprint_artists['local'], artist)
        self.assertEqual(list(artist.get_xdata()), [p[0] for p in moved+moved[:1]])
        self.assertEqual(list(artist.get_ydata()), [p[1] for p in moved+moved[:1]])

    def test_each_polygon_expires_with_transport_delay_included(self):
        self.state['frame']['footprints']['local']['age'] = .85
        self.update(now=100.2)
        self.assertEqual(self.visible(), {'global'})
        self.update(now=101.1)
        self.assertEqual(self.visible(), set())

    def test_pose_loss_map_loss_and_disconnect_remove_previous_outlines(self):
        for kind in ('pose', 'map', 'error', 'old_stream'):
            with self.subTest(kind=kind):
                self.update()
                state = copy.deepcopy(self.state)
                if kind == 'pose':
                    state['frame']['pose'] = None
                elif kind == 'map':
                    state['frame']['map_mode'] = 'empty'
                elif kind == 'error':
                    state['error'] = {'message': 'SSH disconnected'}
                else:
                    state['received'] = 90.
                self.update(state)
                self.assertEqual(self.visible(), set())
                for artist in self.plot.footprint_artists.values():
                    self.assertEqual(len(artist.get_xdata()), 0)

    def test_new_map_hides_old_frame_until_matching_revision_arrives(self):
        self.update()
        state = copy.deepcopy(self.state)
        state['map']['map_revision'] = 2
        self.update(state)
        self.assertEqual(self.visible(), set())
        state['frame']['map_revision'] = 2
        self.update(state)
        self.assertEqual(self.visible(), {'local', 'global'})

    def test_unmatched_coordinate_frames_and_invalid_vertices_do_not_draw(self):
        self.update()
        state = copy.deepcopy(self.state)
        state['frame']['footprints']['local']['frame_id'] = 'odom'
        state['frame']['footprints']['global']['points'][0][0] = float('nan')
        self.update(state)
        self.assertEqual(self.visible(), set())

    def test_older_stream_without_footprints_keeps_map_and_pose_working(self):
        self.state['frame'].pop('footprints')
        self.state['frame'].pop('map_revision')
        self.state['map'].pop('map_revision')
        self.update()
        self.assertEqual(self.visible(), set())
        self.assertIsNotNone(self.plot.map_artist)
        self.assertEqual(list(self.plot.robot.get_xdata()), [1.])


if __name__ == '__main__':
    unittest.main()
