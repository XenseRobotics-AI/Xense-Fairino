"""Offline map-rendering geometry and stale-data regressions (Agg only)."""
import base64
import math
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import patch
import zlib

import matplotlib
matplotlib.use('Agg')
from matplotlib.backends.backend_agg import FigureCanvasAgg
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from apps.map_viewer.map_rendering import COLORS, PoseTrail, covariance_ellipse
from apps.map_viewer.viewer import MapPlot, setup_plotting


def scene(pose=(1., 2., .3), stamp=100., session='A'):
    cells = np.zeros((80, 120), dtype=np.int8)
    cells[:2, :] = cells[-2:, :] = cells[:, :2] = cells[:, -2:] = 100
    cells[20:23, 20:70] = 100
    return {'received': time.monotonic(),
            'map': {'width': 120, 'height': 80, 'resolution': .1,
                    'origin': [-3., -2., 0.],
                    'data': base64.b64encode(zlib.compress(cells.tobytes())).decode()},
            'frame': {'time': stamp, 'map_mode': 'saved', 'pose': list(pose),
                      'display_session': {'map': 'map-A', 'localization': session},
                      'localization': {'active': True, 'ready': True, 'estimate_visible': True,
                                       'pose': [0., 1., .2], 'covariance_xy': [.04, .01, .02]},
                      'scans': {'/front/scan': [[2., 2.], [2.5, 2.]], '/back/scan': [[0., 1.]]},
                      'local_scans': {'/front/scan': [[1., 1.]], '/back/scan': [[-1., -1.]]},
                      'particles': {'points': [[0., 1., .2, .1], [.1, 1.1, .3, .2]],
                                    'age': .1, 'valid': True, 'count': 2, 'reason': ''},
                      'navigation_paths': {'global': {'points': [[1., 2.], [3., 3.]], 'age': .1, 'valid': True},
                                           'local': {'points': [[1., 2.], [1.5, 2.2]], 'age': .1, 'valid': True}}}}


class TrailTests(unittest.TestCase):
    def test_session_gap_and_relocalization_never_draw_impossible_motion(self):
        history = PoseTrail()
        history.add([0, 0, 0], 1., 'A')
        history.add([.1, 0, 0], 1.1, 'A')
        self.assertEqual(len(history.segments()), 1)
        history.add([.2, 0, 0], 1.2, 'B')
        history.add([4, 0, 0], 1.3, 'B')
        history.add([4.1, 0, 0], 4., 'B')
        history.pause()
        history.add([4.2, 0, 0], 4.1, 'B')
        self.assertEqual(len(history.segments()), 1)
        history.add([4.3, 0, 0], 4.2, 'B')
        self.assertEqual(len(history.segments()), 2)

    def test_stationary_frames_do_not_fill_history_or_create_artificial_gaps(self):
        history = PoseTrail(maximum=4)
        history.add([0, 0, 0], 0., 'A')
        for stamp in range(1, 20):
            history.add([0, 0, 0], float(stamp), 'A')
        self.assertEqual(len(history.samples), 1)
        for index in range(10):
            history.add([index*.1, 0, 0], 20.+index*.1, 'A')
        self.assertEqual(len(history.samples), 4)
        self.assertEqual(len(history.segments()), 3)
        history.clear()
        self.assertEqual(history.segments(), [])

    def test_invalid_pose_or_time_breaks_the_trail(self):
        history = PoseTrail()
        history.add([0, 0, 0], 1., 'A')
        history.add([.1, 0, 0], float('nan'), 'A')
        history.add([.2, 0, 0], 1.2, 'A')
        self.assertEqual(history.segments(), [])


class MapDrawingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        setup_plotting()

    def setUp(self):
        self.plot = MapPlot()
        self.plot.show_status_in_plot = False
        self.state = scene()

    def test_urdf_chassis_is_metric_and_heading_transform_follows_tf(self):
        p = self.plot
        self.state['frame']['pose'] = [1., 2., math.pi/2]
        p.update(self.state)
        shape = p.robot_shape
        np.testing.assert_allclose(np.ptp(shape.points, axis=0), [.75, .6])
        self.assertEqual(len(shape.wheels), 4)
        local_nose = shape.nose.get_xy()[0]
        displayed = p.ax.transData.inverted().transform(shape.nose.get_transform().transform(local_nose))
        np.testing.assert_allclose(displayed, [1., 2.475])
        self.assertEqual(shape.pose, [1., 2., math.pi/2])
        self.assertTrue(shape.halo.get_visible())
        self.assertFalse(p.robot.get_visible())

    def test_transport_dropout_retains_grey_last_body_but_clears_live_layers(self):
        p = self.plot
        p.update(self.state)
        self.state['received'] -= 10
        self.state['frame']['pose'] = [8., 8., 0.]
        p.update(self.state)
        self.assertEqual(p.robot_shape.pose, [1., 2., .3])
        self.assertTrue(p.robot_shape.body.get_visible())
        self.assertFalse(p.robot_shape.halo.get_visible())
        self.assertEqual(p.robot_shape.deck.get_facecolor()[:3], matplotlib.colors.to_rgb(COLORS['muted']))
        self.assertIn('停止更新', p.pose_note.get_text())
        self.assertTrue(p.connection_note.get_visible())
        for artist in (p.front, p.back, p.local_front, p.particles):
            self.assertEqual(len(artist.get_offsets()), 0)
        self.assertFalse(p.uncertainty.get_visible())
        self.assertEqual(len(p.global_path.get_xdata()), 0)

    def test_session_change_with_missing_pose_does_not_show_old_robot_on_new_map(self):
        p = self.plot
        p.update(self.state)
        self.state['frame'].update(display_session=['new-map', 'new-localization'], pose=None)
        p.update(self.state)
        self.assertFalse(p.robot_shape.body.get_visible())
        self.assertFalse(p.robot_focus())

    def test_amcl_ellipse_uses_observation_origin_not_current_tf_pose(self):
        p = self.plot
        p.update(self.state)
        self.assertTrue(p.uncertainty.get_visible())
        np.testing.assert_allclose(p.uncertainty.center, [0, 1])
        self.assertNotEqual(p.uncertainty.center, p.robot_shape.pose[:2])
        expected = covariance_ellipse([.04, .01, .02])
        np.testing.assert_allclose((p.uncertainty.width, p.uncertainty.height, p.uncertainty.angle), expected)
        for covariance in (None, [.1, 1., .1], [math.nan, 0., .1]):
            self.state['frame']['localization']['covariance_xy'] = covariance
            p.update(self.state)
            self.assertFalse(p.uncertainty.get_visible())

    def test_particle_positions_are_actual_observations_and_missing_is_not_synthesized(self):
        p = self.plot
        p.update(self.state)
        np.testing.assert_allclose(p.particles.get_offsets(), [[0., 1.], [.1, 1.1]])
        self.assertEqual(len(p.particle_headings.get_segments()), 2)
        self.state['frame']['particles']['valid'] = False
        p.update(self.state)
        self.assertEqual(len(p.particles.get_offsets()), 0)
        self.assertEqual(len(p.particle_headings.get_segments()), 0)

    def test_stationary_retained_particles_use_upstream_validity_and_show_age(self):
        p = self.plot
        item = self.state['frame']['particles']
        item.update(age=90., state='retained')
        p.update(self.state)
        self.assertEqual(len(p.particles.get_offsets()), 2)
        self.assertIn('静止保留', p.particle_note.get_text())
        self.assertIn('90.', p.particle_note.get_text())
        self.state['frame']['localization']['estimate_visible'] = False
        p.update(self.state)
        self.assertEqual(len(p.particles.get_offsets()), 0)
        self.assertFalse(p.particle_note.get_visible())

    def test_map_layers_are_independent_and_persist_across_updates(self):
        p = self.plot
        p.update(self.state)
        p.set_layer('particles', False)
        p.set_layer('scans', False)
        p.set_layer('uncertainty', False)
        p.set_layer('local_radar', False)
        p.set_layer('paths', False)
        p.set_layer('trail', False)
        p.update(self.state)
        for a in (p.particles, p.front, p.back, p.uncertainty, p.local_ax, p.path_artist, p.global_path, p.trail_artist):
            self.assertFalse(a.get_visible())
        self.assertTrue(p.robot_shape.body.get_visible())
        for name in p.layers:
            p.set_layer(name, True)
        self.assertTrue(p.local_ax.get_visible())
        self.assertTrue(p.uncertainty.get_visible())
        with self.assertRaises(ValueError):
            p.set_layer('invented', True)

    def test_preview_global_local_path_styles_and_transport_validity(self):
        p = self.plot
        p.update(self.state)
        p.set_navigation({'x': 3, 'y': 3, 'yaw': .4}, [[1, 2], [3, 3]], '预览')
        self.assertEqual(p.path_artist.get_linestyle(), '--')
        self.assertEqual(p.global_path.get_linestyle(), '-')
        self.assertNotEqual(p.global_path.get_color(), p.local_path.get_color())
        self.state['frame']['navigation_paths']['local']['valid'] = False
        p.update(self.state)
        self.assertEqual(len(p.local_path.get_xdata()), 0)
        self.assertEqual(len(p.global_path.get_xdata()), 2)
        self.assertEqual(len(p.path_artist.get_xdata()), 2)
        p.set_navigation()
        self.assertEqual(len(p.path_artist.get_xdata()), 0)
        self.assertIsNone(p.goal_arrow)

    def test_map_change_clears_trail_but_localization_change_fades_old_segments(self):
        p = self.plot
        f = self.state['frame']
        f['display_session'] = ['saved', 'map-A', None, 'loc-A', 1, 'odom-A']
        p.update(self.state)
        f.update(time=100.1, pose=[1.1, 2., .3])
        p.update(self.state)
        f['display_session'][3] = 'loc-B'
        f.update(time=100.2, pose=[1.2, 2., .3])
        p.update(self.state)
        self.assertEqual(len(p.trail_artist.get_segments()), 1)
        self.assertEqual(tuple(p.trail_artist.get_colors()[0][:3]), matplotlib.colors.to_rgb('#a6b0b7'))
        f['display_session'][1] = 'map-B'
        f.update(time=100.3, pose=[1.3, 2., .3])
        p.update(self.state)
        self.assertEqual(len(p.trail_artist.get_segments()), 0)

    def test_paths_switch_hides_target_marker_arrow_and_text_even_after_refresh(self):
        p = self.plot
        p.update(self.state)
        p.set_navigation({'x': 3., 'y': 3., 'yaw': 0.}, [[1., 2.], [3., 3.]], '待发送')
        p.set_layer('paths', False)
        for artist in (p.goal_marker, p.goal_arrow, p.goal_text):
            self.assertFalse(artist.get_visible())
        p.set_navigation({'x': 4., 'y': 3., 'yaw': 0.}, [[1., 2.], [4., 3.]], '待发送')
        p.update(self.state)
        for artist in (p.goal_marker, p.goal_arrow, p.goal_text):
            self.assertFalse(artist.get_visible())
        p.set_layer('paths', True)
        for artist in (p.goal_marker, p.goal_arrow, p.goal_text):
            self.assertTrue(artist.get_visible())
        p.set_navigation()
        self.assertFalse(p.goal_marker.get_visible())
        self.assertFalse(p.goal_text.get_visible())

    def test_rotated_map_bounds_zoom_focus_and_agg_render(self):
        p = self.plot
        self.state['map']['origin'] = [-3, -2, math.pi/2]
        p.update(self.state)
        np.testing.assert_allclose(p.bounds, [-11, -3, -2, 10], atol=1e-12)
        self.assertFalse(p.title.get_visible())
        self.assertIn(p.local_ax, p.ax.child_axes)
        before = np.ptp(p.ax.get_xlim())
        p.zoom(2)
        self.assertAlmostEqual(np.ptp(p.ax.get_xlim()), before/2)
        self.assertTrue(p.robot_focus())
        self.assertAlmostEqual(np.mean(p.ax.get_xlim()), 1.)
        self.assertAlmostEqual(np.mean(p.ax.get_ylim()), 2.)
        self.assertGreaterEqual(np.ptp(p.ax.get_xlim()), 5.)
        self.assertAlmostEqual(min(np.ptp(p.ax.get_xlim()), np.ptp(p.ax.get_ylim())), 5.)
        self.assertEqual(tuple(p.local_ax.get_xlim()), (-5., 5.))
        self.assertEqual(tuple(p.local_ax.get_ylim()), (-5., 5.))
        FigureCanvasAgg(p.figure).draw()
        self.assertIn('m', p.scale_text.get_text())
        with self.assertRaises(ValueError):
            p.zoom(float('nan'))


if __name__ == '__main__':
    unittest.main()
