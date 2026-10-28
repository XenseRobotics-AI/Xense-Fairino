"""Offline regressions for unsent pose arrows; no robot connections."""
import math
import unittest

import matplotlib
matplotlib.use('Agg')
from matplotlib.backends.backend_agg import FigureCanvasAgg
import numpy as np

from apps.map_viewer.test_rendering import scene
from apps.map_viewer.viewer import MapPlot, setup_plotting


class PosePreviewDrawingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        setup_plotting()

    def setUp(self):
        self.plot = MapPlot()
        self.plot.show_status_in_plot = False
        self.state = scene()
        self.plot.update(self.state)

    def test_press_shows_origin_without_inventing_heading(self):
        p = self.plot
        self.assertTrue(p.set_pose_preview((2., 3.)))
        self.assertIsNone(p.pose_preview.pose)
        self.assertTrue(p.pose_preview.origin.get_visible())
        self.assertTrue(p.pose_preview.note.get_visible())
        self.assertFalse(p.pose_preview.arrow.get_visible())
        self.assertFalse(p.pose_preview.body.get_visible())
        self.assertIn('尚未发送', p.pose_preview.note.get_text())
        self.assertTrue(p.set_pose_preview((2., 3.), (2.01, 3.)))
        self.assertIsNone(p.pose_preview.pose)

    def test_drag_rotates_metric_outline_and_labels_heading(self):
        p = self.plot
        self.assertTrue(p.set_pose_preview((2., 3.), (2., 5.)))
        preview = p.pose_preview
        self.assertEqual(preview.pose, [2., 3., math.pi/2])
        self.assertTrue(preview.arrow.get_visible())
        self.assertTrue(preview.body.get_visible())
        displayed = p.ax.transData.inverted().transform(preview.body.get_transform().transform(preview.body.get_xy()))
        np.testing.assert_allclose(np.ptp(displayed, axis=0), [.6, .75])
        self.assertIn('90.0°', preview.note.get_text())
        self.assertIn('箭头长度仅表示方向', preview.note.get_text())
        self.assertGreater(preview.arrow.get_zorder(), p.robot_shape.nose.get_zorder())

    def test_preview_does_not_move_live_robot_trail_or_map_limits(self):
        p = self.plot
        robot_before = p.robot_shape.pose[:]
        trail_before = list(p.trail.samples)
        limits_before = (p.ax.get_xlim(), p.ax.get_ylim())
        bounds_before = p.bounds
        p.set_pose_preview((30., 40.), (35., 41.))
        self.assertEqual(p.robot_shape.pose, robot_before)
        self.assertEqual(list(p.trail.samples), trail_before)
        self.assertEqual((p.ax.get_xlim(), p.ax.get_ylim()), limits_before)
        self.assertEqual(p.bounds, bounds_before)
        self.assertIsNone(p.navigation_pose)
        p.fit()
        self.assertLess(p.ax.get_xlim()[1], 30.)

    def test_live_updates_and_layers_preserve_drag_preview(self):
        p = self.plot
        p.set_pose_preview((2., 3.), (4., 4.))
        expected = p.pose_preview.pose[:]
        self.state['frame'].update(time=100.1, pose=[1.1, 2., .4])
        p.set_layer('paths', False)
        p.update(self.state)
        self.assertEqual(p.pose_preview.pose, expected)
        self.assertTrue(all(a.get_visible() for a in p.pose_preview.artists))
        self.assertEqual(p.robot_shape.pose, [1.1, 2., .4])
        FigureCanvasAgg(p.figure).draw()

    def test_navigation_preview_is_red_and_clears_all_artists(self):
        p = self.plot
        p.set_pose_preview((2., 3.), (3., 3.), kind='goal')
        self.assertIn('导航目标预览', p.pose_preview.note.get_text())
        self.assertEqual(p.pose_preview.arrow.get_edgecolor()[:3], matplotlib.colors.to_rgb('#c74740'))
        p.clear_pose_preview()
        self.assertIsNone(p.pose_preview.pose)
        self.assertFalse(any(a.get_visible() for a in p.pose_preview.artists))
        self.assertEqual(p.pose_preview.note.get_text(), '')

    def test_nonfinite_or_malformed_coordinates_clear_preview(self):
        p = self.plot
        for start, end in [((math.nan, 0), (1, 0)), ((0, 0), (math.inf, 0)),
                           ((0, 0, 0), None), ('bad', None), ((0, 0), [1]),
                           ((-1e308, 0), (1e308, 0))]:
            with self.subTest(start=start, end=end):
                p.set_pose_preview((2., 3.), (3., 4.))
                self.assertFalse(p.set_pose_preview(start, end))
                self.assertFalse(any(a.get_visible() for a in p.pose_preview.artists))
        with self.assertRaises(ValueError):
            p.set_pose_preview((0, 0), (1, 1), kind='invalid')


if __name__ == '__main__':
    unittest.main()
