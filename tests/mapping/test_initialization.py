"""Stationary initialization and visible estimates must not bypass navigation readiness."""
import math
import sys
import unittest
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import Mock, patch
import fairino_slam.mapping.localization as loc
from . import test_localization as fixtures


class RefinementTests(unittest.TestCase):
    def setUp(self):
        module = ModuleType('std_srvs.srv')
        module.Empty = NS(Request=lambda: object())
        parent = ModuleType('std_srvs')
        parent.srv = module
        self.patcher = patch.dict(sys.modules, {'std_srvs': parent, 'std_srvs.srv': module})
        self.patcher.start()
        self.addCleanup(self.patcher.stop)
        self.ros = NS(latest={}, call=Mock())
        self.refine = loc.InitialPoseRefinement()
        self.state = {'id': 'A', 'initial_pose_stamp': 100., 'active': True,
                      'initialization': {'state': 'refining', 'updates': 0, 'maximum': 6}}

    def scan(self, now):
        self.ros.latest = {'/scan': (now, {}), '/odom': (now, {'vx': 0., 'vy': 0., 'wz': 0.})}

    def test_fresh_scans_drive_six_bounded_requests_without_motion_commands(self):
        for i in range(6):
            self.scan(float(i))
            self.assertTrue(self.refine.step(self.ros, self.state, float(i)))
        self.assertEqual(self.ros.call.call_count, 6)
        self.assertEqual(self.state['initialization']['state'], 'completed')
        self.assertEqual(self.state['initialization']['updates'], 6)
        self.scan(8.)
        self.assertFalse(self.refine.step(self.ros, self.state, 8.))
        for call in self.ros.call.call_args_list:
            self.assertEqual(call.args[1], '/request_nomotion_update')
        self.assertTrue(self.state['active'])

    def test_no_request_without_initial_pose_or_fresh_distinct_scan(self):
        self.assertFalse(self.refine.step(self.ros, self.state, 0.))
        self.scan(1.)
        self.assertTrue(self.refine.step(self.ros, self.state, 1.))
        self.assertFalse(self.refine.step(self.ros, self.state, 1.2))
        self.assertFalse(self.refine.step(self.ros, self.state, 2.))
        self.state['initial_pose_stamp'] = None
        self.scan(3.)
        self.assertFalse(self.refine.step(self.ros, self.state, 3.))
        self.assertEqual(self.ros.call.call_count, 1)

    def test_failed_refinement_does_not_erase_accepted_initial_pose(self):
        self.scan(1.)
        self.ros.call.side_effect = RuntimeError('timeout')
        self.assertTrue(self.refine.step(self.ros, self.state, 1.))
        self.assertEqual(self.state['initial_pose_stamp'], 100.)
        self.assertTrue(self.state['active'])
        self.assertEqual(self.state['initialization']['state'], 'failed')

    def test_motion_ends_stationary_requests_without_driving_robot(self):
        self.scan(1.)
        self.ros.latest['/odom'][1]['vx'] = .06
        self.assertTrue(self.refine.step(self.ros, self.state, 1.))
        self.ros.call.assert_not_called()
        self.assertEqual(self.state['initialization']['state'], 'completed')

    def test_new_pose_restarts_window_and_old_window_has_finite_duration(self):
        self.scan(1.)
        self.refine.step(self.ros, self.state, 1.)
        self.assertTrue(self.refine.step(self.ros, self.state, 13.))
        self.assertEqual(self.state['initialization']['state'], 'completed')
        self.state.update(initial_pose_stamp=120., initialization={'state': 'refining', 'updates': 0, 'maximum': 6})
        self.scan(14.)
        self.assertTrue(self.refine.step(self.ros, self.state, 14.))
        self.assertEqual(self.state['initialization']['updates'], 1)


class CandidateDisplayTests(unittest.TestCase):
    def setUp(self):
        self.base = fixtures.MonitoringTests()
        self.base.setUp()

    def test_uncertain_pose_can_be_drawn_but_cannot_navigate(self):
        b = self.base
        cov = b.cov.copy(); cov[0] = 1.
        b.accept(cov=cov)
        status = b.status()
        self.assertTrue(status['estimate_visible'])
        self.assertFalse(status['ready'])
        b.odom['running'] = False
        self.assertFalse(b.status()['estimate_visible'])

    def test_initialization_blocks_navigation_even_with_small_covariance(self):
        b = self.base
        b.monitor.session['initialization'] = {'state': 'refining', 'updates': 2, 'maximum': 6}
        b.accept()
        self.assertTrue(b.status()['estimate_visible'])
        self.assertFalse(b.status()['ready'])
        self.assertEqual(b.status()['state'], 'initializing')
        b.monitor.session['initialization']['state'] = 'completed'
        self.assertTrue(b.status()['ready'])
        b.monitor.session['initialization']['state'] = 'failed'
        self.assertFalse(b.status()['ready'])

    def test_stale_moving_candidate_is_hidden_and_unique_updates_counted(self):
        b = self.base
        b.accept()
        self.assertFalse(b.accept())
        b.accept(101.1)
        self.assertEqual(b.status()['matches'], 2)
        b.monitor.on_odometry([0., 0., 0.], [0., 0., 0.])
        b.monitor.on_odometry([.11, 0., 0.], [.06, 0., 0.])
        b.monitor.session['watchdog_time'] = 104.
        self.assertTrue(b.status(104.)['estimate_visible'])
        b.monitor.session['watchdog_time'] = 106.1
        self.assertFalse(b.status(106.1)['estimate_visible'])


class CandidateDrawingTests(unittest.TestCase):
    def test_candidate_is_amber_and_hidden_when_transport_stales(self):
        import time
        import matplotlib
        matplotlib.use('Agg')
        from fairino_slam.mapping.viewer import MapPlot
        plot = MapPlot()
        frame = {'time': time.time(), 'map_mode': 'saved', 'pose': [1., 2., .3],
                 'localization': {'ready': False, 'estimate_visible': True}}
        state = {'frame': frame, 'received': time.monotonic()}
        plot.update(state)
        self.assertTrue(plot.pose_note.get_visible())
        self.assertEqual(plot.robot.get_color(), '#d18a16')
        frame['localization']['ready'] = True
        plot.update(state)
        self.assertFalse(plot.pose_note.get_visible())
        self.assertEqual(plot.robot.get_color(), '#c53451')
        state['received'] -= 10
        plot.update(state)
        self.assertEqual(len(plot.robot.get_xdata()), 0)


if __name__ == '__main__':
    unittest.main()
