"""Offline checks against the actual optional TRON2 C++ implementation."""
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
from dataclasses import replace
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'apps/fairino_tcp_probe')]

import numpy as np
from tcp_core import BOUNDS_DEG, multiply, pose_error, tcp_forward_matrix
from tool_frames import GEOMETRY_ID, for_arm, tool_config
from tools.ik_experiments.tron2_art7 import Art7TronIK, dependencies_available, model_xml


class ModelDefinitionTests(unittest.TestCase):
    def test_generated_model_has_art7_dimensions_and_fourteen_active_joints(self):
        root = ET.fromstring(model_xml(tool_config()))
        active = root.findall("joint[@type='revolute']")
        self.assertEqual(len(active), 14)
        self.assertEqual(root.find("joint[@name='proximal_pitch_L_Joint']/origin").get('xyz'),
                         '0 0 0.187')
        self.assertAlmostEqual(float(root.find("joint[@name='wrist_pitch_L_Joint']/limit").get('upper')),
                               math.radians(70.))

    def test_missing_optional_build_reports_setup_without_fallback(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch('tools.ik_experiments.tron2_art7.CACHE', Path(directory)):
                self.assertFalse(dependencies_available())
                with self.assertRaisesRegex(RuntimeError, 'setup_tron2_ik.py --install'):
                    Art7TronIK(tool_config())


@unittest.skipUnless(dependencies_available(), 'Optional TRON2 native dependencies/build absent')
class NativeModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tools = tool_config()
        cls.solver = Art7TronIK(cls.tools)

    def test_random_both_arm_forward_agrees_with_independent_dh(self):
        random = np.random.default_rng(90210)
        bounds = np.radians(np.asarray(BOUNDS_DEG))
        for side in ('left', 'right'):
            for q in random.uniform(bounds[:, 0], bounds[:, 1], (100, 7)):
                expected = multiply(tcp_forward_matrix(q.tolist()), for_arm(self.tools, side))
                actual = self.solver.forward(side, q)
                mm, deg = pose_error(expected, actual)
                self.assertLess(mm, 1e-8)
                self.assertLess(deg, 5e-6)

    def test_reachable_local_motion_and_fixed_unrequested_arm(self):
        random = np.random.default_rng(434)
        references = dict(left=np.radians([30., 45., -80., 60., 80., 10., -15.]).tolist(),
                          right=np.radians([-30., -45., 80., 60., -80., -10., 15.]).tolist())
        for i in range(60):
            sides = ('left', 'right') if i % 3 == 0 else (('left',) if i % 3 == 1 else ('right',))
            goals = {side: self.solver.forward(side, np.asarray(references[side])+
                                             random.uniform(-.015, .015, 7)) for side in sides}
            result = self.solver.solve(goals, references)
            self.assertEqual(set(result), set(sides))
            for side, entry in result.items():
                mm, deg = pose_error(goals[side], entry['tcp'])
                self.assertTrue(entry['success'], entry)
                self.assertLessEqual(mm, .02)
                self.assertLessEqual(deg, .01)
                references[side] = entry['target']

    def test_unreachable_returns_finite_bounded_candidate(self):
        refs = {side: [0.]*7 for side in ('left', 'right')}
        goal = self.solver.forward('left', refs['left'])
        goal[0, 3] += 5000.
        entry = self.solver.solve({'left': goal}, refs)['left']
        self.assertFalse(entry['success'])
        self.assertTrue(np.isfinite(entry['target']).all())
        self.assertLessEqual(entry['iterations'], self.solver.options.max_iterations)

    def test_invalid_targets_and_refs_are_rejected(self):
        refs = {side: [0.]*7 for side in ('left', 'right')}
        for goal in (np.full((4, 4), np.nan), np.zeros((4, 4)), np.eye(3)):
            with self.assertRaises(ValueError):
                self.solver.solve({'left': goal}, refs)
        with self.assertRaises(ValueError):
            self.solver.forward('left', [100.]*7)

    def test_asymmetric_tool_offsets_match_independent_model(self):
        geometry = dict(version=1, geometry_id=GEOMETRY_ID,
            tip_cad_mm=[0., 20., -180.], arms={
                'left': dict(installed=True, confirmed=True, roll_deg=37.),
                'right': dict(installed=True, confirmed=True, roll_deg=-81.)})
        tools = tool_config('taccap', geometry)
        solver = Art7TronIK(tools)
        for side in ('left', 'right'):
            q = [.1, .2, -.3, .4, -.2, .15, -.1]
            expected = multiply(tcp_forward_matrix(q), for_arm(tools, side))
            mm, deg = pose_error(expected, solver.forward(side, q))
            self.assertLess(mm, 1e-8)
            self.assertLess(deg, 5e-6)

    def test_direct_native_calls_match_upstream_facade(self):
        solver = Art7TronIK(tool_config())
        refs = dict(left=[.2, .7, -.5, .8, .4, -.1, .1],
                    right=[-.2, .7, .5, .8, -.4, .1, -.1])
        goals = {side: solver.forward(side, np.asarray(q)+.003) for side, q in refs.items()}
        metres = {side: goal.copy() for side, goal in goals.items()}
        for goal in metres.values():
            goal[:3, 3] /= 1000.
        for posture in (None, np.asarray(refs['left']+refs['right'])):
            solver.options = replace(solver.options, posture_weight=0. if posture is None else .001,
                                     posture_target=posture)
            original = solver.model.solve_ik(metres['left'], metres['right'],
                                             refs['left']+refs['right'], solver.options)
            optimized = solver.solve(goals, refs)
            actual = optimized['left']['target']+optimized['right']['target']
            np.testing.assert_allclose(actual, original.q, rtol=0., atol=1e-14)
            self.assertEqual(optimized['left']['status'], original.status)
            self.assertEqual(optimized['left']['iterations'], original.iterations)

    def test_modified_invalid_options_still_fail_python_validation(self):
        solver = Art7TronIK(tool_config())
        refs = {side: [0.]*7 for side in ('left', 'right')}
        goal = solver.forward('left', refs['left'])
        original = solver.options
        # These include Python-only checks that the direct native path must
        # retain even though the C++ core also validates its own option fields.
        for changes in (dict(max_iterations=True), dict(qp_solver='other'),
                        dict(qp_options={'tolerance': 1e-6}), dict(safety_break='yes'),
                        dict(posture_target=[0.]*13)):
            solver.options = replace(original, **changes)
            with self.assertRaises(ValueError):
                solver.solve({'left': goal}, refs)
            with self.assertRaises(ValueError):
                solver.forward('left', refs['left'])


if __name__ == '__main__':
    unittest.main()
