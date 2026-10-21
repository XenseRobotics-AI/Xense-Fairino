"""Geometry/pose regressions without a renderer, SSH or motor commands."""
import struct
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from apps.workbench.mesh_geometry import load_visuals, load_tool_visuals, jaw_transform, look_at, perspective, stl_vertices
from apps.workbench.model import RobotModel
from apps.fairino_tcp_probe.tool_frames import load_geometry, flange_from_tip


class MeshTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.visuals = load_visuals()

    def test_every_visual_matches_a_kinematic_link_and_has_finite_unit_normals(self):
        transforms, _ = RobotModel().transforms({})
        self.assertEqual(len(self.visuals), 31)
        for visual in self.visuals:
            self.assertIn(visual.link, transforms)
            self.assertTrue(np.isfinite(visual.vertices).all())
            np.testing.assert_allclose(np.linalg.norm(visual.vertices[:, 3:], axis=1), 1., atol=1e-5)
            self.assertLess(np.abs(visual.vertices[:, :3]).max(), 3.)  # meters, not millimeters

    def test_real_joint_changes_only_its_chain_and_mesh_follows_same_transform(self):
        model = RobotModel()
        before, _ = model.transforms({'j1': 0.})
        after, _ = model.transforms({'j1': .5})
        for visual in self.visuals:
            vertex = np.r_[visual.vertices[0, :3], 1.]
            if visual.link.startswith('right_') or visual.link == 'base_link':
                np.testing.assert_allclose(before[visual.link] @ visual.local @ vertex,
                                           after[visual.link] @ visual.local @ vertex)
        self.assertFalse(np.allclose(before['left_waist2_Link'], after['left_waist2_Link']))

    def test_bad_or_nonfinite_stl_cannot_enter_gpu(self):
        with self.assertRaises(ValueError):
            stl_vertices(b'short')
        bad = b'\0'*80 + struct.pack('<I', 1) + struct.pack('<12fH', *([float('nan')]*12), 0)
        with self.assertRaises(ValueError):
            stl_vertices(bad)

    def test_camera_target_is_centered_and_in_front(self):
        eye, target = np.array([2., -3., 1.]), np.array([0., 0., .7])
        view = look_at(eye, target)
        np.testing.assert_allclose((view @ np.r_[eye, 1.])[:3], 0, atol=1e-12)
        clip = perspective(.8) @ view @ np.r_[target, 1.]
        np.testing.assert_allclose(clip[:2], 0, atol=1e-12)
        self.assertGreater(clip[3], 0)

    def test_gripper_mesh_mount_and_tcp_marker_follow_wrist(self):
        data = load_geometry()
        visuals = load_tool_visuals(data)
        transforms, _ = RobotModel().transforms({'j1': .25})
        self.assertEqual(len(visuals), 10)
        for visual in visuals:
            self.assertIn(visual.link, transforms)
            self.assertTrue(np.isfinite(visual.vertices).all())
            self.assertLess(np.abs(visual.vertices[:, :3]).max(), .3)
            if visual.tool_part == 'tcp':
                offset = np.array(flange_from_tip(data, visual.side))[:3, 3]
                offset[0] -= 106.
                np.testing.assert_allclose(visual.local[:3, 3], offset/1000., atol=1e-9)

    def test_measured_opening_moves_only_fingers_and_reduces_gap_monotonically(self):
        visuals = load_tool_visuals()
        for side in ('left', 'right'):
            jaws = [v for v in visuals if v.side == side and v.jaw_pivot]
            self.assertEqual(len(jaws), 2)
            gaps = []
            for opening in (0., .5, 1.):
                bounds = {}
                for visual in jaws:
                    transform = jaw_transform(visual, opening)
                    pivot = np.r_[visual.jaw_pivot, 1.]
                    np.testing.assert_allclose(transform @ pivot, pivot, atol=1e-12)
                    points = (transform @ np.c_[visual.vertices[:, :3], np.ones(len(visual.vertices))].T).T
                    bounds[visual.tool_part] = points[:, 0]
                    if opening == 1.:
                        np.testing.assert_allclose(transform, np.eye(4))
                gaps.append(bounds['jaw_positive'].min()-bounds['jaw_negative'].max())
            self.assertGreater(gaps[0], 0.)
            self.assertAlmostEqual(gaps[0], .001, delta=.0003)
            self.assertLess(gaps[0], gaps[1])
            self.assertLess(gaps[1], gaps[2])
        for visual in visuals:
            if not visual.jaw_pivot:
                for opening in (0., .5, 1.):
                    np.testing.assert_allclose(jaw_transform(visual, opening), np.eye(4))


if __name__ == '__main__':
    unittest.main()
