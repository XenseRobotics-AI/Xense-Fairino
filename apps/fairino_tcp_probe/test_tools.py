"""CAD frame agreement and independent tool FK/IK, without robot access."""
import unittest
import numpy as np

from tool_frames import (load_geometry, tool_config, validate_runtime, visual_mounts,
                         flange_from_tip, multiply, translate)
from tcp_core import pose_error, target_matrix, tcp_forward_matrix
from vendor_kinematics import VendorKinematics


class ToolTests(unittest.TestCase):
    def setUp(self):
        self.geometry = load_geometry()
        self.geometry['arms']['left'].update(roll_deg=90., confirmed=True)
        self.geometry['arms']['right'].update(roll_deg=-90., confirmed=True)
        self.tools = tool_config('taccap', self.geometry, require_confirmed=True)

    def test_cad_boss_and_seating_plane_match_the_adapter_recess(self):
        adapter, gripper = map(np.array, visual_mounts(self.geometry, 'left'))
        np.testing.assert_allclose(gripper @ [0, -15.75, 38.5, 1],
                                   adapter @ [0, 13, 0, 1], atol=1e-9)
        np.testing.assert_allclose(gripper @ [0, -15.75, 46.5, 1],
                                   adapter @ [0, 5, 0, 1], atol=1e-9)

    def test_tip_is_the_same_point_in_cad_and_the_control_transform(self):
        for side in ('left', 'right'):
            _, gripper = visual_mounts(self.geometry, side)
            cad = np.array(gripper) @ [*self.geometry['tip_cad_mm'], 1.]
            control = np.array(multiply(translate((-106, 0, 0)), flange_from_tip(self.geometry, side)))[:, 3]
            np.testing.assert_allclose(cad, control, atol=1e-9)
        left = flange_from_tip(self.geometry, 'left')
        np.testing.assert_allclose(np.array(left)[:3, 3], [-204.212, 36.575, 0.], atol=1e-9)

    def test_unconfirmed_or_corrupt_configuration_cannot_drive(self):
        self.geometry['arms']['right']['confirmed'] = False
        with self.assertRaisesRegex(ValueError, '尚未核对'):
            tool_config('taccap', self.geometry, require_confirmed=True)
        corrupt = tool_config('taccap', self.geometry)
        corrupt['transforms']['left'][0][3] += 1.
        with self.assertRaisesRegex(ValueError, '指纹'):
            validate_runtime(corrupt)
        for invalid in (float('nan'), 181., True):
            self.geometry['arms']['left']['roll_deg'] = invalid
            with self.assertRaises(ValueError):
                tool_config('taccap', self.geometry)

    def test_rotating_tip_keeps_its_position_while_flange_moves(self):
        kin = VendorKinematics(tools=self.tools)
        q = np.deg2rad([0, -35, 0, 70, 0, -25, 0]).tolist()
        for side in ('left', 'right'):
            start, angle = kin.forward_tcp(q, side)
            goal = target_matrix(start, 4, 1, .2)
            result = kin.inverse_tcp(goal, angle, q, side)
            mm, degrees = pose_error(goal, kin.forward_tcp(result, side)[0])
            self.assertLess(mm, .02)
            self.assertLess(degrees, .01)
            self.assertGreater(pose_error(tcp_forward_matrix(q), tcp_forward_matrix(result))[0], .2)
        with self.assertRaisesRegex(ValueError, '必须指定手臂'):
            kin.forward_tcp(q)


if __name__ == '__main__':
    unittest.main()
