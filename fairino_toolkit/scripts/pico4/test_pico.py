"""Offline checks: no service, headset, or robot connection is made."""
import math
import unittest

import numpy as np

from pico import Freshness, build_preview, valid_pose


class FakeSdk:
    def __init__(self):
        self.left_pose = [0., 0., 0., 0., 0., 0., 1.]
        self.right_pose = self.left_pose.copy()
        self.left_grip = self.right_grip = 0.

    def get_left_controller_pose(self): return self.left_pose
    def get_right_controller_pose(self): return self.right_pose
    def get_left_grip(self): return self.left_grip
    def get_right_grip(self): return self.right_grip
    def get_left_trigger(self): return 0.
    def get_right_trigger(self): return 0.
    def get_A_button(self): return False
    def get_B_button(self): return False
    def get_X_button(self): return False
    def get_Y_button(self): return False
    def get_left_axis_click(self): return False
    def get_right_axis_click(self): return False
    def close(self): pass


class InputTests(unittest.TestCase):
    def test_stream_timeout_and_clock_reset(self):
        clock = Freshness()
        self.assertFalse(clock.update(0, 10.))
        self.assertTrue(clock.update(1000000000000, 10.))
        self.assertFalse(clock.update(1000000000000, 10.6))
        self.assertTrue(clock.update(2, 10.7))  # Headset reboot changes clock origin.

    def test_missing_or_corrupt_pose(self):
        self.assertFalse(valid_pose([0.] * 7))
        self.assertFalse(valid_pose([math.nan, 0., 0., 0., 0., 0., 1.]))
        self.assertTrue(valid_pose([0., 0., 0., 0., 0., 0., 1.]))

    def test_existing_mapping_clutch_and_regrip(self):
        sdk = FakeSdk()
        teleop = build_preview(sdk, sensitivity=0.25)
        # Disable only preview rate caps for deterministic mapping measurements.
        for arm in (teleop._left_pico4, teleop._right_pico4):
            arm.config.max_pos_velocity = arm.config.max_rot_velocity = 0.
        def pos():
            action = teleop.get_action()
            return np.array([action[f"left_tcp.{axis}"] for axis in "xyz"])
        initial = pos()
        sdk.left_grip = 1.
        np.testing.assert_allclose(pos(), initial, atol=1e-7)
        sdk.left_pose[2] -= 0.04
        moved = pos()
        self.assertAlmostEqual(float(np.linalg.norm(moved - initial)), 0.01, places=6)
        sdk.left_grip = 0.
        pos()
        sdk.left_pose[2] -= 0.03
        np.testing.assert_allclose(pos(), moved, atol=1e-7)
        sdk.left_grip = 1.
        np.testing.assert_allclose(pos(), moved, atol=1e-7)
        sdk.left_pose[2] -= 0.02
        self.assertAlmostEqual(float(np.linalg.norm(pos() - moved)), 0.005, places=6)

    def test_preview_does_not_change_other_arm(self):
        sdk = FakeSdk()
        teleop = build_preview(sdk)
        sdk.right_grip = 1.
        teleop.get_action()
        sdk.right_pose[0] += 0.02
        action = teleop.get_action()
        np.testing.assert_allclose([action[f"left_tcp.{a}"] for a in "xyz"], 0., atol=1e-7)


if __name__ == "__main__":
    unittest.main()
