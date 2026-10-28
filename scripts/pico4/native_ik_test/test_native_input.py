"""Offline controller geometry tests; no SDK, SSH or robot is constructed."""
import math
import unittest

import numpy as np

from scripts.pico4.native_ik_test.native_input import NativeClutch, continuous_rotation_vector
from lerobot.utils.rotation import Rotation
from vr_input import Clutch, mapped_pose, rotation_mapping


def controller(angle=0., axis=(0., 1., 0.), position=(0., 0., 0.), grip=1., rotation=None):
    axis = np.asarray(axis, dtype=float)
    if rotation is None:
        rotation = Rotation.from_rotvec(axis/np.linalg.norm(axis)*math.radians(angle)).as_matrix()
    return dict(pose=[*position, *Rotation.from_matrix(rotation).as_quat()], grip=grip)


def turn(angle, axis=(0., 1., 0.)):
    axis = np.asarray(axis, dtype=float)
    return Rotation.from_rotvec(axis/np.linalg.norm(axis)*math.radians(angle)).as_matrix()


class NativeInputTests(unittest.TestCase):
    def test_crosses_both_half_turns_continuously_in_all_mapping_modes(self):
        for mode in ('first_person', 'mirror', 'mirror_same_direction'):
            for side in ('left', 'right'):
                for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 2, 3)):
                    for sign in (-1, 1):
                        with self.subTest(mode=mode, side=side, axis=axis, sign=sign):
                            clutch = NativeClutch(side, mode, rotation_mapping(mode))
                            clutch.target(controller(), np.eye(4), .25)
                            previous = None
                            for degrees in (170, 179, 180, 181, 190, 220, 269):
                                hand = controller(sign*degrees, axis)
                                target = clutch.target(hand, np.eye(4), .25)
                                # Map a small rotation to get the correct axial
                                # sign for proper AND reflected orientation frames.
                                mapped = mapped_pose(controller(sign, axis)['pose'], side, mode,
                                                     rotation_mapping(mode))[1]
                                expected = Rotation.from_rotvec(
                                    Rotation.from_matrix(mapped).as_rotvec()*degrees*.25).as_matrix()
                                np.testing.assert_allclose(target[:3, :3], expected, atol=1e-10)
                                if degrees == 181:
                                    jump = np.linalg.norm(Rotation.from_matrix(
                                        target[:3, :3] @ previous.T).as_rotvec())
                                    self.assertAlmostEqual(math.degrees(jump), .25, places=8)
                                previous = target[:3, :3]

    def test_reproduces_old_half_turn_jump_and_removes_it(self):
        old, new = Clutch('left'), NativeClutch('left')
        result = []
        for clutch in (old, new):
            clutch.target(controller(), np.eye(4), .25)
            clutch.target(controller(170), np.eye(4), .25)
            a = clutch.target(controller(179), np.eye(4), .25)
            b = clutch.target(controller(181), np.eye(4), .25)
            result.append(math.degrees(np.linalg.norm(Rotation.from_matrix(
                b[:3, :3] @ a[:3, :3].T).as_rotvec())))
        self.assertAlmostEqual(result[0], 89.5)
        self.assertAlmostEqual(result[1], .5)

    def test_stationary_and_repeated_frames_have_identical_target(self):
        clutch = NativeClutch('left')
        clutch.target(controller(), np.eye(4), .25)
        for degrees in range(0, 221, 10):
            expected = clutch.target(controller(degrees), np.eye(4), .25)
        for _ in range(1000):
            np.testing.assert_array_equal(clutch.target(controller(220), np.eye(4), .25), expected)

    def test_multi_axis_closed_path_returns_to_anchor_without_integrating_drift(self):
        clutch = NativeClutch('right', 'mirror_same_direction', 'mirror')
        anchor = np.eye(4)
        anchor[:3, :3] = turn(37, (1, 2, -1))
        clutch.target(controller(), anchor, .25)
        # A noncommuting loop with exact final identity, repeated many times.
        # Its anchored rotation stays below pi; per-frame gain integration
        # would generally leave a different target on return.
        for _ in range(20):
            for t in np.linspace(0, 2*math.pi, 121):
                rotation = turn(60*math.sin(t), (1, 0, 0)) @ turn(40*(1-math.cos(t)), (0, 0, 1))
                target = clutch.target(controller(rotation=rotation), anchor, .25)
            np.testing.assert_allclose(target, anchor, atol=1e-12)

    def test_mixed_axes_half_turn_path_retraces_exactly(self):
        clutch = NativeClutch('left')
        clutch.target(controller(), np.eye(4), .25)
        path = [controller(angle, (1., .2*math.sin(math.radians(angle)), .1))
                for angle in range(0, 241, 2)]
        forward = [clutch.target(hand, np.eye(4), .25) for hand in path]
        for hand, expected in zip(reversed(path), reversed(forward)):
            np.testing.assert_allclose(clutch.target(hand, np.eye(4), .25), expected, atol=1e-10)
        np.testing.assert_allclose(clutch.target(controller(), np.eye(4), .25), np.eye(4), atol=1e-10)

    def test_rotation_does_not_translate_and_translation_matches_shared_mapping(self):
        for mode in ('first_person', 'mirror', 'mirror_same_direction'):
            old = Clutch('left', mode, rotation_mapping(mode))
            new = NativeClutch('left', mode, rotation_mapping(mode))
            base = np.eye(4); base[:3, 3] = [200., -30., 50.]
            for clutch in (old, new):
                clutch.target(controller(), base, .25)
            for hand in (controller(0, position=(.1, -.2, .3)), controller(100), controller(170)):
                np.testing.assert_allclose(new.target(hand, base, .25), old.target(hand, base, .25), atol=1e-12)
            np.testing.assert_allclose(new.target(controller(190), base, .25)[:3, 3], base[:3, 3])

    def test_rotation_limit_requires_regrip_and_never_clamps(self):
        for sign in (-1, 1):
            clutch = NativeClutch('left')
            clutch.target(controller(), np.eye(4), .25)
            for angle in range(0, 271, 10):
                clutch.target(controller(sign*angle), np.eye(4), .25)
            previous = clutch.continuous_rotation.copy()
            with self.assertRaisesRegex(ValueError, '270°'):
                clutch.target(controller(sign*271), np.eye(4), .25)
            np.testing.assert_array_equal(clutch.continuous_rotation, previous)
            self.assertIsNone(clutch.target(controller(sign*271, grip=0.), np.eye(4), .25))
            base = np.eye(4); base[:3, :3] = turn(33, (1, 0, 0))
            np.testing.assert_allclose(clutch.target(controller(sign*271), base, .25), base, atol=1e-12)

    def test_identity_after_long_branch_is_rejected_instead_of_axis_guess(self):
        previous = np.array([0., math.radians(260), 0.])
        with self.assertRaisesRegex(ValueError, '270°'):
            continuous_rotation_vector(np.eye(3), previous)

    def test_freeze_release_rebase_and_heading_discard_old_rotation_branch(self):
        clutch = NativeClutch('left')
        base = np.eye(4)
        clutch.target(controller(), base, .25)
        for angle in (100, 170, 190, 220):
            clutch.target(controller(angle), base, .25)
        held = base.copy(); held[:3, :3] = turn(20, (1, 0, 0))
        for angle, frozen in ((220, True), (250, True), (350, True), (20, False)):
            np.testing.assert_allclose(clutch.target(controller(angle), held, .25,
                                                    freeze_rotation=frozen), held, atol=1e-12)
        clutch.rebase(controller(40), held)
        np.testing.assert_allclose(clutch.target(controller(40), held, .25), held, atol=1e-12)
        clutch.set_heading(turn(50))
        self.assertFalse(clutch.engaged)
        np.testing.assert_allclose(clutch.target(controller(170), held, .25), held, atol=1e-12)


if __name__ == '__main__':
    unittest.main()
