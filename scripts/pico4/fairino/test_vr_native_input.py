"""Main Cartesian pipeline with native input; geometry only, no device I/O."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'src'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import paths
from lerobot.utils.rotation import Rotation
from tcp_core import ARMS
from vr_cartesian import CartesianInput
from vr_input import controller_sides, rotation_mapping
from vr_native_input import NativeClutch


def turn(angle):
    return Rotation.from_rotvec(np.array([0., math.radians(angle), 0.])).as_matrix()


def hand(angle=0., position=(0., 0., 0.), grip=1., primary=False, secondary=False):
    return dict(pose=[*position, *Rotation.from_matrix(turn(angle)).as_quat()],
                grip=grip, primary=primary, secondary=secondary)


def frame(stamp, left=None, right=None):
    return dict(received_monotonic=stamp,
                left=hand(grip=0.) if left is None else left,
                right=hand(grip=0.) if right is None else right)


def angular_distance(a, b):
    return math.degrees(np.linalg.norm(Rotation.from_matrix(a[:3, :3] @ b[:3, :3].T).as_rotvec()))


class FixedFK:
    """Only the current commanded TCP is needed by the input pipeline."""
    def __init__(self):
        self.tcp = {s: np.eye(4) for s in ARMS}
        self.tcp['left'][:3, 3] = [180., -40., 250.]
        self.tcp['right'][:3, 3] = [190., 50., 260.]

    def forward_tcp(self, _q, side):
        return self.tcp[side].copy(), 0.


class NativeCartesianTests(unittest.TestCase):
    def setUp(self):
        self.hw = dict(command_position=[0.]*20, velocity=[0.]*20)
        self.kin = FixedFK()

    def pipeline(self, mode='first_person', delay=12.):
        return CartesianInput({s: NativeClutch(s, mode, rotation_mapping(mode))
                               for s in ARMS}, delay)

    def goals(self, pipeline, sample, mode='first_person', gain=.25, now=None):
        return pipeline.goals(sample, self.hw, self.kin, controller_sides(mode),
                              gain, True, sample['received_monotonic'] if now is None else now)

    def test_standalone_and_main_import_the_same_tested_implementation(self):
        from scripts.pico4.native_ik_test.native_input import NativeClutch as StandaloneClutch
        self.assertIs(StandaloneClutch, NativeClutch)

    def test_buffered_half_turn_crossing_stays_continuous_for_both_hands_and_gains(self):
        for mode in ('first_person', 'mirror_same_direction'):
            for side, physical in controller_sides(mode).items():
                for gain in (.25, .5):
                    with self.subTest(mode=mode, side=side, gain=gain):
                        p = self.pipeline(mode)
                        self.goals(p, frame(10., **{physical: hand()}), mode, gain)
                        # Batch ingestion is the path used when SDK frames arrive
                        # faster than an IK cycle. Every branch crossing matters.
                        samples = [frame(10.+i*.020, **{physical: hand(angle)})
                                   for i, angle in enumerate((170., 179., 181.), 1)]
                        packet = {**samples[-1], 'samples': samples}
                        goals = self.goals(p, packet, mode, gain, now=10.060)
                        self.assertEqual(set(goals), {side})
                        previous = goals[side]
                        later = self.goals(p, samples[-1], mode, gain, now=10.080)[side]
                        # At 12 ms delay: 179.8 -> 181 controller degrees.
                        self.assertAlmostEqual(angular_distance(later, previous), 1.2*gain, places=6)
                        np.testing.assert_allclose(later[:3, 3], self.kin.tcp[side][:3, 3])
                        self.assertAlmostEqual(np.linalg.norm(p.clutches[side].continuous_rotation),
                                               math.radians(181.), places=8)

    def test_rotation_freeze_flushes_pending_rotation_and_release_does_not_replay(self):
        p = self.pipeline(delay=100.)
        self.goals(p, frame(10., left=hand()))
        for i, angle in enumerate((170., 190., 220.), 1):
            self.goals(p, frame(10.+.02*i, left=hand(angle)))
        for stamp, angle, frozen in ((10.08, 230., True), (10.10, 350., True), (10.12, 20., False)):
            goal = self.goals(p, frame(stamp, left=hand(angle, primary=frozen)))['left']
            np.testing.assert_allclose(goal[:3, :3], self.kin.tcp['left'][:3, :3], atol=1e-12)
            np.testing.assert_allclose(p.clutches['left'].continuous_rotation, np.zeros(3), atol=1e-12)
        resumed = self.goals(p, frame(10.30, left=hand(30.)), now=10.40)['left']
        self.assertAlmostEqual(angular_distance(resumed, self.kin.tcp['left']), 2.5, places=6)

    def test_position_freeze_does_not_reset_continuous_rotation(self):
        p = self.pipeline(delay=0.)
        self.goals(p, frame(10., left=hand()))
        before = None
        for i, angle in enumerate((170., 190., 220.), 1):
            before = self.goals(p, frame(10.+.02*i, left=hand(angle, position=(.05, 0., 0.))))['left']
        frozen = self.goals(p, frame(10.08, left=hand(230., position=(.1, .2, .3), secondary=True)))['left']
        np.testing.assert_allclose(frozen[:3, 3], self.kin.tcp['left'][:3, 3])
        self.assertAlmostEqual(angular_distance(frozen, before), 2.5, places=6)
        released = self.goals(p, frame(10.10, left=hand(240., position=(.5, .6, .7))))['left']
        np.testing.assert_allclose(released[:3, 3], self.kin.tcp['left'][:3, 3])
        self.assertAlmostEqual(angular_distance(released, frozen), 2.5, places=6)

    def test_release_regrip_and_heading_reset_drop_old_samples_and_branch(self):
        p = self.pipeline(delay=100.)
        self.goals(p, frame(10., left=hand()))
        for i, angle in enumerate((170., 190., 250.), 1):
            self.goals(p, frame(10.+.02*i, left=hand(angle)))
        p.references['left'] = [1.]*7
        self.assertEqual(self.goals(p, frame(10.08, left=hand(260., grip=0.))), {})
        self.assertEqual(p.references, {})
        self.assertEqual(p.buffer.paths, {})
        regrip = self.goals(p, frame(10.10, left=hand(260.)))['left']
        np.testing.assert_allclose(regrip, self.kin.tcp['left'], atol=1e-12)
        self.goals(p, frame(10.12, left=hand(270.)))
        p.reset()
        for clutch in p.clutches.values():
            clutch.set_heading(turn(50.))
        self.assertEqual(p.buffer.paths, {})
        calibrated = self.goals(p, frame(10.14, left=hand(190.)))['left']
        np.testing.assert_allclose(calibrated, self.kin.tcp['left'], atol=1e-12)

    def test_partial_native_plan_keeps_full_hand_target_and_branch(self):
        p = self.pipeline(delay=0.)
        self.goals(p, frame(10., left=hand()))
        for i, angle in enumerate((170., 190., 220.), 1):
            sample = frame(10.+.02*i, left=hand(angle, position=(.05, 0., 0.)))
            goal = self.goals(p, sample)['left']
        branch = p.clutches['left'].continuous_rotation.copy()
        p.accepted_plan({'arms': {'left': {'target': [0.]*7,
                        'tcp': self.kin.tcp['left'].flatten().tolist(), 'success': False}}},
                        sample, controller_sides())
        repeated = self.goals(p, sample)['left']
        np.testing.assert_allclose(repeated, goal, atol=1e-12)
        np.testing.assert_array_equal(p.clutches['left'].continuous_rotation, branch)
        self.assertIn('left', p.buffer.paths)


if __name__ == '__main__':
    unittest.main()
