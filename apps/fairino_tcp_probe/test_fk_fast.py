"""Compare the direct DH update with the original full 4×4 matrix chain."""
import itertools
import math
import random
import unittest

from tcp_core import (ALPHA, BOUNDS_DEG, D_MM, FLANGE_OFFSET_MM, THETA,
                      check_arm_limits, forward_matrix, multiply, pose_matrix,
                      tcp_forward_matrix)
from tool_frames import GEOMETRY_ID, for_arm, tool_config


def old_matrix_product(a, b):
    # Frozen pre-optimization implementation, intentionally not using the
    # production multiply/FK helpers as the numerical reference.
    return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def old_rotation(axis, angle):
    c, s = math.cos(angle), math.sin(angle)
    result = [[float(i == j) for j in range(4)] for i in range(4)]
    i, j = ((1, 2), (2, 0), (0, 1))[axis]
    result[i][i] = result[j][j] = c
    result[i][j], result[j][i] = -s, s
    return result


def old_forward(q):
    result = [[float(i == j) for j in range(4)] for i in range(4)]
    for v, d, alpha, offset in zip(q, D_MM, ALPHA, THETA):
        joint = old_matrix_product(old_rotation(2, v+offset), old_rotation(0, alpha))
        joint[2][3] = d
        result = old_matrix_product(result, joint)
    return result


def old_tcp(q):
    wrist = old_forward(q)
    offset = [[float(i == j) for j in range(4)] for i in range(4)]
    for i, value in enumerate(FLANGE_OFFSET_MM):
        offset[i][3] = value
    return old_matrix_product(wrist, offset)


class FastFKEquivalenceTests(unittest.TestCase):
    def assert_matrix_close(self, actual, expected):
        self.assertEqual(len(actual), 4)
        for i in range(4):
            self.assertEqual(len(actual[i]), 4)
            for j in range(4):
                # Matrix elements avoid acos(trace) roundoff amplification at
                # identical rotations. Units: translations mm, rotations unitless.
                self.assertAlmostEqual(actual[i][j], expected[i][j], delta=1e-10 if j == 3 else 2e-14)
        self.assertEqual(actual[3], [0., 0., 0., 1.])

    def test_thousand_random_configurations_match_old_wrist_and_flange_chain(self):
        rng = random.Random(20261109)
        for _ in range(1000):
            q = [math.radians(rng.uniform(lo, hi)) for lo, hi in BOUNDS_DEG]
            saved = q[:]
            self.assert_matrix_close(forward_matrix(q), old_forward(q))
            self.assert_matrix_close(tcp_forward_matrix(q), old_tcp(q))
            self.assertEqual(q, saved)

    def test_all_joint_limit_corners_and_zero_singular_poses_match(self):
        # All 2**7 corners stress accumulated sign/axis/offset errors.
        poses = [[math.radians(value) for value in degrees]
                 for degrees in itertools.product(*BOUNDS_DEG)]
        poses.extend(([0.]*7, [math.pi/2]*7, [-math.pi/2]*7,
                      [0., 0., 0., 0., 0., -math.pi/2, 0.]))
        for q in poses:
            with self.subTest(q=q):
                self.assert_matrix_close(forward_matrix(q), old_forward(q))
                self.assert_matrix_close(tcp_forward_matrix(q), old_tcp(q))

    def test_both_asymmetric_tool_tcp_offsets_match_old_chain(self):
        geometry = dict(version=1, geometry_id=GEOMETRY_ID, tip_cad_mm=[7., 32., -163.],
                        arms=dict(left=dict(installed=True, confirmed=True, roll_deg=27.),
                                  right=dict(installed=True, confirmed=True, roll_deg=-53.)))
        tools = tool_config('taccap', geometry)
        self.assertNotEqual(for_arm(tools, 'left'), for_arm(tools, 'right'))
        rng = random.Random(4109)
        for _ in range(150):
            q = [math.radians(rng.uniform(lo, hi)) for lo, hi in BOUNDS_DEG]
            for side in ('left', 'right'):
                offset = for_arm(tools, side)
                actual = multiply(tcp_forward_matrix(q), offset)
                expected = old_matrix_product(old_tcp(q), offset)
                self.assert_matrix_close(actual, expected)

    def test_nontrivial_rigid_post_transform_composition_matches(self):
        # Runtime TCP configuration supports translations only. This separate
        # math test also checks all rotation columns under a rigid post-transform.
        post = pose_matrix([70., -41., 98., 37., -28., 63.])
        q = list(map(math.radians, [31., -88., 121., 107., -123., 58., -79.]))
        self.assert_matrix_close(multiply(tcp_forward_matrix(q), post), old_matrix_product(old_tcp(q), post))

    def test_invalid_vector_contract_remains_unchanged(self):
        values = (None, [], [0.]*6, [0.]*8, '0000000', [False]*7,
                  [float('nan')]+[0.]*6, [float('inf')]+[0.]*6, ['0']+[0.]*6)
        for q in values:
            with self.subTest(q=q):
                with self.assertRaises(ValueError):
                    forward_matrix(q)
                with self.assertRaises(ValueError):
                    tcp_forward_matrix(q)
        self.assertEqual(forward_matrix((0.,)*7), forward_matrix([0.]*7))

    def test_joint_limits_remain_a_separate_unchanged_validation(self):
        # Raw FK intentionally remains defined outside the robot's allowed
        # interval; actuator request validation must still reject that pose.
        for axis, (lower, upper) in enumerate(BOUNDS_DEG):
            for outside in (lower-.01, upper+.01):
                q = [0.]*7
                q[axis] = math.radians(outside)
                self.assert_matrix_close(forward_matrix(q), old_forward(q))
                with self.assertRaises(ValueError):
                    check_arm_limits(q)


if __name__ == '__main__':
    unittest.main()
