"""Cartesian geometry, causal sampling and real vendor IK without robot I/O."""
import math
import unittest
import numpy as np
import paths
from lerobot.utils.rotation import Rotation
from vr_cartesian import CartesianBuffer, CartesianInput, slerp
from vr_core import direct_positions, validate
from vr_input import Clutch
from vr_plan import request
from vendor_kinematics import VendorKinematics
from tcp_core import ARMS, tcp_forward_matrix, pose_error
from test_tcp import hardware


def pose(x=0., angle=0.):
    t = np.eye(4)
    t[:3, 3] = [x, 20., -30.]
    t[:3, :3] = Rotation.from_rotvec(np.array([0., 0., math.radians(angle)])).as_matrix()
    return t


class CartesianTests(unittest.TestCase):
    def test_pico_delay_uses_clock_offset_not_unsynchronized_wall_clocks(self):
        from pico_latency import calibrated_ages
        offset = 1791600000000000000
        before = {"at_monotonic_ns": 10_000_000_000,
                  "headset_unix_minus_pc_monotonic_ns": offset}
        after = {"at_monotonic_ns": 12_000_000_000,
                 "headset_unix_minus_pc_monotonic_ns": offset+1_000_000}
        sample = {"received_monotonic_ns": 11_000_000_000,
                  "headset_unix_ns": 10_990_000_000+offset+500_000}
        self.assertEqual(calibrated_ages([sample], before, after), [10.])

    def test_linear_translation_and_constant_speed_shortest_arc(self):
        b = CartesianBuffer(0.)
        b.push("left", 10., pose(0., 170.))
        b.push("left", 11., pose(100., -170.))
        previous = pose(0., 170.)
        for i in range(11):
            actual = b.sample(10.+i/10.)["left"]
            np.testing.assert_allclose(actual[:3, 3], [i*10., 20., -30.], atol=1e-10)
            np.testing.assert_allclose(actual[:3, :3], pose(0., 170.+2*i)[:3, :3], atol=1e-10)
            if i:
                self.assertAlmostEqual(pose_error(previous, actual)[1], 2., places=5)
            previous = actual

    def test_quaternion_sign_flip_and_half_turn(self):
        a = np.array([0., 0., 0., 1.])
        b = np.array([1., 0., 0., 0.])
        for u in (0., .2, .5, 1.):
            q = slerp(a, b, u)
            self.assertAlmostEqual(np.linalg.norm(q), 1.)
            np.testing.assert_allclose(Rotation.from_quat(q).as_matrix(),
                Rotation.from_rotvec(np.array([math.pi*u, 0., 0.])).as_matrix(), atol=1e-10)
            np.testing.assert_allclose(slerp(a, -a, u), a, atol=1e-10)

    def test_delay_is_explicit_and_missing_future_samples_are_held(self):
        b = CartesianBuffer(12.)
        b.push("left", 10., pose(0.))
        b.push("left", 10.020, pose(20.))
        self.assertAlmostEqual(b.sample(10.020)["left"][0, 3], 8.)
        self.assertAlmostEqual(b.last_metadata["left"]["position_age_ms"], 12.)
        self.assertEqual(b.sample(10.200)["left"][0, 3], 20.)
        self.assertEqual(b.last_metadata["left"]["position_mode"], "hold_latest")

    def test_rotating_in_place_never_moves_tcp_and_translation_never_rotates(self):
        for rotating in (True, False):
            b = CartesianBuffer(0.)
            b.push("left", 0., pose(15., 20.))
            b.push("left", 1., pose(15. if rotating else 45., 80. if rotating else 20.))
            for t in np.linspace(0., 1., 101):
                actual = b.sample(float(t))["left"]
                if rotating:
                    np.testing.assert_allclose(actual[:3, 3], [15., 20., -30.])
                else:
                    np.testing.assert_allclose(actual[:3, :3], pose(0., 20.)[:3, :3])

    def test_freeze_component_discards_pending_history_without_affecting_other(self):
        b = CartesianBuffer(12.)
        b.push("left", 10., pose(0., 0.))
        b.push("left", 10.02, pose(20., 20.))
        b.push("left", 10.04, pose(40., 5.), reset_rotation=True)
        goal = b.sample(10.04)["left"]
        self.assertAlmostEqual(goal[0, 3], 28.)
        np.testing.assert_allclose(goal[:3, :3], pose(0., 5.)[:3, :3])
        b.clear("left")
        self.assertEqual(b.sample(100.), {})

    def test_real_ik_at_every_cartesian_sample_preserves_path_and_body(self):
        kin, hw = VendorKinematics(), hardware()
        for rotate in (False, True):
            b = CartesianBuffer(0.)
            start = {}
            for side, joints in ARMS.items():
                t = np.array(tcp_forward_matrix([hw["position"][j-1] for j in joints]))
                start[side] = t.copy()
                end = t.copy()
                if rotate:
                    end[:3, :3] = Rotation.from_rotvec(np.array([0., 0., .02])).as_matrix() @ t[:3, :3]
                else:
                    end[:3, 3] += [2., -1., .5]
                b.push(side, 0., t)
                b.push(side, 1., end)
            refs = {}
            for i, stamp in enumerate(np.linspace(0., 1., 91)):
                goals = b.sample(float(stamp))
                packet = request(kin, hw, goals, i+1, 10., refs)
                targets = validate(hw, packet, 10.)
                for side, q in targets.items():
                    actual = np.array(tcp_forward_matrix(q))
                    mm, deg = pose_error(goals[side], actual)
                    self.assertLess(mm, .002)
                    self.assertLess(deg, .002)
                    if rotate:
                        np.testing.assert_allclose(actual[:3, 3], start[side][:3, 3], atol=.002)
                    if side in refs:
                        self.assertLess(max(abs(a-c) for a, c in zip(q, refs[side])), math.radians(1.))
                refs = targets
                points = direct_positions(hw["position"], targets, hw["position"])
                self.assertEqual(len(points), 1)
                self.assertEqual(points[0][0], 0.)
                self.assertEqual(points[0][1][14:], hw["position"][14:])

    def test_grip_release_bypasses_buffer_and_reengagement_never_replays(self):
        kin, hw = VendorKinematics(), hardware()
        clutches = {s: Clutch(s) for s in ARMS}
        p = CartesianInput(clutches, 100.)
        def frame(t, grip, x):
            hand = {"pose": [x, 0., 0., 0., 0., 0., 1.], "grip": grip,
                    "primary": False, "secondary": False}
            return {"received_monotonic": t, "left": dict(hand), "right": dict(hand)}
        def goals(f, now):
            return p.goals(f, hw, kin, {s: s for s in ARMS}, 1., True, now)
        baseline = goals(frame(10., 1., 0.), 10.)
        goals(frame(10.01, 1., .02), 10.01)
        self.assertEqual(goals(frame(10.02, 0., .02), 10.02), {})
        engaged = goals(frame(10.03, 1., .2), 10.03)
        for side in ARMS:
            np.testing.assert_allclose(engaged[side], baseline[side])


if __name__ == "__main__":
    unittest.main()
