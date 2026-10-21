"""Real vendor IK + local wrist module, without connecting to a robot."""
import copy
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import paths
import numpy as np
from tcp_core import pose_error
from test_tcp import hardware
from vendor_kinematics import VendorKinematics
from vr_plan import request
from vr_core import validate
from vr_home import HomeMotion, load_home
from vr_cartesian import CartesianInput
from vr_input import Clutch
from wrist_guard import WristGuard, WristBlocked


class WristPlanningTests(unittest.TestCase):
    def test_real_vendor_ik_reduces_cartesian_target_and_preserves_independent_fk(self):
        kin = VendorKinematics()
        hw = hardware()
        q = load_home()["target"]["left"][:]
        q[5:7] = [math.radians(52), math.radians(31)]
        hw["position"][:7] = hw["command_position"][:7] = q
        goal_q = q[:]; goal_q[6] = math.radians(39)
        goal = kin.forward_tcp(goal_q, "left")[0]
        packet = request(kin, hw, {"left": goal}, 1, 10.)
        entry = packet["arms"]["left"]
        self.assertGreater(entry["wrist_scale"], 0.)
        self.assertLess(entry["wrist_scale"], 1.)
        validate(hw, packet, 10.)  # Independent stdlib robot-side FK.
        WristGuard().arm_segment(q, entry["target"])
        mm, deg = pose_error(np.asarray(entry["tcp"]).reshape(4, 4),
                             kin.forward_tcp(entry["target"], "left")[0])
        self.assertLess(mm, .02); self.assertLess(deg, .01)
        unprotected = request(kin, hw, {"left": goal}, 2, 10., guard=WristGuard(False))
        self.assertEqual(unprotected["arms"]["left"]["wrist_scale"], 1.)
        self.assertGreater(math.degrees(unprotected["arms"]["left"]["target"][6]), 38.)

    def test_limited_clutch_discards_excess_and_inward_input_works_without_regrip(self):
        c = Clutch("left")
        cartesian = CartesianInput({"left": c})
        hand = {"pose": [0., 0., 0., 0., 0., 0., 1.], "grip": 1.}
        origin = np.eye(4)
        c.target(hand, origin, 1.)
        hand["pose"][0] = .1
        requested = c.target(hand, origin, 1.)
        accepted = origin.copy()
        accepted[:3, 3] = .2*requested[:3, 3]
        cartesian.buffer.push("left", 1., requested)
        cartesian.accepted_plan({"arms": {"left": {"target": [0.]*7,
            "tcp": accepted.flatten().tolist(), "wrist_scale": .2}}},
            {"left": hand}, {"left": "left"})
        self.assertTrue(c.engaged)
        self.assertNotIn("left", cartesian.buffer.paths)
        # Holding the same hand pose cannot replay the remaining 80 mm.
        np.testing.assert_allclose(c.target(hand, accepted, 1.), accepted, atol=1e-9)
        hand["pose"][0] -= .001
        retreat = c.target(hand, accepted, 1.)
        self.assertAlmostEqual(np.linalg.norm(retreat[:3, 3]-accepted[:3, 3]), 1.)
        self.assertLess(float((retreat[:3, 3]-accepted[:3, 3]) @ requested[:3, 3]), 0.)

    def test_home_refuses_interior_collision_before_streaming_any_target(self):
        hw, home = hardware(), copy.deepcopy(load_home())
        for field in ("position", "command_position"):
            hw[field][5:7] = [0., math.radians(70)]
        home["target"]["left"][5:7] = [math.radians(60), math.radians(30)]
        with self.assertRaises(WristBlocked):
            HomeMotion(home, hw, 10.)
        HomeMotion(home, hw, 10., WristGuard(False))

    def test_both_existing_homes_and_velocity_reserve_work(self):
        from home_presets import resolve_home
        hw = hardware()
        for preset in ("default", "right_mirrored"):
            home = load_home(resolve_home(preset))
            motion = HomeMotion(home, hw, 10.)
            for i in range(100):
                target = motion.sample(10.+i*.01)
                for side in target:
                    WristGuard().arm_segment(motion.start[side], target[side])


if __name__ == "__main__":
    unittest.main()
