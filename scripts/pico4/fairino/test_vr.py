"""Offline bimanual integration, real vendor IK, and failure-path regression."""
import copy
import json
import math
import sys
import unittest
import io
import queue
import threading
import tempfile
import contextlib
import ctypes as C
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parent))
import paths
import numpy as np
import robot_probe as rp
from test_tcp import hardware, probe
from test_probe import STUBS
from vendor_kinematics import VendorKinematics
from tcp_core import ARMS, tcp_forward_matrix, target_matrix, pose_error
from vr_input import Clutch, decode_packet, Stream
from lerobot.utils.rotation import Rotation
from vr_plan import request
from vr_core import validate, direct_positions as position_trajectory
from vr_robot import VRProbe


def controller(x=0., grip=0., quat=None):
    return {"pose": [x, 0., 0., *(quat or [0., 0., 0., 1.])], "grip": grip}


class InputTests(unittest.TestCase):
    def test_duplicate_stream_timestamp_does_not_refresh_cached_controller_pose(self):
        stream = Stream()  # Loads library only; PXREAInit is never called here.
        value = {"timeStampNs": 123, "Controller": {s: controller() for s in ARMS}}
        state = stream.State()
        state.devID = b"test-headset"
        state.stateJson = json.dumps({"value": json.dumps(value)}).encode()
        with patch("vr_input.time.monotonic", return_value=10.):
            stream._callback(None, 1 << 25, 0, C.pointer(state))
            self.assertEqual(stream.read()["stamp"], 123)
        with patch("vr_input.time.monotonic", return_value=10.3):
            stream._callback(None, 1 << 25, 0, C.pointer(state))
            with self.assertRaisesRegex(ValueError, "未更新"): stream.read()

    def test_packet_missing_one_hand_or_invalid_tracking_cannot_reuse_previous(self):
        value = {"timeStampNs": 10, "Controller": {s: controller() for s in ARMS}}
        def decode():
            return decode_packet(json.dumps({"value": json.dumps(value)}))
        self.assertIn("left", decode())
        del value["Controller"]["right"]
        with self.assertRaises(ValueError): decode()
        value["Controller"]["right"] = {**controller(), "isTracked": False}
        with self.assertRaises(ValueError): decode()
        value["Controller"]["right"] = controller(quat=[0., 0., 0., 0.])
        with self.assertRaises(ValueError): decode()

    def test_relative_clutch_uses_real_tcp_release_discards_backlog_and_regrip_rebases(self):
        clutch = Clutch("right")
        tcp = np.array(tcp_forward_matrix(hardware()["position"][:7]))
        self.assertIsNone(clutch.target(controller(), tcp, .25))
        np.testing.assert_allclose(clutch.target(controller(grip=1.), tcp, .25), tcp)
        target = clutch.target(controller(x=.04, grip=1.), tcp, .25)
        np.testing.assert_allclose(target[:3, 3]-tcp[:3, 3], [0., 0., 10.], atol=.001)
        self.assertIsNone(clutch.target(controller(x=.08), tcp, .25))
        # The robot only moved 2 mm before release. Do not reuse the 10 mm goal.
        actual = tcp.copy(); actual[2, 3] += 2.
        np.testing.assert_allclose(clutch.target(controller(x=.12, grip=1.), actual, .25), actual)

    def test_rotation_keeps_flange_center_and_quaternion_sign_has_no_effect(self):
        from lerobot.utils.rotation import Rotation
        tcp = np.array(tcp_forward_matrix(hardware()["position"][:7]))
        results = []
        for sign in (-1., 1.):
            clutch = Clutch("left")
            clutch.target(controller(grip=1.), tcp, .5)
            q = (sign*Rotation.from_rotvec(np.array([.2, 0., 0.])).as_quat()).tolist()
            target = clutch.target(controller(grip=1., quat=q), tcp, .5)
            np.testing.assert_allclose(target[:3, 3], tcp[:3, 3])
            self.assertAlmostEqual(pose_error(tcp, target)[1], math.degrees(.1), places=5)
            results.append(target)
        np.testing.assert_allclose(results[0], results[1])

    def test_one_hand_clutch_does_not_engage_the_other(self):
        clutches = {s: Clutch(s) for s in ARMS}
        tcp = np.eye(4)
        self.assertIsNotNone(clutches["left"].target(controller(grip=1.), tcp, .25))
        self.assertIsNone(clutches["right"].target(controller(), tcp, .25))
        self.assertIsNone(Clutch("left").target(controller(grip=1.), tcp, .25, stationary=False))

    def test_controller_displacement_above_100mm_is_not_clamped_or_rejected(self):
        clutch = Clutch("right")
        clutch.target(controller(grip=1.), np.eye(4), .25)
        target = clutch.target(controller(x=.2, grip=1.), np.eye(4), .25)
        np.testing.assert_allclose(target[:3, 3], [0., 0., 50.], atol=.001)
        self.assertAlmostEqual(np.linalg.norm(target[:3, 3]), 50., places=9)


class LeftMountTests(unittest.TestCase):
    # Independent physical basis from the user's observations, expressed as
    # (forward, left, up). Columns are the original left base's X/Y/Z axes.
    observed_base_axes = np.column_stack((
        np.array([1., 0., -1.])/math.sqrt(2.),
        np.array([-1., 0., -1.])/math.sqrt(2.),
        [0., 1., 0.]))
    # Forward / left / up expressed in native PICO tracking coordinates.
    tracking_directions = ([0., 0., -1.], [-1., 0., 0.], [0., 1., 0.])

    def start(self):
        tcp = np.array(tcp_forward_matrix(hardware()["position"][:7]))
        rotation = Rotation.from_rotvec(np.array([.21, -.36, .12])).as_matrix()
        hand = {"pose": [-.22, .83, -.4, *Rotation.from_matrix(rotation).as_quat()], "grip": 1.}
        return tcp, hand, rotation

    def test_six_physical_translation_directions_and_sensitivity(self):
        for sensitivity in (.25, .5, 1.):
            for axis, raw_direction in enumerate(self.tracking_directions):
                for sign in (-1., 1.):
                    with self.subTest(sensitivity=sensitivity, axis=axis, sign=sign):
                        tcp, hand, _ = self.start()
                        clutch = Clutch("left")
                        np.testing.assert_allclose(clutch.target(hand, tcp, sensitivity), tcp, atol=1e-10)
                        hand["pose"][:3] = np.array(hand["pose"][:3]) + sign*.04*np.array(raw_direction)
                        target = clutch.target(hand, tcp, sensitivity)
                        physical_delta = self.observed_base_axes @ (target[:3, 3]-tcp[:3, 3])
                        # The URDF exports pi/2 as 1.5708, not exact pi/2.
                        np.testing.assert_allclose(physical_delta, np.eye(3)[axis]*sign*40.*sensitivity, atol=.001)
                        np.testing.assert_allclose(target[:3, :3], tcp[:3, :3], atol=1e-10)

    def test_six_physical_rotations_from_tilted_hand_keep_flange_center(self):
        for axis, raw_direction in enumerate(self.tracking_directions):
            for sign in (-1., 1.):
                with self.subTest(axis=axis, sign=sign):
                    tcp, hand, initial_rotation = self.start()
                    clutch = Clutch("left")
                    clutch.target(hand, tcp, .5)
                    angle = sign*math.radians(23.)
                    raw_delta = Rotation.from_rotvec(angle*np.array(raw_direction)).as_matrix()
                    hand["pose"][3:] = Rotation.from_matrix(raw_delta @ initial_rotation).as_quat()
                    target = clutch.target(hand, tcp, .5)
                    actual_delta = self.observed_base_axes @ (target[:3, :3] @ tcp[:3, :3].T) @ self.observed_base_axes.T
                    expected_delta = Rotation.from_rotvec(np.eye(3)[axis]*angle*.5).as_matrix()
                    np.testing.assert_allclose(actual_delta, expected_delta, atol=1e-5)
                    np.testing.assert_allclose(target[:3, 3], tcp[:3, 3], atol=1e-10)

    def test_regrip_uses_new_flange_and_hand_pose_without_absolute_rotation(self):
        tcp, hand, _ = self.start()
        clutch = Clutch("left")
        clutch.target(hand, tcp, .25)
        hand["pose"][1] += .2
        self.assertGreater(np.linalg.norm(clutch.target(hand, tcp, .25)[:3, 3]-tcp[:3, 3]), 49.)
        hand["grip"] = 0.
        self.assertIsNone(clutch.target(hand, tcp, .25))
        actual = tcp.copy()
        actual[:3, 3] += [4., -2., 1.]
        actual[:3, :3] = Rotation.from_rotvec(np.array([.02, -.03, .01])).as_matrix() @ actual[:3, :3]
        hand["pose"] = [.18, .2, -.5, *Rotation.from_rotvec(np.array([-.3, .2, .1])).as_quat()]
        hand["grip"] = 1.
        np.testing.assert_allclose(clutch.target(hand, actual, .25), actual, atol=1e-9)

    def test_right_arm_now_uses_its_own_mount_rotation(self):
        tcp, hand, initial_rotation = self.start()
        clutch = Clutch("right")
        clutch.target(hand, tcp, .5)
        hand["pose"][:3] = np.array(hand["pose"][:3]) + [.03, .04, -.05]
        hand["pose"][3:] = Rotation.from_matrix(
            Rotation.from_rotvec(np.array([0., .2, 0.])).as_matrix() @ initial_rotation).as_quat()
        target = clutch.target(hand, tcp, .5)
        np.testing.assert_allclose(target[:3, 3]-tcp[:3, 3],
            [5./math.sqrt(2.), 45./math.sqrt(2.), 15.], atol=.001)
        np.testing.assert_allclose(target[:3, :3],
            Rotation.from_rotvec(np.array([-1., 1., 0.])*.1/math.sqrt(2.)).as_matrix() @ tcp[:3, :3], atol=1e-5)

    def test_corrected_six_dof_goals_pass_vendor_ik_and_robot_validation(self):
        kin = VendorKinematics()
        hw = hardware()
        for axis in range(6):
            for sign in (-1., 1.):
                with self.subTest(axis=axis, sign=sign):
                    tcp, hand, initial_rotation = self.start()
                    clutch = Clutch("left")
                    clutch.target(hand, tcp, 1.)
                    raw_direction = np.array(self.tracking_directions[axis % 3])
                    if axis < 3:
                        hand["pose"][:3] = np.array(hand["pose"][:3])+sign*.0004*raw_direction
                    else:
                        raw_delta = Rotation.from_rotvec(sign*math.radians(.1)*raw_direction).as_matrix()
                        hand["pose"][3:] = Rotation.from_matrix(raw_delta @ initial_rotation).as_quat()
                    goal = clutch.target(hand, tcp, 1.)
                    packet = request(kin, hw, {"left": goal}, 1, 10.)
                    targets = validate(hw, packet, 10.)
                    actual = np.array(tcp_forward_matrix(targets["left"]))
                    mm, deg = pose_error(goal, actual)
                    self.assertLess(mm, .001)
                    self.assertLess(deg, .001)
                    physical_delta = self.observed_base_axes @ (actual[:3, 3]-tcp[:3, 3])
                    expected = np.eye(3)[axis]*sign*.4 if axis < 3 else np.zeros(3)
                    np.testing.assert_allclose(physical_delta, expected, atol=.001)
                    points = position_trajectory(hw["position"], targets, hw["position"])
                    self.assertEqual(points[-1][1][7:], hw["position"][7:])


class PlanningTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.kin = VendorKinematics()

    def targets(self, hw, axis=0, side_names=ARMS):
        return {s: target_matrix(tcp_forward_matrix([hw["command_position"][j-1] for j in ARMS[s]]),
                axis, 1 if s == "left" else -1, .4 if axis < 3 else .1) for s in side_names}

    def test_both_arms_all_six_axes_vendor_ik_and_held_body(self):
        hw = hardware()
        for axis in range(6):
            r = request(self.kin, hw, self.targets(hw, axis), 1, 10.)
            targets = validate(hw, r, 10.)
            self.assertEqual(set(targets), {"left", "right"})
            points = position_trajectory(hw["position"], targets, hw["position"])
            for _, q in points:
                self.assertEqual(q[14:], hw["position"][14:])
            self.assertEqual(len(points), 1)
            self.assertEqual(points[0][0], 0.)
            for side, joints in ARMS.items():
                self.assertEqual([points[0][1][j-1] for j in joints], targets[side])

    def test_receiver_rejects_stale_tampered_model_limits_and_body_injection(self):
        hw = hardware()
        r = request(self.kin, hw, self.targets(hw), 1, 10.)
        for mutation in ({"created": 9.7}, {"created": math.nan}, {"id": True},
                         {"model": "bad"}, {"kind": "fairino_pico_bimanual_v1"},
                         {"arms": {"body": {}}}):
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                validate(hw, {**r, **mutation}, 10.)
        bad = copy.deepcopy(r); bad["arms"]["left"]["target"][0] += .001
        with self.assertRaisesRegex(ValueError, "正解"): validate(hw, bad, 10.)
        bad = copy.deepcopy(r); bad["arms"]["right"]["target"][0] = math.radians(176.)
        with self.assertRaisesRegex(ValueError, "模型限制"): validate(hw, bad, 10.)

    def test_full_reachable_goal_above_old_tcp_and_joint_caps_is_preserved(self):
        hw = hardware(); goals = {}
        for side, joints in ARMS.items():
            q = [hw["command_position"][j-1] for j in joints]
            q[-1] += math.radians(6.)
            goals[side] = tcp_forward_matrix(q)
        r = request(self.kin, hw, goals, 1, 10.)
        targets = validate(hw, r, 10.)
        for side, entry in r["arms"].items():
            actual = tcp_forward_matrix(entry["target"])
            mm, deg = pose_error(goals[side], actual)
            self.assertLess(mm, .001)
            self.assertLess(deg, .001)
            mm, deg = pose_error(tcp_forward_matrix(entry["reference"]), actual)
            self.assertGreater(mm, .8)
            self.assertGreater(deg, .2)
            self.assertGreater(max(abs(math.degrees(a-b)) for a, b in zip(entry["target"], entry["reference"])), 2.)
        points = position_trajectory(hw["position"], targets, hw["position"])
        for side, joints in ARMS.items():
            self.assertEqual([points[0][1][j-1] for j in joints], targets[side])

    def test_reference_angle_difference_is_not_a_hidden_speed_limit(self):
        hw = hardware()
        r = request(self.kin, hw, self.targets(hw), 1, 10.)
        hw["command_position"][0] += math.radians(3.)
        validate(hw, r, 10.)

    def test_unreachable_goal_is_rejected_without_partial_arm_request(self):
        hw = hardware(); goals = self.targets(hw)
        goals["right"][0][3] += 5000.
        with self.assertRaises(ValueError): request(self.kin, hw, goals, 1, 10.)


class EndSimulation(Exception): pass


class ClientTests(unittest.TestCase):
    def test_nonmoving_check_waits_for_ros_cleanup_without_false_feedback_timeout(self):
        from vr_client import operate
        hw = hardware()
        with tempfile.TemporaryDirectory() as directory:
            c = NS(events=queue.Queue(), done=threading.Event(), last_status=9.4,
                latest={"hardware": hw, "stamp": 9.4}, log_path=Path(directory)/"check.jsonl")
            c.events.put({"checked": True, "hardware": hw})
            c.send = lambda **kw: c.done.set()
            args = NS(sensitivity=.25, execute=False)
            with patch("vr_client.time.monotonic", return_value=10.), patch("vr_client.Clutch", wraps=Clutch) as factory, contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(operate(args, VendorKinematics(), None, c), 0)
                self.assertEqual([call.args for call in factory.call_args_list], [("left",), ("right",)])
                self.assertTrue(all(call.kwargs == {"mode": "first_person", "rotation_mode": "first_person"}
                                    for call in factory.call_args_list))


class RobotLoopTests(PlanningTests):
    def simulate(self, mode):
        p = probe(); p.__class__ = VRProbe
        clock = [10.]; ticks = [0]; published = []; last_request = [None]
        p.node = NS(get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: clock[0])),
            create_publisher=lambda *a: NS(get_subscription_count=lambda: 1, publish=published.append))
        def prepare():
            p.hw_time = p.received = clock[0]; p.packet["ack"] = clock[0]
            rp.Probe.enable_all(p); p.ready = True
        p.prepare = prepare
        def pump(seconds=.02, guarded=True):
            clock[0] += seconds; ticks[0] += 1
            if getattr(p, "path", None) is not None:
                q = p.path[-1][1][:]
                # Discrete position follower; estimate measured velocity from
                # successive commands, not an interpolator's derivative.
                v = [(b-a)/seconds for a, b in zip(p.hw["position"], q)]
                p.hw["position"] = q[:]; p.hw["command_position"] = q[:]; p.hw["velocity"] = v[:]
            p.hw_time = p.received = clock[0]; p.packet["ack"] = clock[0]
            if ticks[0] > 90: raise EndSimulation()
            if ticks[0] < 22:
                r = request(self.kin, p.hw, self.targets(p.hw), ticks[0], clock[0])
                r["_wrist_guard"] = p.wrist.active
                last_request[0] = r; p.packet["request"] = r
            elif mode == "release": p.packet["request"] = None
            elif mode == "stale": pass
            elif mode == "estop": p.gpio = lambda: [1, 1]
            elif mode == "one_release":
                if ticks[0] < 55:
                    p.packet["request"] = request(self.kin, p.hw, self.targets(p.hw, side_names=("right",)), ticks[0], clock[0])
                    p.packet["request"]["_wrist_guard"] = p.wrist.active
                    if ticks[0] > 40:
                        self.assertLess(max(abs(x) for x in p.hw["velocity"][:7]), 1e-6)
                        self.assertGreater(max(abs(x) for x in p.hw["velocity"][7:14]), .0001)
                else: p.packet["request"] = None
            if guarded: p.guard()
        p.pump = pump
        stubs = {**STUBS, "JointTrajectory": lambda: NS(joint_names=[], points=[], header=NS(stamp=None))}
        with patch.dict(rp.__dict__, stubs), patch("vr_robot.time.monotonic", lambda: clock[0]):
            if mode == "estop":
                with self.assertRaisesRegex(RuntimeError, "急停"): p.run()
                self.assertTrue(p.cleanup())
                self.assertFalse(any(p.hw["enabled"]))
            else:
                with self.assertRaises(EndSimulation): p.run()
                self.assertIsNone(p.path)
                self.assertEqual(p.hw["velocity"], [0.]*20)
            if mode == "stale": self.assertTrue(any("jog_rejected" in e for e in p.events))
            self.assertGreater(len(published), 10)
            for message in published:
                self.assertEqual(len(message.points), 1)
                self.assertEqual(message.points[0].time_from_start, 0.)
                self.assertEqual(message.joint_names, rp.JOINTS)
                self.assertEqual(message.points[-1].velocities, [])
                self.assertEqual(message.points[-1].accelerations, [])
                for point in message.points: self.assertEqual(point.positions[14:], p.baseline[14:])

    def test_release_both_stops_without_disabling_hold(self): self.simulate("release")
    def test_release_one_stops_only_that_arm(self): self.simulate("one_release")
    def test_stale_motion_lease_stops_despite_live_ssh(self): self.simulate("stale")
    def test_estop_disables_all_twenty_axes(self): self.simulate("estop")

    def test_old_arm_speed_trip_is_removed_but_body_and_hardware_guards_remain(self):
        p = probe(); p.__class__ = VRProbe
        rp.Probe.enable_all(p)
        p.moving_joints = list(range(1, 15))
        p.hw["velocity"][0] = math.radians(30.)
        p.guard()
        p.hw["errorcode"][0] = 1
        with self.assertRaisesRegex(RuntimeError, "故障"): p.guard()
        p.hw["errorcode"][0] = 0
        p.hw["position"][17] += math.radians(.4)
        with self.assertRaisesRegex(RuntimeError, "保持"): p.guard()

    def test_vr_held_speed_threshold_is_one_degree_per_second(self):
        p = probe(); p.__class__ = VRProbe
        rp.Probe.enable_all(p)
        p.selected = None
        for active in (list(range(1, 15)), None):
            p.moving_joints = active
            held = range(15, 21) if active else range(1, 21)
            for joint in held:
                for speed in (-1., -.535, .535, 1.):
                    with self.subTest(active=active, joint=joint, speed=speed):
                        p.hw["velocity"][joint-1] = math.radians(speed)
                        p.guard()
                for speed in (-1.01, 1.01):
                    with self.subTest(active=active, joint=joint, speed=speed):
                        p.hw["velocity"][joint-1] = math.radians(speed)
                        with self.assertRaisesRegex(RuntimeError, f"J{joint} 速度 .*1.0°/s"):
                            p.guard()
                p.hw["velocity"][joint-1] = 0.

    def test_keyboard_held_speed_threshold_stays_half_a_degree(self):
        p = probe()
        rp.Probe.enable_all(p)
        p.hw["velocity"][15] = math.radians(.535)
        with self.assertRaisesRegex(RuntimeError, "J16 速度 .*0.5°/s"):
            p.guard()


if __name__ == "__main__": unittest.main()
