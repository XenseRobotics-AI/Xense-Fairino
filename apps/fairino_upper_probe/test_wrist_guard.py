"""Offline collision-envelope and hot-plug protocol tests. No robot connection."""
import math
import unittest
import json
import hashlib
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import patch
from wrist_guard import (WristGuard, WristBlocked, GuardClient, GuardSession,
                         BOUNDARY, MARGIN_DEG, SLOW_BAND_DEG, PREDICTION_SECONDS,
                         VERSION, MODEL_SHA256, CONFIG_ID, policy, cap)


def arm(q6, q7):
    return [0.]*5 + [math.radians(q6), math.radians(q7)]


def whole(q6, q7):
    return arm(q6, q7)+[0.]*13


class EnvelopeTests(unittest.TestCase):
    def setUp(self):
        self.guard = WristGuard()

    def test_observed_collision_is_blocked_for_both_signs_and_wrists(self):
        for x in (-52, 52):
            for y in (-55, 55):
                for offset in (0, 7):
                    q, target = [0.]*20, [0.]*20
                    q[offset:offset+7] = arm(x, 0)
                    target[offset:offset+7] = arm(x, y)
                    with self.assertRaises(WristBlocked):
                        self.guard.segment(q, target)

    def test_two_degree_cells_use_smaller_cap_and_two_axis_erosion(self):
        for (a, _), (b, c) in zip(BOUNDARY, BOUNDARY[1:]):
            self.assertEqual(cap((a+b)/2), c)
        self.assertEqual(MARGIN_DEG, 2.)
        self.assertEqual(SLOW_BAND_DEG, 5.)
        self.assertEqual(PREDICTION_SECONDS, .20)
        self.assertAlmostEqual(self.guard.clearance(arm(52, 37.7)[5:]), 0.)
        self.guard.arm_segment(arm(52, 30), arm(52, 37.6))
        with self.assertRaises(WristBlocked):
            self.guard.arm_segment(arm(52, 30), arm(52, 37.8))
        self.assertAlmostEqual(self.guard.clearance(arm(68, 0)[5:]), 0.)
        with self.assertRaises(WristBlocked):
            self.guard.arm_segment(arm(67, 0), arm(68.01, 0))

    def test_safe_endpoints_can_have_an_unsafe_interior(self):
        a, b = arm(0, 70), arm(60, 30)
        self.assertGreater(self.guard.clearance(a[5:]), 0)
        self.assertGreater(self.guard.clearance(b[5:]), 0)
        with self.assertRaises(WristBlocked):
            self.guard.arm_segment(a, b)

    def test_home_wrist_paths_are_clear(self):
        for sign in (-1, 1):
            a, b = arm(0, 0), arm(-51.971206, sign*6.081857)
            self.guard.arm_segment(a, b)
            self.guard.arm_segment(b, a)

    def test_retreat_inside_margin_allowed_but_outward_and_collision_pose_blocked(self):
        self.guard.arm_segment(arm(52, 39), arm(52, 36))
        self.guard.arm_segment(arm(52, 39), arm(51, 39))
        with self.assertRaises(WristBlocked):
            self.guard.arm_segment(arm(52, 39), arm(52, 40))
        with self.assertRaises(WristBlocked):
            self.guard.arm_segment(arm(52, 55), arm(52, 30))

    def test_predictive_reserve_accounts_for_actual_wrist_velocity(self):
        self.guard.arm_segment(arm(52, 36), arm(52, 37), [0.]*7)
        with self.assertRaises(WristBlocked):
            self.guard.arm_segment(arm(52, 36), arm(52, 37), arm(0, 20))
        self.guard.arm_segment(arm(52, 39), arm(52, 36), arm(0, -5))

    def test_approaching_slows_but_retreat_does_not(self):
        self.assertLess(self.guard.approach_scale(arm(52, 36), arm(52, 37)), .5)
        self.assertEqual(self.guard.approach_scale(arm(52, 36), arm(52, 35)), 1)

    def test_full_cubic_reversal_is_checked_not_just_endpoints(self):
        q = whole(52, 30); v0 = [0.]*20; v1 = [0.]*20
        v0[6], v1[6] = math.radians(100), -math.radians(100)
        with self.assertRaises(WristBlocked):
            self.guard.trajectory([(0., q, v0), (.4, q, v1)])
        self.guard.trajectory([(0., q, [0.]*20), (.4, q, [0.]*20)])

    def test_disabled_bypasses_only_wrist_geometry_and_finite_inputs_still_required(self):
        disabled = WristGuard(False)
        disabled.arm_segment(arm(52, 0), arm(52, 55))
        with self.assertRaises(WristBlocked):
            disabled.arm_segment(arm(52, 0), [float("nan")]*7)

    def test_verified_boxes_contain_dense_cubic_samples(self):
        # Real cubic trajectories, including reversals near the curved boundary.
        from random import Random
        rng = Random(42)
        checked = 0
        for _ in range(150):
            a = whole(rng.uniform(-63, 63), rng.uniform(-30, 30))
            b = whole(math.degrees(a[5])+rng.uniform(-2, 2),
                      math.degrees(a[6])+rng.uniform(-2, 2))
            va, vb = [0.]*20, [0.]*20
            for k in (5, 6):
                va[k], vb[k] = math.radians(rng.uniform(-4, 4)), math.radians(rng.uniform(-4, 4))
            try:
                self.guard.trajectory([(0., a, va), (.3, b, vb)])
            except WristBlocked:
                continue
            checked += 1
            for n in range(101):
                u = n/100
                pair = [a[k]+(-2*u**3+3*u**2)*(b[k]-a[k])
                        +(u**3-2*u*u+u)*.3*va[k]+(u**3-u*u)*.3*vb[k] for k in (5, 6)]
                self.assertGreaterEqual(self.guard.clearance(pair), -1e-7)
        self.assertGreater(checked, 100)


class HotPlugTests(unittest.TestCase):
    def setUp(self):
        self.receiver, self.client = GuardSession(), GuardClient()
        self.hw = {"velocity": [0.]*20, "command_velocity": [0.]*20}

    def exchange(self, request=None, idle=True):
        wire = self.client.filter(request, self.receiver.status())
        return self.receiver.filter(
            {"request": wire, "wrist_guard": self.client.wanted}, self.hw, idle)

    def ready(self):
        self.exchange()
        self.exchange()
        self.assertTrue(self.client.released)

    def test_start_requires_both_peers_and_an_empty_input(self):
        self.assertIsNone(self.exchange({"id": 1}))
        self.assertIsNone(self.exchange({"id": 2}))
        self.assertFalse(self.client.released)
        self.ready()
        self.assertIsNone(self.exchange({"id": 2}))
        self.assertEqual(self.exchange({"id": 3})["id"], 3)

    def test_switch_holds_until_idle_and_measured_and_commanded_speeds_stop(self):
        self.ready()
        old = self.exchange({"id": 9})
        self.client.switch(False)
        self.assertIsNone(self.exchange(old, idle=False))
        self.assertTrue(self.receiver.guard.enabled)
        for name in ("velocity", "command_velocity"):
            self.hw[name][6] = .01
            self.exchange()
            self.assertTrue(self.receiver.guard.enabled)
            self.hw[name][6] = 0.
        self.ready()
        self.assertFalse(self.receiver.guard.enabled)
        self.assertIsNone(self.exchange(old))
        self.assertEqual(self.exchange({"id": 10})["id"], 10)
        self.client.switch(True)
        self.ready()
        self.assertTrue(self.receiver.guard.enabled)

    def test_stale_epoch_missing_module_and_mismatch_never_pass_requests(self):
        self.ready()
        for config in (None, {**policy(), "config_id": "wrong"}, {**policy(), "enabled": 1}):
            value = self.receiver.filter({"wrist_guard": config, "request": {"id": 1}}, self.hw, True)
            self.assertIsNone(value)
            self.assertTrue(self.receiver.status()["error"])
        self.client.switch(False); self.ready()
        self.assertIsNone(self.receiver.filter(
            {"wrist_guard": policy(), "request": {"id": 100}}, self.hw, True))
        self.assertIsNone(self.receiver.filter(
            {"wrist_guard": self.client.wanted, "request": {"id": 100, "_wrist_guard": policy()}}, self.hw, True))

    def test_previous_five_degree_config_cannot_complete_handshake(self):
        old_id = hashlib.sha256(json.dumps(
            [VERSION, MODEL_SHA256, BOUNDARY, 5., SLOW_BAND_DEG, PREDICTION_SECONDS]
        ).encode()).hexdigest()[:16]
        self.assertNotEqual(CONFIG_ID, old_id)
        old_policy = {**policy(), "config_id": old_id}
        self.assertFalse(self.client.ready({"policy": old_policy}))
        self.assertIsNone(self.receiver.filter(
            {"wrist_guard": old_policy, "request": None}, self.hw, True))
        self.assertIsNone(self.receiver.active)
        self.assertIn("边界不一致", self.receiver.status()["error"])
        self.ready()
        self.assertEqual(self.receiver.status()["margin_deg"], 2.)

    def test_bad_request_fails_closed(self):
        self.ready()
        for request in (True, [], {"id": True}, {"id": "1"}):
            self.assertIsNone(self.receiver.filter(
                {"wrist_guard": policy(), "request": request}, self.hw, True))

    def test_missing_module_refuses_enable_before_robot_command(self):
        import robot_probe as rp
        from test_probe import fake_probe
        probe = fake_probe()
        probe.wrist = GuardSession()
        probe.packet.pop("wrist_guard")
        clock = [0.]
        probe.pump = lambda *_: clock.__setitem__(0, clock[0]+1.)
        with patch.object(rp.time, "monotonic", lambda: clock[0]), self.assertRaisesRegex(RuntimeError, "拒绝使能"):
            probe.enable_all()
        self.assertEqual(probe.commands, [])


class TransportTests(unittest.TestCase):
    def test_gripper_protocol_requires_its_own_worker_and_unknown_protocol_is_rejected(self):
        from upper_keyboard import Connection
        with patch('pexpect.spawn', side_effect=AssertionError('must reject before SSH')):
            with self.assertRaisesRegex(ValueError, '独立的夹爪 worker'):
                Connection(NS(), request_protocol='gripper')
            with self.assertRaisesRegex(ValueError, '未知控制协议'):
                Connection(NS(), worker_command='unused', request_protocol='unprotected')

    def test_snapshot_preserves_optional_command_velocity_and_checks_finiteness(self):
        from test_probe import hardware
        from probe_core import snapshot, JOINTS
        raw = hardware()
        raw.update(joint_names=JOINTS, hexerrorcode_curerrcode=0,
                   command_velocity=[0.]*19+[.03])
        self.assertEqual(snapshot(raw)["command_velocity"][-1], .03)
        raw["command_velocity"][-1] = float("nan")
        with self.assertRaisesRegex(RuntimeError, "command_velocity"):
            snapshot(raw)

    def test_unacknowledged_motion_is_blocked_but_readonly_benchmark_still_sends(self):
        from upper_keyboard import Connection
        connection = Connection.__new__(Connection)
        connection.wrist = GuardClient()
        connection.done = threading.Event()
        connection.write_lock = threading.Lock()
        connection.last_status = time.monotonic()
        connection.feedback_timeout = .5
        connection.latest = {"stamp": time.monotonic()}
        connection.pexpect = NS(EOF=EOFError)
        sent = []
        connection.child = NS(sendline=lambda line: sent.append(json.loads(line)))
        connection.link_check = False
        connection.send(request={"id": 1})
        self.assertIsNone(sent[-1]["request"])
        connection.link_check = True
        connection.send(request={"id": 2})
        self.assertIsNone(sent[-1]["request"])  # Still an ordinary/unconfirmed peer.
        connection.latest["link_check"] = True
        connection.send(request={"id": 3})
        self.assertEqual(sent[-1]["request"]["id"], 3)


if __name__ == "__main__":
    unittest.main()
