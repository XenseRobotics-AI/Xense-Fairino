"""Offline facing-direction, freeze continuity, and real client routing checks."""
import contextlib
import copy
import io
import json
import queue
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np
import paths
from lerobot.utils.rotation import Rotation
from mounts import tcp_in_chest
from tcp_core import ARMS, pose_error, tcp_forward_matrix
from test_tcp import hardware
from vendor_kinematics import VendorKinematics
from vr_input import Clutch, controller_sides, decode_packet
from vr_client import operate
from vr_controls import JoystickControls


def hand(position=(0., 0., 0.), angle=(0., 0., 0.), grip=1., primary=False, secondary=False):
    return {"pose": [*position, *Rotation.from_rotvec(np.array(angle)).as_quat()],
            "grip": grip, "primary": primary, "secondary": secondary}


class FacingTests(unittest.TestCase):
    def test_both_arms_six_room_directions_in_each_mode(self):
        directions = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
        # Independent expected robot-chest displacements for user forward/left/up.
        expected = {"first_person": np.eye(3), "mirror": np.diag([1., -1., 1.]),
                    "mirror_same_direction": np.diag([-1., -1., 1.])}
        for mode, axes in expected.items():
            for side in ARMS:
                for axis in range(3):
                    for sign in (-1., 1.):
                        c = Clutch(side, mode)
                        c.target(hand(), np.eye(4), .5)
                        result = c.target(hand(sign * .04 * directions[axis]), np.eye(4), .5)
                        chest_delta = tcp_in_chest(side, result)[:3, 3] - tcp_in_chest(side, np.eye(4))[:3, 3]
                        np.testing.assert_allclose(chest_delta, sign * 20. * axes[:, axis], atol=1e-8)

    def test_mirror_rotations_use_axial_vector_sign_and_keep_tcp_fixed(self):
        # In a reflection, angular vectors transform by det(M)*M, not just M.
        directions = ([0., 0., -1.], [-1., 0., 0.], [0., 1., 0.])
        for side in ARMS:
            for axis, direction in enumerate(directions):
                for sign in (-1., 1.):
                    c = Clutch(side, "mirror")
                    tcp = np.eye(4); tcp[:3, 3] = [40., -25., 350.]
                    start_angle = np.array([.13, -.21, .09])
                    c.target(hand(angle=start_angle), tcp, .5)
                    raw_rotation = (Rotation.from_rotvec(sign*.2*np.array(direction)).as_matrix()
                                    @ Rotation.from_rotvec(start_angle).as_matrix())
                    packet = hand(); packet["pose"][3:] = Rotation.from_matrix(raw_rotation).as_quat()
                    target = c.target(packet, tcp, .5)
                    before, after = tcp_in_chest(side, tcp), tcp_in_chest(side, target)
                    actual = after[:3, :3] @ before[:3, :3].T
                    expected_axis = -np.diag([1., -1., 1.])[:, axis]
                    np.testing.assert_allclose(actual, Rotation.from_rotvec(sign*.1*expected_axis).as_matrix(), atol=1e-9)
                    np.testing.assert_allclose(after[:3, 3], before[:3, 3], atol=1e-9)
                    self.assertAlmostEqual(np.linalg.det(after[:3, :3]), 1.)

    def test_routing_can_be_explicitly_overridden(self):
        self.assertEqual(controller_sides("mirror"), {"left": "right", "right": "left"})
        self.assertEqual(controller_sides("mirror", "same"), {"left": "left", "right": "right"})
        self.assertEqual(controller_sides("first_person", "crossed"), {"left": "right", "right": "left"})
        with self.assertRaises(ValueError): controller_sides("invalid")


class FreezeTests(unittest.TestCase):
    def test_freeze_holds_current_command_and_release_discards_accumulated_motion(self):
        for mode in ("first_person", "mirror", "mirror_same_direction"):
            for side in ARMS:
                for freeze_pos, freeze_rot in ((True, False), (False, True), (True, True)):
                    with self.subTest(mode=mode, side=side, freeze_pos=freeze_pos, freeze_rot=freeze_rot):
                        c = Clutch(side, mode); tcp = np.eye(4)
                        c.target(hand(), tcp, .5)
                        pending = c.target(hand((.1, .1, .1), (.1, .2, .1)), tcp, .5)
                        self.assertGreater(np.linalg.norm(pending[:3, 3]), 50.)
                        # Robot is behind its last requested target when freeze is pressed.
                        current = np.eye(4); current[:3, 3] = [1., 2., 3.]
                        current[:3, :3] = Rotation.from_rotvec(np.array([.01, -.02, .03])).as_matrix()
                        hold = c.target(hand((.2, .2, .2), (.2, .3, .2)), current, .5,
                                        freeze_position=freeze_pos, freeze_rotation=freeze_rot)
                        moved = hand((.6, .5, .7), (.5, .6, .7))
                        during = c.target(moved, current, .5, freeze_position=freeze_pos, freeze_rotation=freeze_rot)
                        # The release packet also moves: that final held interval must be discarded.
                        release = hand((.7, .8, .9), (.7, .8, .9))
                        after = c.target(release, current, .5)
                        repeated = c.target(release, current, .5)
                        for value in (hold, during, after, repeated):
                            if freeze_pos: np.testing.assert_allclose(value[:3, 3], current[:3, 3], atol=1e-9)
                            if freeze_rot: np.testing.assert_allclose(value[:3, :3], current[:3, :3], atol=1e-9)
                        if not freeze_pos: self.assertGreater(np.linalg.norm(during[:3, 3]-hold[:3, 3]), 50.)
                        if not freeze_rot: self.assertGreater(pose_error(hold, during)[1], 1.)
                        # Post-release increments resume normally, relative to the release packet.
                        next_hand = copy.deepcopy(release); next_hand["pose"][0] += .002
                        rot = Rotation.from_quat(next_hand["pose"][3:]).as_matrix()
                        next_hand["pose"][3:] = Rotation.from_matrix(
                            Rotation.from_rotvec(np.array([.002, 0., 0.])).as_matrix() @ rot).as_quat()
                        resumed = c.target(next_hand, current, .5)
                        if freeze_pos: self.assertAlmostEqual(np.linalg.norm(resumed[:3, 3]-after[:3, 3]), 1., places=7)
                        if freeze_rot: self.assertAlmostEqual(pose_error(after, resumed)[1], np.degrees(.001), places=6)

    def test_grip_release_discards_freeze_reference_and_regrip_starts_at_current_tcp(self):
        c = Clutch("right", "mirror")
        tcp = np.eye(4)
        c.target(hand(), tcp, .25, freeze_position=True, freeze_rotation=True)
        self.assertIsNone(c.target(hand(grip=0.), tcp, .25, freeze_position=True, freeze_rotation=True))
        self.assertFalse(c.frozen_position or c.frozen_rotation)
        tcp[:3, 3] = [10., 20., 30.]
        target = c.target(hand((.7, .5, -.2), (.3, .2, .1)), tcp, .25)
        np.testing.assert_allclose(target, tcp, atol=1e-9)

    def test_freezing_one_clutch_does_not_change_the_other(self):
        clutches = {s: Clutch(s, "mirror") for s in ARMS}
        for c in clutches.values(): c.target(hand(), np.eye(4), 1.)
        frozen = clutches["left"].target(hand((.02, 0., 0.)), np.eye(4), 1., freeze_position=True)
        moving = clutches["right"].target(hand((.02, 0., 0.)), np.eye(4), 1.)
        np.testing.assert_allclose(frozen[:3, 3], 0., atol=1e-9)
        self.assertAlmostEqual(np.linalg.norm(moving[:3, 3]), 20.)


class DoubleTapTests(unittest.TestCase):
    def frame(self, stamp, x=False, y=False, left_click=False, right_click=False, axis_y=0.):
        return dict(received_monotonic=stamp,
                    left=dict(primary=x, secondary=y, axis_click=left_click, axis_y=axis_y),
                    right=dict(primary=False, secondary=False, axis_click=right_click))

    def feed(self, controls, *samples):
        return [controls.update(sample, now=sample['received_monotonic']) for sample in samples]

    def tap_pair(self, button, start=0.):
        return [self.frame(start), self.frame(start+.04, **{button: True}),
                self.frame(start+.08), self.frame(start+.14, **{button: True}), self.frame(start+.18)]

    def test_short_x_lowers_and_y_raises_on_second_release_only(self):
        for button, direction in (('x', -1), ('y', 1)):
            controls = JoystickControls()
            results = self.feed(controls, *self.tap_pair(button))
            self.assertEqual([r['sensitivity'] for r in results], [0, 0, 0, 0, direction])
            self.assertTrue(all(not r['home'] and not r['pause'] for r in results))

    def test_explicit_loop_clock_fallback_and_repeated_frame_are_supported(self):
        controls = JoystickControls()
        results = []
        for sample in self.tap_pair('y'):
            now = sample.pop('received_monotonic')
            results.append(controls.update(sample, now=now)['sensitivity'])
            self.assertEqual(controls.update(sample, now=now)['sensitivity'], 0)
        self.assertEqual(results, [0, 0, 0, 0, 1])

    def test_held_button_is_not_a_tap_even_as_second_press(self):
        for button in ('x', 'y'):
            for first_long in (False, True):
                controls = JoystickControls()
                samples = [self.frame(0.), self.frame(.02, **{button: True})]
                if first_long:
                    samples += [self.frame(.2, **{button: True}), self.frame(.32)]
                    samples += [self.frame(.4, **{button: True}), self.frame(.45)]
                else:
                    samples += [self.frame(.08), self.frame(.14, **{button: True}),
                                self.frame(.3, **{button: True}), self.frame(.45)]
                self.assertTrue(all(r['sensitivity'] == 0 for r in self.feed(controls, *samples)))

    def test_startup_held_and_mixed_or_simultaneous_buttons_do_not_form_pairs(self):
        sequences = [
            [self.frame(0., y=True), self.frame(.05), self.frame(.1, y=True), self.frame(.15)],
            [self.frame(0.), self.frame(.02, x=True), self.frame(.04),
             self.frame(.06, y=True), self.frame(.08), self.frame(.1, x=True), self.frame(.12)],
            [self.frame(0.), self.frame(.02, x=True), self.frame(.04),
             self.frame(.06, x=True, y=True), self.frame(.08), self.frame(.1, x=True), self.frame(.12)]]
        for samples in sequences:
            self.assertTrue(all(r['sensitivity'] == 0 for r in self.feed(JoystickControls(), *samples)))

    def test_taps_expire_without_extending_gap_on_repeated_frames(self):
        controls = JoystickControls()
        samples = [self.frame(0.), self.frame(.02, y=True), self.frame(.06),
                   self.frame(.2), self.frame(.4), self.frame(.43, y=True), self.frame(.48)]
        self.assertTrue(all(r['sensitivity'] == 0 for r in self.feed(controls, *samples)))

    def test_reset_or_input_gap_discards_first_tap_and_requires_release(self):
        for reset in (False, True):
            controls = JoystickControls()
            self.feed(controls, self.frame(0.), self.frame(.02, y=True), self.frame(.06))
            if reset:
                controls.reset()
            start = .1 if reset else .32
            result = self.feed(controls, self.frame(start, y=True), self.frame(start+.04))
            self.assertEqual([r['sensitivity'] for r in result], [0, 0])

    def test_batched_edges_are_preserved_and_duplicate_or_old_samples_never_retrigger(self):
        controls = JoystickControls()
        samples = self.tap_pair('y')
        batch = dict(samples[-1], samples=samples)
        self.assertEqual(controls.update(batch, now=.18)['sensitivity'], 1)
        for now in (.19, .2, .24):
            self.assertEqual(controls.update(batch, now=now)['sensitivity'], 0)
        controls.reset()
        self.assertEqual(controls.update(batch, now=.24)['sensitivity'], 0)
        self.assertEqual(controls.update(dict(samples[-1], samples=[]), now=.24)['sensitivity'], 0)

    def test_stale_batch_cannot_apply_a_delayed_double_tap(self):
        controls = JoystickControls()
        samples = self.tap_pair('y')
        batch = dict(samples[-1], samples=samples)
        self.assertEqual(controls.update(batch, now=.6)['sensitivity'], 0)

    def test_stick_clicks_clear_taps_and_axes_do_not_change_sensitivity(self):
        controls = JoystickControls()
        self.feed(controls, self.frame(0.), self.frame(.02, y=True), self.frame(.06))
        result = controls.update(self.frame(.1, left_click=True), now=.1)
        self.assertTrue(result['pause'])
        self.assertEqual(result['sensitivity'], 0)
        result = self.feed(controls, self.frame(.12), self.frame(.14, y=True), self.frame(.18))
        self.assertTrue(all(r['sensitivity'] == 0 for r in result))
        for n, y in enumerate((1., -1., 0., 1., 0.)):
            self.assertEqual(controls.update(self.frame(.2+n*.02, axis_y=y), now=.2+n*.02)['sensitivity'], 0)
        self.assertTrue(controls.update(self.frame(.32, right_click=True), now=.32)['home'])
        self.assertFalse(controls.update(self.frame(.34, right_click=True), now=.34)['home'])

    def test_batched_pause_has_priority_over_tap_and_clears_history(self):
        controls = JoystickControls()
        samples = [self.frame(0.), self.frame(.02, left_click=True), *self.tap_pair('y', .04)[1:]]
        result = controls.update(dict(samples[-1], samples=samples), now=.22)
        self.assertTrue(result['pause'])
        self.assertEqual(result['sensitivity'], 0)


class EndClient(Exception): pass


class ClientRoutingTests(unittest.TestCase):
    def run_frames(self, frames, freeze_buttons, mode="mirror"):
        clock = [10.]; packets = []; hw = hardware()
        with tempfile.TemporaryDirectory() as directory:
            c = NS(events=queue.Queue(), done=threading.Event(), last_status=10.,
                   latest={"hardware": hw, "stamp": 10., "ready": True}, log_path=Path(directory)/"run.jsonl")
            from test_probe import connected_wrist
            connected_wrist(c)
            def send(**kw):
                packets.append(copy.deepcopy(kw))
                if len(packets) >= len(frames): raise EndClient()
            c.send = send
            stream = NS(read=lambda: frames[len(packets)])
            args = NS(sensitivity=.25, execute=True, mode=mode, hand_mapping="auto",
                      freeze_buttons=freeze_buttons, no_auto_home=True)
            with patch("vr_client.time.monotonic", side_effect=lambda: clock[0]), \
                 patch("vr_client.time.sleep", side_effect=lambda _: clock.__setitem__(0, clock[0]+.02)), \
                 patch("vr_client.keyboard", contextlib.nullcontext), patch("vr_client.key", return_value=""), \
                 contextlib.redirect_stdout(io.StringIO()), self.assertRaises(EndClient):
                operate(args, VendorKinematics(), stream, c)
            logs = [json.loads(x) for x in c.log_path.with_suffix('.actions.jsonl').read_text().splitlines()]
            logs = [r for r in logs if "request" in r]
        return packets, logs

    def frame(self, left=None, right=None):
        # Exercise raw SDK names -> decoded primary/secondary -> live client.
        raw = {}
        for s, value in (("left", left), ("right", right)):
            value = dict(value or hand(grip=0.))
            value["primaryButton"] = value.pop("primary")
            value["secondaryButton"] = value.pop("secondary")
            raw[s] = value
        return decode_packet(json.dumps({"value": {"timeStampNs": 1, "Controller": raw}}))

    def test_x_y_follow_crossed_hand_and_do_not_adjust_sensitivity_or_pause(self):
        frames = [self.frame(), self.frame(hand()),
                  self.frame(hand((.0004, 0., 0.), primary=True)),
                  self.frame(hand((.0004, 0., 0.), (.0004, 0., 0.), secondary=True)),
                  self.frame(hand(grip=0.))]
        packets, logs = self.run_frames(frames, True)
        self.assertEqual(set(packets[2]["request"]["arms"]), {"right"})
        self.assertEqual(set(packets[3]["request"]["arms"]), {"right"})
        self.assertIsNone(packets[-1]["request"])
        self.assertTrue(all(x['sensitivity'] == .25 for x in logs))
        self.assertTrue(logs[-2]["freeze"]["right"]["rotation"])
        self.assertTrue(logs[-1]["freeze"]["right"]["position"])
        self.assertFalse(logs[-1]["freeze"]["left"]["position"])
        initial = np.array(tcp_forward_matrix(hardware()["command_position"][7:14]))
        translated = np.array(packets[2]["request"]["arms"]["right"]["tcp"]).reshape(4, 4)
        np.testing.assert_allclose(translated[:3, :3], initial[:3, :3], atol=1e-8)

    def test_a_b_can_freeze_both_components_on_robot_left_arm(self):
        frames = [self.frame(), self.frame(right=hand()),
                  self.frame(right=hand((.01, .01, .01), (.1, .1, .1), primary=True, secondary=True))]
        packets, logs = self.run_frames(frames, True)
        self.assertEqual(set(packets[-1]["request"]["arms"]), {"left"})
        self.assertEqual(logs[-1]["freeze"]["left"], {"position": True, "rotation": True})
        self.assertEqual(packets[-1]["request"]["arms"]["left"]["tcp"], packets[1]["request"]["arms"]["left"]["tcp"])

    def test_single_x_no_longer_changes_sensitivity_without_freeze_flag(self):
        frames = [self.frame(), self.frame(hand()), self.frame(hand(primary=True))]
        packets, _ = self.run_frames(frames, False, "first_person")
        self.assertIsNotNone(packets[1]["request"])
        self.assertIsNotNone(packets[2]["request"])

    def test_double_y_changes_sensitivity_and_pauses_with_or_without_freeze(self):
        frames = [self.frame(), self.frame(hand()), self.frame(hand(secondary=True)),
                  self.frame(hand()), self.frame(hand(secondary=True)), self.frame(hand())]
        for freeze in (False, True):
            packets, _ = self.run_frames(frames, freeze, 'first_person')
            self.assertIsNotNone(packets[1]['request'])
            self.assertIsNotNone(packets[4]['request'])
            self.assertIsNone(packets[5]['request'])

    def test_selected_same_room_mode_moves_robot_back_when_operator_reaches_forward(self):
        frames = [self.frame(), self.frame(hand()),
                  self.frame(hand((0., 0., -.0004), primary=True))]
        packets, logs = self.run_frames(frames, True, "mirror_same_direction")
        arm = "right"
        start = np.array(packets[1]["request"]["arms"][arm]["tcp"]).reshape(4, 4)
        target = np.array(packets[2]["request"]["arms"][arm]["tcp"]).reshape(4, 4)
        delta = tcp_in_chest(arm, target)[:3, 3] - tcp_in_chest(arm, start)[:3, 3]
        # At 20 ms, the default 12 ms TCP playback delay samples 40% of
        # the segment. Mapping remains backwards in the robot chest frame.
        np.testing.assert_allclose(delta, [-.04, 0., 0.], atol=1e-8)
        np.testing.assert_allclose(target[:3, :3], start[:3, :3], atol=1e-8)
        self.assertEqual(logs[-1]["mode"], "mirror_same_direction")


if __name__ == "__main__": unittest.main()
