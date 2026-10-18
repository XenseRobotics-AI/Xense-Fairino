import math
import unittest
from control_core import (JOINTS, check_feedback, check_input, key_twist,
                          normalise_hardware, neutral_duration, DEFAULT_LINEAR,
                          DEFAULT_ANGULAR, DRIVE_FEEDBACK_LIMIT)


def hardware():
    return {"joint_names": JOINTS[:], "position": [0.0]*8, "velocity": [0.0]*8,
            "command_position": [0.0]*8, "command_velocity": [0.0]*8,
            "stateword": [0x40]*8, "errorcode": [0]*8, "hexerrorcode_curerrcode": 0}


class ControlTests(unittest.TestCase):
    def test_translation_and_rotation_are_independent(self):
        self.assertEqual(key_twist("a"), (0, DEFAULT_LINEAR, 0))
        self.assertEqual(key_twist("d"), (0, -DEFAULT_LINEAR, 0))
        self.assertEqual(key_twist("j"), (0, 0, DEFAULT_ANGULAR))
        self.assertEqual(key_twist("l"), (0, 0, -DEFAULT_ANGULAR))
        vx, vy, wz = key_twist("wdj")
        self.assertAlmostEqual(math.hypot(vx, vy), DEFAULT_LINEAR)
        self.assertLess(vy, 0)
        self.assertEqual(wz, DEFAULT_ANGULAR)

    def test_release_and_opposite_keys_stop(self):
        self.assertEqual(key_twist(""), (0, 0, 0))
        self.assertEqual(key_twist("wasdjl"), (0, 0, 0))
        self.assertEqual(key_twist("wsj"), (0, 0, DEFAULT_ANGULAR))

    def test_joint_mapping_uses_names(self):
        raw = hardware()
        raw["joint_names"] = list(reversed(JOINTS))
        raw["errorcode"][2] = 5
        self.assertEqual(normalise_hardware(raw)["errorcode"][5], 5)

    def test_missing_duplicate_and_nonfinite_feedback_rejected(self):
        for field, value in (("joint_names", ["j1"]*8), ("velocity", []),
                             ("position", [float("nan")]*8)):
            raw = hardware(); raw[field] = value
            with self.assertRaises(ValueError):
                normalise_hardware(raw)

    def test_stale_estop_and_drive_fault_prevent_motion(self):
        hw = normalise_hardware(hardware())
        check_feedback(hw, .01, 0, .01)
        for hw_age, upper, upper_age in ((.5, 0, 0), (0, 0, .5), (0, 235929601, 0)):
            with self.assertRaises(RuntimeError):
                check_feedback(hw, hw_age, upper, upper_age)
        hw["errorcode"][5] = 5
        with self.assertRaisesRegex(RuntimeError, "J6=5"):
            check_feedback(hw, 0, 0, 0)

    def test_speed_and_steering_limit_guard(self):
        for field, index, value in (("velocity", 5, DRIVE_FEEDBACK_LIMIT + .1), ("position", 4, 1.7)):
            hw = normalise_hardware(hardware()); hw[field][index] = value
            with self.assertRaises(RuntimeError):
                check_feedback(hw, 0, 0, 0)

    def test_disconnect_and_delayed_commands_stop_but_focus_only_pauses(self):
        packet = {"keys": ["w"], "ack": 9.9, "focused": True, "stop": False}
        self.assertEqual(check_input(packet, 9.95, 10), ["w"])
        self.assertEqual(check_input({**packet, 'focused': False}, 9.95, 10), [])
        cases = [(packet, 9, 10), ({**packet, "ack": 9}, 10, 10),
                 ({**packet, "ack": float("nan")}, 10, 10),
                 ({**packet, "stop": True}, 10, 10)]
        for item, received, now in cases:
            with self.assertRaises(RuntimeError):
                check_input(item, received, now)

    def test_homing_quintic_peak_speed_is_bounded(self):
        for angle in (0, .2, .72, 1.658):
            duration = neutral_duration([angle, -angle, 0, .01])
            self.assertGreaterEqual(duration, 4)
            self.assertLessEqual(1.875 * max(angle, .01) / duration, .20 + 1e-12)


if __name__ == "__main__":
    unittest.main()
