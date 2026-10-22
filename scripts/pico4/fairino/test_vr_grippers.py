"""Opt-in Trigger routing and pause behavior with a fake shared Session."""
import json
import contextlib
import io
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

from vr_input import decode_packet
from vr_grippers import GripperTeleop
from vr_gui_ipc import gripper_status


class FakeSession:
    def __init__(self, *_, **__):
        self.state = dict(ready=True, devices={'left': dict(connected=True, controlled=False)})
        self.commands = []
        self.values = None

    def snapshot(self):
        return self.state

    def command(self, *args, **fields):
        self.commands.append((args, fields))

    def set_targets(self, values):
        self.values = values


def frame(left=0., right=0.):
    return {s: dict(trigger=v, trigger_available=True) for s, v in [('left', left), ('right', right)]}


class GripperTests(unittest.TestCase):
    def setUp(self):
        self.control = GripperTeleop(NS(host='unused', user='unused'), None, FakeSession)
        self.hands = {'left': 'right', 'right': 'left'}

    def test_home_pause_and_held_trigger_never_enable(self):
        self.control.update(frame(), self.hands, False)
        self.control.update(frame(right=1.), self.hands, True)
        self.assertFalse(self.control.session.commands)
        self.control.update(frame(), self.hands, True)
        self.assertEqual(self.control.session.commands[0][0], ('enable', 'both'))

    def test_continuous_intermediate_values_follow_the_arm_hand_mapping(self):
        self.control.update(frame(), self.hands, True)
        self.control.session.state['devices']['left']['controlled'] = True
        self.control.update(frame(), self.hands, True)
        for value in (.1, .25, .5, .9, 1., 0.):
            self.control.update(frame(right=value), self.hands, True)
            self.assertEqual(self.control.session.values, {'left': 1.-value})
        self.control.update(None, self.hands, False)
        self.assertIsNone(self.control.session.values)
        self.control.update(frame(right=1.), self.hands, True)
        self.assertFalse(self.control.session.values)

    def test_missing_side_is_not_mirrored_onto_the_existing_jaw(self):
        self.control.session.state['devices'] = {}
        self.control.update(frame(), self.hands, True)
        self.assertFalse(self.control.session.commands)
        self.assertIn('未连接', self.control.note)

    def test_sdk_failure_is_reported_to_the_parent_control_loop(self):
        self.control.session.state['error'] = 'lost hardware'
        with self.assertRaisesRegex(RuntimeError, 'lost hardware'):
            self.control.update(frame(), self.hands, True)

    def test_selected_speed_reaches_worker_with_device_max_continuous_torque(self):
        control = GripperTeleop(NS(host='unused', user='unused', gripper_speed_rad_s=.9), None, FakeSession)
        control.update(frame(), self.hands, True)
        fields = control.session.commands[0][1]
        self.assertEqual(fields, dict(torque_mode='max_continuous', speed=.9))
        self.assertEqual(control.snapshot()['speed_rad_s'], .9)
        self.assertEqual(control.snapshot()['torque_mode'], 'max_continuous')
        for speed in (0., 1.01, float('nan'), True):
            with self.assertRaises(ValueError):
                GripperTeleop(NS(host='unused', user='unused', gripper_speed_rad_s=speed), None, FakeSession)

    def test_selected_opening_limit_reaches_target_and_invalid_values_never_connect(self):
        args = NS(host='unused', user='unused', left_gripper_max_open_percent=60.)
        control = GripperTeleop(args, None, FakeSession)
        control.update(frame(), self.hands, True)
        control.session.state['devices']['left']['controlled'] = True
        control.update(frame(), self.hands, True)
        control.update(frame(right=.5), self.hands, True)
        self.assertEqual(control.session.values, {'left': .3})
        self.assertEqual(control.snapshot()['max_openings'], {'left': .6, 'right': 1.})
        factory = Mock()
        for value in (-1., 101., True, float('nan')):
            args.left_gripper_max_open_percent = value
            with self.assertRaises(ValueError):
                GripperTeleop(args, None, factory)
        factory.assert_not_called()

    def test_gui_measurement_packet_keeps_source_timestamp_and_excludes_targets(self):
        self.control.session.state.update(received=10.)
        self.control.session.state['devices']['left'].update(position=.3, target=.8, age_ms=20., open_span_rad=1.2)
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            gripper_status(self.control.snapshot())
        header, body = out.getvalue().split(' ', 1)
        self.assertEqual(header, 'WORKBENCH_GRIPPERS')
        record = json.loads(body)
        self.assertEqual(record['received'], 10.)
        self.assertEqual(record['devices']['left'],
                         dict(connected=True, position=.3, age_ms=20., open_span_rad=1.2))


class TriggerPacketTests(unittest.TestCase):
    def test_real_trigger_field_and_missing_vs_invalid_data(self):
        hands = {s: dict(pose=[0.,0.,0.,0.,0.,0.,1.], grip=0., trigger=.37) for s in ('left', 'right')}
        def decode():
            return decode_packet(json.dumps({'value': {'timeStampNs': 1, 'Controller': hands}}))
        self.assertAlmostEqual(decode()['left']['trigger'], .37)
        self.assertTrue(decode()['left']['trigger_available'])
        for value in (-.1, 1.1, True, float('nan'), 'pressed'):
            hands['left']['trigger'] = value
            with self.assertRaises(ValueError):
                decode()
        del hands['left']['trigger']
        self.assertFalse(decode()['left']['trigger_available'])
        self.assertIsNone(decode()['left']['trigger'])


if __name__ == '__main__':
    unittest.main()
