"""Opt-in Trigger routing and pause behavior with a fake shared Session."""
import json
from types import SimpleNamespace as NS
import unittest

from vr_input import decode_packet
from vr_grippers import GripperTeleop


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
