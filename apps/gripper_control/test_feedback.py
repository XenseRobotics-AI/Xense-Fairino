"""Measured opening selection across debug and PICO sessions; no hardware."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from apps.gripper_control.feedback import measured_openings


def state(received=10., **unit):
    return dict(received=received, devices={'right': dict(
        connected=True, age_ms=50., position=.3, target=.9, **unit)})


class FeedbackTests(unittest.TestCase):
    def test_uses_observation_not_command_and_expires_without_new_source_sample(self):
        value = state()
        self.assertEqual(measured_openings(value, now=10.1), {'right': .3})
        self.assertEqual(measured_openings(value, now=10.46), {})
        # Forwarding the same packet does not renew its host reception timestamp.
        self.assertEqual(measured_openings(dict(value), now=10.6), {})
        del value['devices']['right']['position']
        self.assertEqual(measured_openings(value, now=10.1), {})

    def test_selects_latest_actual_sample_instead_of_latest_transport_packet(self):
        old = state(10.2)
        old['devices']['right'].update(age_ms=300., position=.7)
        new = state(10.)
        self.assertEqual(measured_openings(new, old, now=10.25), {'right': .3})
        self.assertEqual(measured_openings(old, new, now=10.25), {'right': .3})
        new['devices']['left'] = dict(connected=True, age_ms=0., position=.8)
        self.assertEqual(measured_openings(old, new, now=10.25), {'right': .3, 'left': .8})

    def test_invalid_disconnected_faulty_or_stale_feedback_is_not_displayed_as_live(self):
        for field, value in [('connected', False), ('position', float('nan')),
                             ('position', -1.), ('position', True), ('age_ms', None),
                             ('age_ms', float('inf')), ('age_ms', -500.)]:
            record = state()
            record['devices']['right'][field] = value
            with self.subTest(field=field, value=value):
                self.assertEqual(measured_openings(record, now=10.1), {})
        for record in (None, [], dict(error='lost', **state()), state(11.),
                       dict(received=10., devices=None), dict(received=10., devices={'right': None})):
            self.assertEqual(measured_openings(record, now=10.1), {})


if __name__ == '__main__':
    unittest.main()
