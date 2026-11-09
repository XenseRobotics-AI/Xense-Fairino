"""Mapping/navigation transport isolation; SSH and hardware are never opened."""
import json
import threading
import time
import unittest
from types import SimpleNamespace as NS

from fairino_slam.chassis import transport
from fairino_slam.chassis.control_core import DEFAULT_LINEAR, DEFAULT_ANGULAR


class TransportTests(unittest.TestCase):
    def connection(self, navigation=True):
        c = transport.Connection.__new__(transport.Connection)
        c.navigation = navigation
        c.done = threading.Event()
        c.last_status = time.monotonic()
        c.last_send = 0.
        c.write_lock = threading.Lock()
        c.speeds = (DEFAULT_LINEAR, DEFAULT_ANGULAR)
        c.latest = {'stamp': 123.}
        c.sent = []
        c.child = NS(sendline=lambda row: c.sent.append(json.loads(row)))
        return c

    def test_navigation_heartbeat_preserves_robot_ack(self):
        c = self.connection()
        c.send()
        self.assertEqual(c.sent[0]['keys'], [])
        self.assertEqual(c.sent[0]['ack'], 123.)
        self.assertFalse(c.sent[0]['stop'])
        self.assertEqual(c.sent[0]['linear'], DEFAULT_LINEAR)

    def test_stale_robot_status_latches_stop_in_packet(self):
        c = self.connection()
        c.last_status -= 1.
        c.send()
        self.assertTrue(c.sent[0]['stop'])

    def test_manual_motion_and_speed_override_are_not_transmitted(self):
        for kwargs in ({'keys': ['w']}, {'speeds': (.2, .4)}):
            with self.subTest(kwargs=kwargs):
                c = self.connection()
                with self.assertRaises(ValueError):
                    c.send(**kwargs)
                self.assertFalse(c.sent)

    def test_mapping_mode_transmits_keys_and_adjusted_speed(self):
        c = self.connection(navigation=False)
        c.send(keys='wdj', speeds=(.2, .4))
        self.assertEqual(c.sent[0]['keys'], ['d', 'j', 'w'])
        self.assertEqual((c.sent[0]['linear'], c.sent[0]['angular']), (.2, .4))
        self.assertFalse(c.sent[0]['stop'])

    def test_mapping_focus_loss_keeps_session_but_removes_motion_at_worker(self):
        from fairino_slam.chassis.control_core import check_input
        c = self.connection(navigation=False)
        c.latest['stamp'] = time.monotonic()
        c.send(keys='w', focused=False)
        now = time.monotonic()
        self.assertEqual(check_input(c.sent[0], now, now), [])
        self.assertFalse(c.sent[0]['stop'])

    def test_package_contains_no_keyboard_window(self):
        self.assertFalse(hasattr(transport, 'Window'))


if __name__ == '__main__':
    unittest.main()
