"""Diagnostic timing must not weaken stale-input rejection."""
import threading
import unittest
from unittest.mock import patch
from control_core import check_input, LINK_TIMEOUT
from robot_worker import Worker


class LinkTimingTests(unittest.TestCase):
    def test_stale_ack_still_rejected_and_reports_measured_age(self):
        packet = {'ack': 9.5, 'keys': [], 'focused': False}
        with self.assertRaisesRegex(RuntimeError, 'ACK 500 ms.*450 ms'):
            check_input(packet, 9.99, 10.)
        packet['ack'] = 9.6
        self.assertEqual(check_input(packet, 9.99, 10.), [])
        self.assertEqual(LINK_TIMEOUT, .45)

    def test_worker_reports_separate_input_ack_and_client_timing(self):
        worker = Worker.__new__(Worker)
        worker.input_lock = threading.Lock()
        worker.received = 9.98
        worker.packet = {'ack': 9.7, 'client_status_age': .1, 'client_send_gap': .05}
        with patch('robot_worker.os.getpriority', return_value=0):
            timing = worker.link_status(10.)
        self.assertAlmostEqual(timing['input_age'], .02)
        self.assertAlmostEqual(timing['ack_age'], .3)
        self.assertEqual(timing['client_send_gap'], .05)
        self.assertEqual(timing['process_nice'], 0)


if __name__ == '__main__':
    unittest.main()
