"""Lifecycle preparation/activation regressions without ROS or motor commands."""
from concurrent.futures import Future
import io
import json
from pathlib import Path
import queue
import sys
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from fairino_slam.navigation.lifecycle import LifecycleMonitor, NAMES
from fairino_slam.navigation import navigation_remote


class LifecycleTests(unittest.TestCase):
    def monitor(self, root, state=2):
        m = LifecycleMonitor.__new__(LifecycleMonitor)
        m.root, m.run_id = Path(root), 'boot'
        m.GetState = m.Manage = NS(Request=NS)
        m.clients = {n: Mock() for n in NAMES}
        m.manager = Mock()
        m.future = None
        m.activation, m.pending = {}, {}
        m.last_poll = 10.
        m.states = {n: (state, 10.) for n in NAMES}
        return m

    def request(self, m, **changes):
        request = {'id': 'request', 'run_id': 'boot', 'session': ['A', 1, 'epoch'],
                   'expires': time.time()+30}
        request.update(changes)
        (m.root/'activation-request.json').write_text(json.dumps(request))

    def test_prepared_is_not_active_and_stale_states_never_ready(self):
        with tempfile.TemporaryDirectory() as root:
            m = self.monitor(root)
            self.assertTrue(m.prepared(10))
            self.assertFalse(m.active(10))
            self.assertFalse(m.prepared(14))
            m.states = {n: (3, 10) for n in NAMES}
            self.assertTrue(m.active(10))
            self.assertFalse(m.active(14))

    def test_activation_uses_existing_manager_and_only_once(self):
        with tempfile.TemporaryDirectory() as root:
            m = self.monitor(root)
            future = Future()
            m.manager.call_async.return_value = future
            self.request(m)
            m.tick(10, True, ('A', 1, 'epoch'), False)
            self.assertEqual(m.manager.call_async.call_args.args[0].command, 2)
            self.assertEqual(m.activation['state'], 'pending')
            future.set_result(NS(success=True))
            m.tick(10.1, True, ('A', 1, 'epoch'), False)
            m.tick(10.2, True, ('A', 1, 'epoch'), False)
            m.manager.call_async.assert_called_once()
            self.assertEqual(m.activation['state'], 'completed')

    def test_invalid_request_or_health_never_activates(self):
        for changes, healthy, session, busy in [
            ({'run_id': 'old'}, True, ('A', 1, 'epoch'), False),
            ({'expires': 0}, True, ('A', 1, 'epoch'), False),
            ({}, False, ('A', 1, 'epoch'), False),
            ({}, True, ('B', 1, 'epoch'), False),
            ({}, True, ('A', 1, 'epoch'), True),
        ]:
            with self.subTest(changes=changes, healthy=healthy, session=session, busy=busy):
                with tempfile.TemporaryDirectory() as root:
                    m = self.monitor(root)
                    self.request(m, **changes)
                    m.tick(10, healthy, session, busy)
                    m.manager.call_async.assert_not_called()

    def test_lost_session_during_activation_stays_failed_after_success(self):
        with tempfile.TemporaryDirectory() as root:
            m = self.monitor(root)
            future = Future()
            m.manager.call_async.return_value = future
            self.request(m)
            m.tick(10, True, ('A', 1, 'epoch'), False)
            future.set_result(NS(success=True))
            m.tick(10.1, False, ('B', 1, 'epoch'), False)
            self.assertEqual(m.activation['state'], 'failed')

    def test_inflight_timeout_is_not_replayed(self):
        with tempfile.TemporaryDirectory() as root:
            m = self.monitor(root)
            future = Future()
            m.manager.call_async.return_value = future
            self.request(m)
            m.tick(10, True, ('A', 1, 'epoch'), False)
            m.activation['expires'] = 0
            m.tick(10.1, True, ('A', 1, 'epoch'), False)
            self.assertEqual(m.activation['state'], 'failed')
            m.manager.call_async.assert_called_once()
            future.set_result(NS(success=True))
            m.tick(10.2, True, ('A', 1, 'epoch'), False)
            self.assertEqual(m.activation['state'], 'failed')

    def test_cli_activation_never_starts_process_and_checks_run_id(self):
        with tempfile.TemporaryDirectory() as root:
            state = {'running': True, 'run_id': 'boot', 'localization_ready': True,
                     'session_token': ['A', 1, 'epoch'], 'prepared': True}
            def status():
                request = Path(root)/'activation-request.json'
                if request.exists():
                    data = json.loads(request.read_text())
                    return dict(state, controller_ready=True,
                                activation={'id': data['id'], 'state': 'completed'})
                return state
            with patch.object(navigation_remote, 'HERE', Path(root)), \
                 patch.object(navigation_remote, 'status', side_effect=status), \
                 patch.object(navigation_remote.subprocess, 'run') as run:
                self.assertIn('已激活', navigation_remote.activate()['message'])
                run.assert_not_called()

    def test_reader_preserves_error_before_final_disabled_status(self):
        from fairino_slam.chassis.transport import Connection, PREFIX
        c = Connection.__new__(Connection)
        c.latest, c.last_error = {}, ''
        c.last_status = 0.
        c.events, c.done, c.log = queue.Queue(), threading.Event(), io.StringIO()
        c.pexpect = NS(TIMEOUT=TimeoutError, EOF=EOFError)
        c.child = Mock()
        c.child.readline.side_effect = [
            PREFIX+json.dumps({'error': 'feedback timeout'})+'\n',
            PREFIX+json.dumps({'error': 'cleanup failure'})+'\n',
            PREFIX+json.dumps({'stamp': 1, 'disabled_confirmed': True})+'\n', '']
        c.reader()
        self.assertEqual(c.last_error, 'feedback timeout')
        self.assertTrue(c.latest['disabled_confirmed'])


if __name__ == '__main__':
    unittest.main()
