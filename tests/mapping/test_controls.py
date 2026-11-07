"""Mapping input and navigation ownership regressions without GTK or a robot."""
import threading
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock

from fairino_slam.chassis.control_core import DEFAULT_LINEAR, DEFAULT_ANGULAR
from fairino_slam.ui.mapping_controls import Window
from fairino_slam.ui.mapping_session import ChassisOwnership, MappingSession


class OwnershipTests(unittest.TestCase):
    def test_mapping_and_navigation_cannot_own_chassis_together(self):
        ownership = ChassisOwnership()
        self.assertTrue(ownership.acquire('base', '建图键控'))
        self.assertFalse(ownership.acquire('base', '导航'))
        ownership.release('base', '导航')
        self.assertEqual(ownership.owner, '建图键控')
        ownership.release('base', '建图键控')
        self.assertTrue(ownership.acquire('base', '导航'))
        self.assertFalse(ownership.acquire('base', '建图键控'))

    def test_recovery_reservation_waits_for_old_owner_and_blocks_new_sessions(self):
        ownership = ChassisOwnership()
        self.assertTrue(ownership.acquire('base', '建图键控'))
        self.assertTrue(ownership.reserve('恢复接入'))
        self.assertFalse(ownership.reserve('恢复接入'))
        self.assertFalse(ownership.acquire('base', '导航'))
        self.assertFalse(ownership.acquire('base', '恢复接入'))
        ownership.release('base', '建图键控')
        self.assertFalse(ownership.acquire('base', '建图键控'))
        self.assertTrue(ownership.acquire('base', '恢复接入'))
        ownership.release('base', '导航')
        self.assertEqual(ownership.owner, '恢复接入')
        ownership.release('base', '恢复接入')
        ownership.unreserve('wrong')
        self.assertFalse(ownership.acquire('base', '导航'))
        ownership.unreserve('恢复接入')
        self.assertTrue(ownership.acquire('base', '导航'))

    def test_recovery_timeout_does_not_release_original_owner(self):
        ownership = ChassisOwnership()
        ownership.acquire('base', '导航')
        ownership.reserve('恢复接入')
        ownership.unreserve('恢复接入')
        self.assertEqual(ownership.owner, '导航')
        self.assertFalse(ownership.acquire('base', '建图键控'))

    def test_external_owner_refusal_does_not_create_local_lease(self):
        ownership = ChassisOwnership(acquire=lambda *_: False)
        self.assertFalse(ownership.acquire('base', '建图键控'))
        self.assertIsNone(ownership.owner)


class SessionTests(unittest.TestCase):
    def session(self, factory):
        panel = Mock(closed=False)
        panel.stream.args = NS(host='robot', user='fairino')
        panel.stream.control_password = 'unused'
        return MappingSession(panel, ChassisOwnership(), factory)

    def drain(self, session, condition):
        deadline = time.monotonic()+2
        while not condition() and time.monotonic() < deadline:
            session.tick()
            time.sleep(.005)
        self.assertTrue(condition())

    def test_navigation_owner_blocks_mapping_before_ssh(self):
        factory = Mock()
        session = self.session(factory)
        session.ownership.acquire('base', '导航')
        session.start()
        factory.assert_not_called()
        self.assertFalse(session.busy)

    def test_close_during_connect_stops_late_connection_without_attaching_controls(self):
        release_connection = threading.Event()
        connection = Mock(done=threading.Event(), latest={}, last_error='')
        def finish():
            connection.latest = {'disabled_confirmed': True}
            connection.done.set()
        connection.finish.side_effect = finish
        def factory(args, password):
            self.assertFalse(args.navigation)
            self.assertEqual((args.linear, args.angular), (DEFAULT_LINEAR, DEFAULT_ANGULAR))
            release_connection.wait(1)
            return connection
        session = self.session(factory)
        session._attach = Mock()
        session.start()
        self.assertFalse(session.close())
        self.assertFalse(session.ownership.acquire('base', '导航'))
        release_connection.set()
        self.drain(session, lambda: not session.busy)
        session._attach.assert_not_called()
        connection.finish.assert_called_once()
        self.assertIsNone(session.ownership.owner)

    def test_finish_timeout_does_not_release_live_chassis(self):
        session = self.session(Mock())
        session.ownership.acquire('base', session.OWNER)
        session.leased = session.stopping = session.finishing = True
        connection = Mock(done=threading.Event(), latest={}, last_error='')
        session.connection = connection
        # Avoid a new retry thread while inspecting the timeout event.
        session.stop = Mock()
        session.events.put(('closed', None))
        session.tick()
        self.assertTrue(session.busy)
        self.assertFalse(session.ownership.acquire('base', '导航'))
        connection.done.set()
        connection.latest = {'disabled_confirmed': True}
        session.events.put(('closed', None))
        session.tick()
        self.assertFalse(session.busy)
        self.assertIsNone(session.ownership.owner)

    def test_retry_after_window_closed_finishes_transport_and_destroys_host_once(self):
        session = self.session(Mock())
        session.ownership.acquire('base', session.OWNER)
        session.leased = session.stopping = True
        connection = Mock(done=threading.Event(), latest={}, last_error='')
        session.connection = connection
        control, host = Mock(closing=True), Mock()
        session.control, session.host = control, host
        def finish():
            connection.latest = {'disabled_confirmed': True}
            connection.done.set()
        connection.finish.side_effect = finish
        session.stop()
        self.drain(session, lambda: not session.busy)
        control.close.assert_not_called()
        connection.finish.assert_called_once()
        host.destroy.assert_called_once()
        self.assertIsNone(session.control)
        self.assertIsNone(session.host)


class KeyboardPolicyTests(unittest.TestCase):
    def control(self):
        control = Window.__new__(Window)
        control.keys, control.held = {'w'}, {'w'}
        control.phase = Mock()
        control.input_suspended = False
        control.send = Mock()
        control.Gdk = NS(keyval_name=lambda value: value)
        control.Gtk = NS(Entry=type('Entry', (), {}))
        control.window = Mock()
        control.window.is_active.return_value = True
        control.window.get_focus.return_value = None
        control.ready = True
        control.stopped = control.speed_pending = False
        return control

    def test_focus_loss_stops_and_held_autorepeat_cannot_restart_motion(self):
        control = self.control()
        control.focus_out()
        self.assertEqual(control.keys, set())
        self.assertTrue(control.input_suspended)
        control.send.assert_called_once()
        control.focus_in()
        control.key_press(None, NS(keyval='w'))
        self.assertEqual(control.keys, set())
        control.key_release(None, NS(keyval='w'))
        control.key_press(None, NS(keyval='w'))
        self.assertEqual(control.keys, {'w'})

    def test_key_release_sends_stop_without_ending_session(self):
        control = self.control()
        control.key_release(None, NS(keyval='w'))
        self.assertEqual(control.keys, set())
        self.assertFalse(control.stopped)
        control.send.assert_called_once()

    def test_closed_window_timer_removes_itself_without_touching_widgets(self):
        control = self.control()
        control.closing = True
        self.assertFalse(control.tick())
        control.send.assert_not_called()


if __name__ == '__main__':
    unittest.main()
