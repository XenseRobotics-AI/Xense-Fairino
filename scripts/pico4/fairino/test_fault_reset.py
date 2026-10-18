"""Offline regression checks: no ROS, SSH or robot commands are used."""
import contextlib
import copy
import io
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import clear_errors as client
import fault_reset_remote as worker


def message(word=0x2240, error=0):
    return SimpleNamespace(joint_names=[f"j{i}" for i in range(1, 9)],
                           stateword=[word] * 8, errorcode=[error] * 8,
                           position=[0.] * 8, velocity=[0.] * 8, command_velocity=[0.] * 8,
                           hexerrorcode_curerrcode=0x0E10FF01 if error else 0)


class Guards(unittest.TestCase):
    def test_fault_reaction_is_not_operation_enabled(self):
        for word in (0x220F, 0x130F):
            self.assertEqual(worker.drive_state(word), "fault_reaction")
            s = worker.snapshot(message(word, 30), 8)
            worker.require_idle(s)
            self.assertEqual(s["enabled_axes"], [])
            self.assertFalse(worker.is_clear(s))

    def test_enabled_and_quick_stop_rejected(self):
        for word in (0x2237, 0x1337, 0x2207, 0x0000):
            with self.assertRaises(RuntimeError):
                worker.require_idle(worker.snapshot(message(word), 8))

    def test_enabled_is_resettable_but_is_not_a_finished_reset(self):
        state = worker.snapshot(message(0x2237), 8)
        worker.require_resettable(state)
        self.assertTrue(worker.faults_clear(state))
        self.assertFalse(worker.is_clear(state))
        for word in (0x2207, 0x0000, 0xffff):
            with self.assertRaises(RuntimeError):
                worker.require_resettable(worker.snapshot(message(word), 8))
        state['error_hex'] = '0x0F100001'
        with self.assertRaisesRegex(RuntimeError, 'FATAL'):
            worker.require_resettable(state)

    def test_stationary_failure_identifies_axis_and_keeps_existing_threshold(self):
        msg = message()
        msg.velocity[7] = -.012658
        with self.assertRaisesRegex(RuntimeError, r'J8=-0.012658.*0.005'):
            worker.require_resettable(worker.snapshot(msg, 8))

    def test_invalid_feedback_rejected(self):
        for key, value in (("joint_names", ["j1"] * 8), ("stateword", [0]),
                           ("position", [float("nan")] * 8), ("velocity", [float("inf")] * 8)):
            m = message()
            setattr(m, key, value)
            with self.assertRaises(RuntimeError):
                worker.snapshot(m, 8)
        m = message()
        m.velocity[0] = .1
        with self.assertRaises(RuntimeError):
            worker.require_idle(worker.snapshot(m, 8))

    def test_estop_and_relay_fail_closed(self):
        worker.require_interlock("upper", gpio={"56": "0", "57": "0"})
        worker.require_interlock("base", upper_error=0, upper_age=.1)
        for kwargs in ({"part": "upper", "gpio": None},
                       {"part": "upper", "gpio": {"56": "1", "57": "0"}},
                       {"part": "base", "upper_error": 30, "upper_age": .1},
                       {"part": "base", "upper_error": 0, "upper_age": .5}):
            with self.assertRaises(RuntimeError):
                worker.require_interlock(**kwargs)

    def test_nonzero_command_velocity_blocks_even_when_stationary(self):
        m = message()
        m.command_velocity[0] = .1
        with self.assertRaises(RuntimeError):
            worker.require_stationary(worker.snapshot(m, 8))


class SingleShot(unittest.TestCase):
    def simulate(self, *, execute=True, blocked=False, result_code="0xC000",
                 stays_faulted=False, accepted=True, stale=False, action_timeout=False,
                 initially_clear=False, enabled=False, part='upper', disable_code='0xC000',
                 stays_enabled=False, late_fault=False, delayed_clear=0):
        session = worker.ResetSession.__new__(worker.ResetSession)
        session.part, session.execute, session.node, session.upper_time = part, execute, None, 1.
        session.report = {"ok": False, "send_attempted": False}
        clock = [0.]
        session.received, session.read_error = 0., None
        def spin_once(*args, **kwargs):
            clock[0] += .02
            session.received = clock[0]
        session.ros = SimpleNamespace(spin_once=spin_once)
        def future(value):
            available = clock[0] + .04
            return SimpleNamespace(done=lambda: clock[0] >= available, result=lambda: value)
        bad = worker.snapshot(message(0x2208, 30), 8)
        good = worker.snapshot(message(0x2221), 8)
        partial = message(0x2208, 30)
        partial.stateword[1] = partial.stateword[7] = 0x2237
        partial.errorcode[1] = partial.errorcode[7] = 0
        partial = worker.snapshot(partial, 8)
        enabled_good = copy.deepcopy(partial)
        enabled_good.update(error_hex='0x00000000', joint_errors={}, errorcode=[0]*8)
        session.state = (enabled_good if enabled else good) if initially_clear else (partial if enabled else bad)
        commands, order, reset_returned = [], [], [False]
        clear_reads = [0]

        def fresh(timeout=5.):
            clock[0] += .1
            session.received = clock[0]
            if reset_returned[0] and not session.report.get('disable_attempted'):
                if stale:
                    raise RuntimeError("反馈中断")
                clear_reads[0] += 1
                if not stays_faulted and clear_reads[0] > delayed_clear:
                    session.state = enabled_good if enabled else good
            if late_fault and session.report.get('phase') == 'verify_final':
                session.state = bad
            session.report['latest'] = session.state
            return session.state

        session.fresh = fresh
        session.controllers = Mock(return_value=[worker.BASE_CONTROLLER] if part == 'base' else [])
        session.interlock = Mock(side_effect=RuntimeError('急停') if blocked else None)
        session.pause_odometry = Mock(side_effect=lambda: order.append('pause_odom'))
        session.deactivate_base_controller = Mock(side_effect=lambda: order.append('stop_controller'))
        action = Mock()
        action.wait_for_server.return_value = True

        def send(goal):
            commands.append(goal.command)
            order.append(goal.command)
            def result():
                if goal.command == 'reseterror()':
                    if action_timeout:
                        raise RuntimeError('超时')
                    reset_returned[0] = True
                    code = result_code
                else:
                    # Detect a real regression: disable with faults would fail
                    # in the installed vendor controller.
                    self.assertTrue(worker.faults_clear(session.state))
                    code = disable_code
                    if code == '0xC000' and not stays_enabled:
                        session.state = good
                return future(SimpleNamespace(status=4, result=SimpleNamespace(error_code=code)))
            return future(SimpleNamespace(accepted=accepted, get_result_async=result))

        action.send_goal_async.side_effect = send
        modules = {
            "rclpy.action": SimpleNamespace(ActionClient=lambda *a: action),
            "fairino_hardware_command_msgs.action": SimpleNamespace(StringCommand=SimpleNamespace(Goal=SimpleNamespace)),
        }
        error = None
        with patch.dict(sys.modules, modules), patch.object(worker.time, "monotonic", lambda: clock[0]), contextlib.redirect_stdout(io.StringIO()):
            try:
                session.run()
            except RuntimeError as exc:
                error = str(exc)
        session.command_history, session.order = commands, order
        return session, action, error

    def test_check_cannot_send(self):
        s, a, e = self.simulate(execute=False, enabled=True)
        a.send_goal_async.assert_not_called()
        self.assertTrue(s.report["ok"])
        self.assertTrue(s.report['ready_to_reset'])
        self.assertFalse(s.report['cleared_and_disabled'])

    def test_interlock_blocks_send(self):
        s, a, e = self.simulate(blocked=True)
        a.send_goal_async.assert_not_called()
        self.assertIsNotNone(e)

    def test_already_clear_skips_command(self):
        s, a, e = self.simulate(initially_clear=True)
        a.send_goal_async.assert_not_called()
        self.assertTrue(s.report["ok"])

    def test_single_reset_and_feedback_verification(self):
        s, a, e = self.simulate()
        a.send_goal_async.assert_called_once()
        self.assertEqual(a.send_goal_async.call_args.args[0].command, "reseterror()")
        self.assertIsNone(e)
        self.assertTrue(s.report["cleared_and_disabled"])
        self.assertTrue(s.report["ok"])

    def test_faulted_and_enabled_axes_reset_before_disable_on_both_boards(self):
        for part in ('base', 'upper'):
            with self.subTest(part=part):
                s, _, error = self.simulate(part=part, enabled=True, delayed_clear=3)
                self.assertIsNone(error)
                self.assertEqual(s.command_history, ['reseterror()', 'disable()'])
                if part == 'base':
                    self.assertEqual(s.order, ['pause_odom', 'stop_controller', 'reseterror()', 'disable()'])
                self.assertEqual(s.report['before']['enabled_axes'], ['j2', 'j8'])
                self.assertEqual(s.report['after_reset']['enabled_axes'], ['j2', 'j8'])
                self.assertEqual(s.report['after']['enabled_axes'], [])
                self.assertTrue(s.report['cleared_and_disabled'])

    def test_clear_but_enabled_only_disables(self):
        s, _, error = self.simulate(initially_clear=True, enabled=True)
        self.assertIsNone(error)
        self.assertEqual(s.command_history, ['disable()'])
        self.assertFalse(s.report['send_attempted'])
        self.assertTrue(s.report['cleared_and_disabled'])

    def test_disable_failure_or_bad_feedback_is_not_a_success_or_another_reset(self):
        for kwargs in ({'disable_code': '0xC012'}, {'stays_enabled': True}, {'late_fault': True}):
            with self.subTest(kwargs=kwargs):
                s, _, error = self.simulate(enabled=True, **kwargs)
                self.assertIsNotNone(error)
                self.assertEqual(s.command_history, ['reseterror()', 'disable()'])
                self.assertFalse(s.report['ok'])
                self.assertNotIn('未发送复位', error)

    def test_rejected_timeout_stale_or_uncleared_never_retries(self):
        for kwargs in ({"accepted": False}, {"action_timeout": True}, {"stale": True},
                       {"stays_faulted": True}, {"result_code": "0xC012"}):
            with self.subTest(kwargs=kwargs):
                s, a, e = self.simulate(enabled=True, **kwargs)
                a.send_goal_async.assert_called_once()
                self.assertIsNotNone(e)
                self.assertFalse(s.report["ok"])


class BasePreparation(unittest.TestCase):
    def session(self, *, enabled=True, active=True):
        s = worker.ResetSession.__new__(worker.ResetSession)
        s.part, s.execute, s.report, s.upper_time = "base", True, {}, 1.
        s.state = worker.snapshot(message(0x2237 if enabled else 0x2208, 5), 8)
        clock = [0.]
        def fresh(*args, **kwargs):
            clock[0] += .1
            return s.state
        s.fresh = fresh
        s.pause_odometry = Mock()
        s.controllers = Mock(return_value=[worker.BASE_CONTROLLER] if active else [])
        s.interlock = Mock()
        s.deactivate_base_controller = Mock()
        s.disable_all = Mock()
        return s, clock

    def test_stops_factory_controller_without_disabling_before_reset(self):
        s, clock = self.session()
        calls = Mock()
        calls.attach_mock(s.deactivate_base_controller, 'stop')
        calls.attach_mock(s.disable_all, 'disable')
        with patch.object(worker.time, 'monotonic', lambda: clock[0]):
            s.prepare_base()
        self.assertEqual([c[0] for c in calls.mock_calls], ['stop'])
        self.assertEqual(len(s.report['prepared']['enabled_axes']), 8)

    def test_readonly_or_upper_cannot_run_preparation(self):
        for part, execute in [('base', False), ('upper', True)]:
            s, _ = self.session()
            s.part, s.execute = part, execute
            with self.assertRaises(RuntimeError):
                s.prepare_base()
            s.controllers.assert_not_called()

    def test_already_disabled_skips_disable(self):
        s, clock = self.session(enabled=False)
        with patch.object(worker.time, 'monotonic', lambda: clock[0]):
            s.prepare_base()
        s.deactivate_base_controller.assert_called_once()
        s.disable_all.assert_not_called()

    def test_failed_stop_or_pause_does_not_prepare(self):
        for method in ['deactivate_base_controller', 'pause_odometry']:
            with self.subTest(method=method):
                s, clock = self.session()
                getattr(s, method).side_effect = RuntimeError('not confirmed')
                with patch.object(worker.time, 'monotonic', lambda: clock[0]):
                    with self.assertRaises(RuntimeError):
                        s.prepare_base()
                self.assertNotIn('prepared', s.report)
                s.disable_all.assert_not_called()

    def test_other_controller_or_motion_blocks_before_changes(self):
        for cause in ['controller', 'motion', 'interlock']:
            with self.subTest(cause=cause):
                s, clock = self.session()
                if cause == 'controller':
                    s.controllers.side_effect = RuntimeError('unknown controller')
                elif cause == 'motion':
                    s.state['max_abs_velocity_rad_s'] = .1
                else:
                    s.interlock.side_effect = RuntimeError('estop')
                with patch.object(worker.time, 'monotonic', lambda: clock[0]):
                    with self.assertRaises(RuntimeError):
                        s.prepare_base()
                s.deactivate_base_controller.assert_not_called()
                s.disable_all.assert_not_called()

    def test_fault_recurrence_blocks_disable_without_sending_another_reset(self):
        s, _ = self.session()
        s.node = Mock()
        action = Mock()
        action.wait_for_server.return_value = True
        s.wait = lambda future, *args, **kwargs: future
        modules = {
            'rclpy.action': SimpleNamespace(ActionClient=lambda *args: action),
            'fairino_hardware_command_msgs.action': SimpleNamespace(StringCommand=SimpleNamespace(Goal=SimpleNamespace)),
        }
        with patch.dict(sys.modules, modules):
            with self.assertRaisesRegex(RuntimeError, '故障未归零或已复发'):
                worker.ResetSession.disable_all(s)
        action.send_goal_async.assert_not_called()
        self.assertFalse(s.report.get('send_attempted', False))

    def test_only_factory_base_controller_is_allowed_for_stopping(self):
        for part, allow, motion, accepted in [
            ('base', True, worker.BASE_CONTROLLER, True),
            ('base', False, worker.BASE_CONTROLLER, False),
            ('upper', True, worker.BASE_CONTROLLER, False),
            ('base', True, 'other_motion_controller', False),
        ]:
            with self.subTest(part=part, allow=allow, motion=motion):
                s, _ = self.session()
                s.part, s.node = part, Mock()
                client_mock = s.node.create_client.return_value
                client_mock.wait_for_service.return_value = True
                client_mock.call_async.return_value = SimpleNamespace(controller=[
                    SimpleNamespace(name='fairino_hardware_command_controller', state='active'),
                    SimpleNamespace(name=motion, state='active'),
                ])
                s.wait = lambda future, *args, **kwargs: future
                modules = {'controller_manager_msgs.srv': SimpleNamespace(
                    ListControllers=SimpleNamespace(Request=SimpleNamespace))}
                with patch.dict(sys.modules, modules):
                    if accepted:
                        active = worker.ResetSession.controllers(s, allow_base_stop=allow)
                        self.assertIn(motion, active)
                    else:
                        with self.assertRaises(RuntimeError):
                            worker.ResetSession.controllers(s, allow_base_stop=allow)


class Client(unittest.TestCase):
    def test_shared_report_distinguishes_reset_sent_from_final_success(self):
        state = worker.snapshot(message(0x2237), 8)
        report = dict(phase='disable', send_attempted=True, command_result='0xC000',
                      disable_attempted=True, disable_result='0xC012',
                      after=state, ok=False, failure='下使能失败')
        text = '\n'.join(client.report_lines(report, True))
        self.assertIn('阶段：下使能', text)
        self.assertIn('复位请求已发送', text)
        self.assertIn('下使能请求已发送', text)
        self.assertIn('未完成', text)
        self.assertNotIn('复位请求未发送', text)
        self.assertNotIn('完成：已复查', text)

    def test_preflight_failure_does_not_claim_reset_was_attempted(self):
        text = '\n'.join(client.report_lines(dict(phase='preflight', ok=False,
                         send_attempted=False, failure='J8 速度反馈超出阈值'), True))
        self.assertIn('复位请求未发送', text)
        self.assertIn('J8', text)

    def test_guard_discovery_keeps_feedback_callbacks_running(self):
        session = worker.ResetSession.__new__(worker.ResetSession)
        session.report = {}
        session.node = Mock()
        session.node.create_client.return_value.service_is_ready.return_value = False
        clock = [0.]
        session.ros = SimpleNamespace(spin_once=lambda *a, **kw: clock.__setitem__(0, clock[0]+.02))
        with patch.dict(sys.modules, {'std_srvs.srv': SimpleNamespace(Trigger=SimpleNamespace(Request=SimpleNamespace))}), patch.object(worker.time, 'monotonic', lambda: clock[0]):
            session.pause_odometry()
        self.assertGreaterEqual(clock[0], 1.)
        self.assertIn('odometry_warning', session.report)
        session.node.destroy_client.assert_called_once()

    def test_all_stops_if_upper_reset_fails(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(client, "ROOT", Path(folder)), contextlib.redirect_stdout(io.StringIO()):
            with patch.object(client, "run_part", return_value={"ok": False, "failure": "未清除"}) as run:
                self.assertEqual(client.main(["--execute"]), 1)
                run.assert_called_once_with("upper", True, "fairino")

    def test_defaults_are_read_only_and_inspect_both(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(client, "ROOT", Path(folder)), contextlib.redirect_stdout(io.StringIO()):
            with patch.object(client, "run_part", side_effect=lambda *a: {"ok": True}) as run:
                self.assertEqual(client.main([]), 0)
                self.assertEqual([c.args for c in run.call_args_list],
                                 [("upper", False, "fairino"), ("base", False, "fairino")])

    def test_missing_or_duplicate_results_are_not_success(self):
        for out in ("", worker.PREFIX + "{}\n" + worker.PREFIX + "{}"):
            with self.assertRaises(RuntimeError):
                client.parse_report(out)


if __name__ == "__main__":
    unittest.main()
