"""Offline regression for mixed desktop/robot navigation deployments."""
import contextlib
import io
import json
import math
import unittest
from unittest.mock import patch

from fairino_slam.navigation import client


class ClientCompatibilityTests(unittest.TestCase):
    def setUp(self):
        mode = patch.object(client, 'configured_heading_mode', return_value='hold_initial')
        self.mode = mode.start()
        self.addCleanup(mode.stop)

    def test_legacy_plan_and_goal_error_explains_required_deployment_without_retry(self):
        legacy_error = (
            "SSH failed: usage: navigation_remote.py [-h] [--x X] [--y Y] [--yaw YAW]\n"
            "navigation_remote.py: error: plan/goal requires finite "
            "--x --y --yaw (metres/radians)"
        )
        for action in ("plan", "goal"):
            with self.subTest(action=action):
                with patch.object(client, "run", side_effect=RuntimeError(legacy_error)) as run:
                    with self.assertRaises(RuntimeError) as raised:
                        client.execute(action, "offline-host", "fairino", "offline-password",
                                       {"x": 3.0, "y": 0.0, "session_id": "localization-1"})
                message = str(raised.exception)
                self.assertIn("机器人端导航程序", message)
                self.assertIn("旧版", message)
                self.assertIn("deploy navigation --execute", message)
                run.assert_called_once()
                command = run.call_args.args[1]
                self.assertIn("--x 3.0", command)
                self.assertIn("--y 0.0", command)
                self.assertNotIn("--yaw", command)
                self.assertIn("--heading-mode hold_initial", command)

    def test_stop_and_cancel_remain_available_without_version_probe(self):
        for action in ("stop", "cancel"):
            with self.subTest(action=action):
                expected = {"message": action + " accepted"}
                with patch.object(client, "run", return_value="NAVIGATION " + json.dumps(expected)) as run:
                    result = client.execute(action, "offline-host", "fairino", "offline-password")
                self.assertEqual(result, expected)
                run.assert_called_once()
                self.assertIn(client.REMOTE + " " + action, run.call_args.args[1])

    def test_unknown_heading_argument_requires_update_without_retry_or_fallback(self):
        for mode in ('hold_initial', 'path_tangent'):
            pose = {'x': 1., 'y': 2., 'heading_mode': mode}
            if mode == 'path_tangent':
                pose['yaw'] = math.pi/2
            error = 'navigation_remote.py: error: unrecognized arguments: --heading-mode '+mode
            with self.subTest(mode=mode), patch.object(
                    client, 'run', side_effect=RuntimeError(error)) as run:
                with self.assertRaisesRegex(RuntimeError, 'deploy navigation --execute'):
                    client.execute('goal', 'offline-host', 'fairino', 'offline-password', pose)
                run.assert_called_once()
                self.assertIn('--heading-mode '+mode, run.call_args.args[1])

    def test_path_requests_include_expected_mode_and_user_radians(self):
        self.mode.return_value = 'path_tangent'
        for action in ('plan', 'goal'):
            pose = {'x': 1., 'y': 2., 'yaw': math.pi/2, 'session_id': 'A'}
            original = dict(pose)
            result = {'heading_policy': 'path_tangent', 'goal_yaw': math.pi/2,
                      'locked_yaw': -.4, 'requested_yaw_ignored': False}
            with self.subTest(action=action), patch.object(
                    client, 'run', return_value='NAVIGATION '+json.dumps(result)) as run:
                self.assertEqual(client.execute(action, 'offline-host', 'fairino',
                                                'offline-password', pose), result)
                run.assert_called_once()
                command = run.call_args.args[1]
                self.assertIn('--heading-mode path_tangent', command)
                self.assertIn('--yaw '+str(math.pi/2), command)
                self.assertIn('--session-id A', command)
                self.assertEqual(pose, original)

    def test_invalid_path_yaw_is_rejected_before_ssh(self):
        self.mode.return_value = 'path_tangent'
        for yaw in (None, True, '1.0', float('nan'), float('inf'), -float('inf')):
            for action in ('plan', 'goal'):
                with self.subTest(action=action, yaw=yaw), patch.object(client, 'run') as run:
                    with self.assertRaisesRegex(ValueError, '有限.*--yaw'):
                        client.execute(action, 'offline-host', 'fairino', 'offline-password',
                                       {'x': 1., 'y': 2., 'yaw': yaw})
                    run.assert_not_called()

    def test_explicit_ui_mode_is_checked_remotely_without_overriding_from_local_config(self):
        self.mode.side_effect = AssertionError('captured UI mode must be kept')
        error = 'heading_mode mismatch: expected path_tangent, deployed hold_initial'
        with patch.object(client, 'run', return_value='NAVIGATION '+json.dumps({'error': error})) as run:
            with self.assertRaisesRegex(RuntimeError, 'heading_mode mismatch'):
                client.execute('goal', 'offline-host', 'fairino', 'offline-password',
                               {'x': 1., 'y': 2., 'yaw': 1., 'heading_mode': 'path_tangent'})
            run.assert_called_once()

    def test_stop_and_cancel_do_not_depend_on_heading_configuration(self):
        self.mode.side_effect = AssertionError('stop/cancel must not load heading config')
        for action in ('stop', 'cancel'):
            with patch.object(client, 'run', return_value='NAVIGATION {}') as run:
                self.assertEqual(client.execute(action, 'offline-host', 'fairino', 'offline-password'), {})
                run.assert_called_once()
                self.assertNotIn('--heading-mode', run.call_args.args[1])

    def test_authentication_failure_keeps_original_error(self):
        error = "SSH authentication rejected"
        with patch.object(client, "run", side_effect=RuntimeError(error)) as run:
            with self.assertRaisesRegex(RuntimeError, error):
                client.execute("plan", "offline-host", "fairino", "offline-password",
                               {"x": 3.0, "y": 0.0})
        run.assert_called_once()

    def test_current_navigation_results_are_parsed_without_changes(self):
        for action, data in (
            ("plan", {"points": [[0.0, 0.0], [3.0, 0.0]], "heading_policy": "hold_initial",
                      "locked_yaw": 0.75}),
            ("goal", {"goal_id": "aabb", "heading_policy": "hold_initial", "locked_yaw": 0.75}),
        ):
            with self.subTest(action=action):
                output = "diagnostic output\nNAVIGATION " + json.dumps(data) + "\n"
                with patch.object(client, "run", return_value=output) as run:
                    result = client.execute(action, "offline-host", "fairino", "offline-password",
                                            {"x": 3.0, "y": 0.0})
                self.assertEqual(result, data)
                run.assert_called_once()
                self.assertNotIn("--yaw", run.call_args.args[1])


class ClientHeadingCLITests(unittest.TestCase):
    def test_configured_strategy_reads_both_supported_deployment_modes(self):
        for mode in ('hold_initial', 'path_tangent'):
            config = {'nav2': {'controller_server': {'ros__parameters': {
                'FollowPath': {'heading_mode': mode}}}}}
            with patch.object(client, 'load_navigation', return_value=config):
                self.assertEqual(client.configured_heading_mode(), mode)

    def test_cli_expected_mode_and_radians_are_sent_after_explicit_goal(self):
        for mode, extra, expected in (
                ('hold_initial', [], {'x': 1., 'y': 2., 'heading_mode': 'hold_initial'}),
                ('path_tangent', ['--yaw', '1.25'],
                 {'x': 1., 'y': 2., 'yaw': 1.25, 'heading_mode': 'path_tangent'})):
            with self.subTest(mode=mode), patch.object(
                    client, 'configured_heading_mode', return_value=mode), patch.object(
                    client.sys, 'argv', ['navigation', 'goal', '--x', '1', '--y', '2']+extra), patch.dict(
                    client.os.environ, {'FAIRINO_SSH_PASSWORD': 'offline-password'}), patch.object(
                    client, 'execute', return_value={'goal_id': 'offline-goal'}) as execute, contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(client.main(), 0)
                execute.assert_called_once_with('goal', '192.168.58.3', 'fairino', 'offline-password', expected)

    def test_cli_path_requires_finite_yaw_before_password_or_ssh(self):
        for extra in ([], ['--yaw', 'nan'], ['--yaw', 'inf']):
            with self.subTest(extra=extra), patch.object(
                    client, 'configured_heading_mode', return_value='path_tangent'), patch.object(
                    client.sys, 'argv', ['navigation', 'goal', '--x', '1', '--y', '2']+extra), patch.object(
                    client.getpass, 'getpass') as getpass, patch.object(
                    client, 'execute') as execute, contextlib.redirect_stderr(io.StringIO()) as stderr:
                with self.assertRaises(SystemExit) as raised:
                    client.main()
                self.assertEqual(raised.exception.code, 2)
                self.assertIn('有限', stderr.getvalue())
                getpass.assert_not_called()
                execute.assert_not_called()


if __name__ == "__main__":
    unittest.main()
