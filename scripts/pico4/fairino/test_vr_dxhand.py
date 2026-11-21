"""Exercise the PICO control loop with synthetic feedback, input and a local hand."""
import contextlib
import io
import sys
from pathlib import Path
import queue
import tempfile
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np
import paths
from test_probe import connected_wrist
from test_tcp import hardware
from test_vr_demo import frame, hand
from vendor_kinematics import VendorKinematics
from vr_client import main, operate, wait_for_input
from vr_demo import DemoEngine
from vr_dxhand import DxHandTeleop, service_command
from vr_input import Clutch
from vr_cartesian import CartesianInput
from tcp_core import ARMS


class DxHandTests(unittest.TestCase):
    def test_public_help_describes_usb_and_calibration(self):
        with contextlib.redirect_stdout(io.StringIO()) as output, self.assertRaises(SystemExit) as exit_code:
            main(['--help'])
        self.assertEqual(exit_code.exception.code, 0)
        for option in ('--dxhand', '--dxhand-left', '--dxhand-right-calibration'):
            self.assertIn(option, output.getvalue())

    def test_cli_rejects_conflicts_before_any_transport(self):
        opts = ['--execute', '--dxhand']
        with patch('vr_client.Stream') as pico, patch('vr_client.Connection') as robot, \
                contextlib.redirect_stderr(io.StringIO()):
            for invalid in (['--grippers'], ['--tcp-tool', 'taccap'], ['--dxhand-right', '/dev/a', '--dxhand-left', '/dev/a']):
                with self.assertRaises(SystemExit):
                    main(opts + invalid)
        pico.assert_not_called()
        robot.assert_not_called()

    def test_service_command_for_selected_hand_and_calibration(self):
        args = NS(dxhand_right='auto', dxhand_left=None, tcp_tool='flange',
                  dxhand_left_calibration=None, dxhand_right_calibration='/tmp/Manus right.json')
        cmd = service_command(args)
        self.assertEqual(cmd[-4:], ['--right', 'auto', '--right-calibration', '/tmp/Manus right.json'])
        self.assertEqual(Path(cmd[0]), paths.ROOT / '.local/dxhand/env/bin/python')
        self.assertNotIn('--left', cmd)

    def test_broken_child_and_incomplete_status_cannot_enable_follow(self):
        args = NS(dxhand_right='auto', dxhand_left=None, tcp_tool='flange',
                  dxhand_left_calibration=None, dxhand_right_calibration=None)
        for command in ('pass', 'print(\'{"event":"status","ready":true,"hands":{}}\')'):
            with self.subTest(command=command):
                client = DxHandTeleop(args)
                client.argv = [sys.executable, '-c', command]
                client.start()
                self.addCleanup(client.finish)
                client.reader.join(timeout=3)
                with self.assertRaises(RuntimeError):
                    client.update(frame(1.), {}, True)
                self.assertFalse(client.finish())
                self.assertFalse(client.follow)

    def test_stopping_status_reports_the_real_reason_before_cleanup_finishes(self):
        args = NS(dxhand_right='auto', dxhand_left=None, tcp_tool='flange',
                  dxhand_left_calibration=None, dxhand_right_calibration=None)
        client = DxHandTeleop(args)
        client.state = dict(event='status', ready=False, stopping=True, reason='shutdown',
                            errors=['缺少完整的已回零实测起步姿态：right'], hands={})
        with self.assertRaises(RuntimeError) as raised:
            client.update(frame(1.), {}, True)
        self.assertIn('缺少完整的已回零实测起步姿态', str(raised.exception))
        self.assertNotIn('超时', str(raised.exception))
        self.assertFalse(client.follow)

    def test_lifecycle_only_opens_local_hand_for_execute_and_closes_before_robot_wait(self):
        for mode in ('--check', '--prepare-check', '--link-check', '--execute'):
            args = [mode, '--ik-backend', 'vendor', '--dxhand', '--dxhand-right', 'auto']
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory, contextlib.ExitStack() as stack:
                order = []
                robot = Mock(latest={'disabled_confirmed': True})
                robot.finish.side_effect = lambda: order.append('upper.finish')
                client = Mock()
                for action in ('start', 'close', 'finish'):
                    getattr(client, action).side_effect = lambda name=action: order.append(name) or True
                client.snapshot.return_value = dict(event='stopped', hands={
                    'right': dict(phase='closed', driver_off_sent=True, disable_confirmed=False)})
                stack.enter_context(patch('vr_client.ROOT', Path(directory)))
                stack.enter_context(patch('vr_client.Stream'))
                stack.enter_context(patch('vr_client.wait_for_input'))
                stack.enter_context(patch('vr_client.Connection', return_value=robot))
                stack.enter_context(patch('vr_client.operate', return_value=0))
                stack.enter_context(patch('vr_link_check.run_client', return_value=0))
                stack.enter_context(patch('vr_client.DxHandTeleop', return_value=client))
                stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
                self.assertEqual(main(args), 0)
                self.assertEqual(order, ['start', 'close', 'upper.finish', 'finish']
                                 if mode == '--execute' else ['upper.finish'])

    def test_dxhand_start_input_ignores_grip_and_trigger(self):
        signal = frame(1., left=hand(1.), right=hand(1.))
        for side in ARMS:
            signal[side].update(trigger=1., trigger_available=False)
        self.assertIs(wait_for_input(NS(read=lambda: signal), require_grips=False), signal)

    def test_resume_anchors_latest_pose_to_measured_tcp_with_moving_arms(self):
        hw = hardware()
        hw['command_position'][1] += .01
        hw['velocity'][1] = .1
        kin = VendorKinematics()
        inputs = CartesianInput({s: Clutch(s) for s in ARMS}, 0., automatic=True)
        sample = frame(2., left=hand(position=(.1, 0., 0.)))
        sample['samples'] = [frame(1.98), sample.copy()]
        goals = inputs.goals(sample, hw, kin, {s:s for s in ARMS}, 1., False, 2.)
        for side, joints in ARMS.items():
            actual, _ = kin.forward_tcp([hw['position'][j-1] for j in joints], side)
            np.testing.assert_allclose(goals[side], actual, atol=1e-8)

    def test_operate_gates_both_arms_and_fingers_through_hold_home_and_dropout(self):
        steps = ['not_ready', 'ready', 'follow', 'pause', 'held', 'resume', 'align', 'follow',
                 'drop', 'align', 'follow', 'home', 'home', 'quit']
        index = [0]
        def step():
            return steps[index[0]]
        def now():
            return 10. + .02*index[0]
        dx = Mock(ready=False)
        dx.snapshot.return_value = {'hands': {'left': {'phase': 'holding'}}}
        fingers, arm_packets = {}, {}
        dx.update.side_effect = lambda frame, hands, active: fingers.update({index[0]: active})
        def read():
            if step() == 'drop':
                raise ValueError('PICO interrupted')
            x = .02 if index[0] >= 3 else 0.
            return frame(now(), left=hand(0., (x,0.,0.)), right=hand(1.))
        with tempfile.TemporaryDirectory() as directory:
            conn = connected_wrist(NS(events=queue.Queue(), done=threading.Event(), last_status=now(),
                                      latest=dict(hardware=hardware(), stamp=now(), ready=True),
                                      log_path=Path(directory)/'loop.jsonl'))
            conn.send = lambda request, **_: arm_packets.update({index[0]: request})
            def sleep(_):
                index[0] += 1
                dx.ready = True
                conn.last_status = conn.latest['stamp'] = now()
            keys = {'pause':' ', 'resume':' ', 'home':'h', 'quit':'q'}
            args = NS(execute=True, sensitivity=.25, no_auto_home=True, dxhand_control=dx)
            with patch('vr_client.time.monotonic', now), patch('vr_client.time.sleep', sleep), \
                    patch('vr_client.key', lambda: keys.get(step(), '') if index[0] != 12 else ''), \
                    patch('vr_client.keyboard', contextlib.nullcontext), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(operate(args, VendorKinematics(), NS(read=read), conn), 0)
        for i, action in enumerate(steps[:-1]):
            with self.subTest(tick=i, step=action):
                self.assertEqual(fingers[i], action == 'follow')
                if action == 'home':
                    self.assertEqual(arm_packets[i]['source'], 'home')
                elif action == 'follow':
                    self.assertEqual(set(arm_packets[i]['arms']), set(ARMS))
                else:
                    self.assertIsNone(arm_packets[i])

    def test_model_automatically_reanchors_after_pause_and_drop(self):
        engine = DemoEngine(dxhand=True, ik_backend='vendor', interpolation_delay_ms=0.)
        def tick(t, x, command=None):
            return engine.step(frame(t, left=hand(position=(x,0,0))), t, command)
        tick(10., 0.)
        self.assertEqual(tick(10.01, 0.)['sides'], 'left/right')
        original = engine.q[:]
        tick(10.02, .004)
        self.assertGreater(np.max(np.abs(np.subtract(original, engine.q))), .0001)
        held = engine.q[:]
        tick(10.03, .004, 'pause')
        self.assertTrue(tick(10.04, .1)['paused'])
        tick(10.05, .15, 'pause')
        tick(10.06, .15)
        status = tick(10.07, .15)
        self.assertFalse(status['paused'], status['message'])
        np.testing.assert_allclose(engine.q, held, atol=1e-5)
        engine.step(None, 10.08)
        tick(10.09, -.12)
        status = tick(10.10, -.12)
        self.assertEqual(status['grippers'], {})
        self.assertEqual(status['sides'], 'left/right')
        np.testing.assert_allclose(engine.q, held, atol=1e-5)
