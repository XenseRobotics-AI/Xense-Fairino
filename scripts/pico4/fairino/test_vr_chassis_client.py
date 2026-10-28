"""Actual PICO loop + chassis adapter/gate; no threads, SSH, ROS or motors."""
import contextlib
import copy
import io
import json
from pathlib import Path
import queue
import tempfile
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import paths
from test_probe import connected_wrist
from test_tcp import hardware
from vendor_kinematics import VendorKinematics
from vr_chassis import ChassisTeleop
from vr_client import operate, wait_for_input
from vr_core import LIFT_MODE
from apps.fairino_keyboard.axis_session import AxisSession, ZERO
from apps.fairino_keyboard.control_core import check_axes, input_twist


class EndSimulation(Exception):
    pass


def frame(now, x=0., y=0., yaw=0., lift=0., left_click=False, right_click=False,
          left_available=True, right_available=True, **_):
    controller = dict(pose=[0., 0., 0., 0., 0., 0., 1.], grip=0., primary=False,
                      secondary=False, axis_x=0., axis_y=0., axis_click=False,
                      joystick_available=True)
    return dict(stamp=now, received_monotonic=now,
                left={**controller, 'axis_x': x, 'axis_y': y, 'axis_click': left_click,
                      'joystick_available': left_available},
                right={**controller, 'axis_x': yaw, 'axis_y': lift, 'axis_click': right_click,
                       'joystick_available': right_available})


class ChassisClientTests(unittest.TestCase):
    def simulate(self, steps, body_lift=False, mode='first_person', hand_mapping='auto'):
        """Advance the real AxisSession gate once per sent upper-body heartbeat."""
        clock, sent = [10.], []
        session = AxisSession('unused', 'unused', None)
        session.latest = dict(ready=True, hardware={'enabled': [True]*8}, command=ZERO)
        args = NS(execute=True, sensitivity=.25, no_auto_home=True, body_lift=body_lift,
                  chassis_linear=.06, chassis_angular=.10, user='unused',
                  mode=mode, hand_mapping=hand_mapping)
        chassis = ChassisTeleop(args, None, session_factory=lambda *_, **__: session)
        args.chassis_control = chassis
        hw = hardware()
        hw['enabled'] = [True]*20
        with tempfile.TemporaryDirectory() as directory:
            connection = connected_wrist(NS(events=queue.Queue(), done=threading.Event(), last_status=10.,
                latest=dict(hardware=hw, stamp=10., ready=steps[0].get('ready', True)),
                log_path=Path(directory)/'client.jsonl'))

            def send(**fields):
                axes = session.command_at(clock[0], session.latest['ready'])
                twist = input_twist(check_axes(axes), args.chassis_linear, args.chassis_angular)
                session.latest['command'] = twist
                sent.append(dict(request=copy.deepcopy(fields.get('request')), axes=axes,
                                 command=twist, armed=session.armed, active=session.active))
                if len(sent) == len(steps):
                    raise EndSimulation()

            def sleep(_):
                clock[0] += .02
                connection.last_status = connection.latest['stamp'] = clock[0]
                connection.latest['ready'] = steps[len(sent)].get('ready', True)

            def read():
                step = steps[len(sent)]
                if step.get('read_error'):
                    raise ValueError('双手柄数据超过 0.25 秒未更新')
                result = frame(clock[0], **step)
                if 'samples' in step:
                    result['samples'] = [frame(clock[0], **sample) for sample in step['samples']]
                return result

            connection.send = send
            with patch('vr_client.time.monotonic', lambda: clock[0]), \
                 patch('vr_client.time.sleep', sleep), \
                 patch('vr_client.key', lambda: steps[len(sent)].get('key', '')), \
                 patch('vr_client.keyboard', contextlib.nullcontext), \
                 contextlib.redirect_stdout(io.StringIO()), self.assertRaises(EndSimulation):
                operate(args, VendorKinematics(), NS(read=read), connection)
            events = [json.loads(line) for line in connection.log_path.with_suffix('.actions.jsonl').read_text().splitlines()]
        self.assertIsNone(session.thread.ident)  # The real gate ran without starting I/O.
        return sent, events

    def test_released_grips_allow_translation_yaw_and_mixed_yaw_plus_lift(self):
        sent, _ = self.simulate([{}, {'y': 1.}, {'x': 1.}, {'yaw': 1.},
                                 {'x': .6, 'y': .8, 'yaw': .7, 'lift': .6}], body_lift=True)
        self.assertEqual(sent[1]['command'], (.06, 0., 0.))
        self.assertEqual(sent[2]['command'], (0., -.06, 0.))
        self.assertEqual(sent[3]['command'], (0., 0., -.10))
        for row in sent[:4]:
            self.assertEqual(row['request']['arms'], {})  # No fabricated arm target.
            self.assertEqual(row['request']['lift'], 0.)  # Body heartbeat keeps its lease alive.
            self.assertEqual(row['request']['lift_mode'], LIFT_MODE)
            self.assertEqual(row['request']['lift_epoch'], 1)
        mixed = sent[4]
        for actual, expected in zip(mixed['command'], (.048, -.036, -.0625)):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(mixed['request']['arms'], {})
        self.assertEqual(mixed['request']['lift'], .6)

    def test_modes_use_fixed_physical_sticks_and_do_not_reverse_or_swap_body_lift(self):
        # The arms may exchange controllers, but base translation stays on the
        # physical left stick and yaw/lift stay on physical right X/Y.
        expected = {
            'first_person': ((.06, 0., 0.), (.048, -.036, -.0625)),
            'mirror_same_direction': ((-.06, 0., 0.), (-.048, .036, -.0625)),
            'mirror': ((.06, 0., 0.), (.048, .036, .0625)),
        }
        for mode, (forward, mixed) in expected.items():
            for hand_mapping in ('auto', 'same', 'crossed'):
                with self.subTest(mode=mode, hand_mapping=hand_mapping):
                    sent, _ = self.simulate(
                        [{}, {'y': 1.}, {'lift': 1.},
                         {'x': .6, 'y': .8, 'yaw': .7, 'lift': -.6}],
                        body_lift=True, mode=mode, hand_mapping=hand_mapping)
                    self.assertEqual(sent[1]['command'], forward)
                    self.assertEqual(sent[1]['request']['arms'], {})
                    self.assertEqual(sent[1]['request']['lift'], 0.)  # Left Y never triggers lift.
                    self.assertEqual(sent[2]['command'], ZERO)  # Right Y never translates the base.
                    self.assertEqual(sent[2]['request']['lift'], 1.)
                    for actual, wanted in zip(sent[3]['command'], mixed):
                        self.assertAlmostEqual(actual, wanted)
                    self.assertEqual(sent[3]['request']['arms'], {})
                    self.assertEqual(sent[3]['request']['lift'], -.6)

    def test_body_session_epoch_changes_after_pause_and_input_loss(self):
        sent, _ = self.simulate([{}, {'lift': .8}, {'lift': .8, 'key': ' '},
                                 {'lift': .8}, {}, {'lift': .8}, {'read_error': True},
                                 {'lift': .8}, {}, {'lift': .8}], body_lift=True)
        self.assertEqual(sent[1]['request']['lift'], .8)
        self.assertIsNone(sent[2]['request'])
        self.assertIsNone(sent[6]['request'])
        for indices, epoch in (((0, 1), 1), ((3, 4, 5), 2), ((7, 8, 9), 3)):
            for index in indices:
                self.assertEqual(sent[index]['request']['lift_epoch'], epoch)
                self.assertEqual(sent[index]['request']['lift_mode'], LIFT_MODE)
        # Reconnected deflection cannot resume a gesture until a real neutral sample.
        for index in (3, 4, 7, 8):
            self.assertEqual(sent[index]['request']['lift'], 0.)
        self.assertEqual(sent[5]['request']['lift'], .8)
        self.assertEqual(sent[9]['request']['lift'], .8)

    def test_batch_direction_changes_remain_visible_in_later_heartbeats(self):
        sent, _ = self.simulate([{}, {'lift': .8},
            {'lift': .8, 'samples': [{'lift': .8}, {}, {'lift': .8}]}, {'lift': .8}],
            body_lift=True)
        requests = [row['request'] for row in sent]
        self.assertEqual([r['lift'] for r in requests], [0., .8, .8, .8])
        self.assertEqual(requests[2]['lift_gesture'], requests[1]['lift_gesture']+2)
        self.assertEqual(requests[3]['lift_gesture'], requests[2]['lift_gesture'])
        self.assertEqual({r['lift_epoch'] for r in requests}, {1})

    def test_hidden_missing_sample_changes_body_session_after_recovery(self):
        sent, _ = self.simulate([{}, {'lift': .8},
            {'lift': .8, 'samples': [{'right_available': False}, {'lift': .8}]},
            {'lift': .8}, {}, {'lift': .8}], body_lift=True)
        self.assertIsNone(sent[2]['request'])
        self.assertEqual(sent[3]['request']['lift_epoch'], 2)
        self.assertEqual(sent[3]['request']['lift'], 0.)
        self.assertEqual(sent[5]['request']['lift'], .8)

    def test_space_pauses_base_until_a_new_neutral_sample(self):
        sent, events = self.simulate([{}, {'y': 1.}, {'y': 1.}, {'y': 1., 'key': ' '},
                                     {'y': 1.}, {}, {'y': 1.}])
        self.assertEqual(sent[2]['command'], (.06, 0., 0.))
        self.assertTrue(all(row['command'] == ZERO for row in sent[3:6]))
        self.assertFalse(sent[3]['active'])
        self.assertFalse(sent[4]['armed'])
        self.assertTrue(sent[5]['armed'])
        self.assertEqual(sent[6]['command'], (.06, 0., 0.))
        self.assertTrue(any(e.get('event') == 'input_paused' and '用户暂停' in e['reason'] for e in events))

    def test_pico_read_error_zeroes_motion_and_does_not_replay_on_reconnect(self):
        sent, events = self.simulate([{}, {'yaw': 1.}, {'read_error': True},
                                     {'yaw': 1.}, {}, {'yaw': 1.}])
        self.assertEqual(sent[1]['command'], (0., 0., -.10))
        self.assertTrue(all(row['command'] == ZERO for row in sent[2:5]))
        self.assertFalse(sent[2]['active'])
        self.assertFalse(sent[3]['armed'])
        self.assertEqual(sent[5]['command'], (0., 0., -.10))
        self.assertTrue(any(e.get('event') == 'input_paused' and '未更新' in e['reason'] for e in events))

    def test_real_home_suppresses_base_even_while_sticks_remain_deflected(self):
        sent, events = self.simulate([{}, {'y': 1.}, {'y': 1., 'yaw': .7, 'key': 'h'},
                                     {'y': 1., 'yaw': .7}, {'y': 1., 'yaw': .7}])
        self.assertEqual(sent[1]['command'], (.06, 0., 0.))
        self.assertTrue(any(e.get('event') == 'home_started' for e in events))
        for row in sent[2:]:
            self.assertEqual(row['command'], ZERO)
            self.assertFalse(row['active'])
            self.assertEqual(row['request']['source'], 'home')
            self.assertEqual(set(row['request']['arms']), {'left', 'right'})

    def test_upper_not_ready_holds_base_and_requires_recenter_after_recovery(self):
        sent, _ = self.simulate([{}, {'y': 1.}, {'y': 1., 'ready': False},
                                {'y': 1., 'ready': False}, {'y': 1.}, {}, {'y': 1.}])
        self.assertEqual(sent[1]['command'], (.06, 0., 0.))
        self.assertTrue(all(row['command'] == ZERO for row in sent[2:6]))
        self.assertTrue(all(not row['active'] for row in sent[2:4]))
        self.assertFalse(sent[4]['armed'])
        self.assertEqual(sent[6]['command'], (.06, 0., 0.))


class ChassisStartupInputTests(unittest.TestCase):
    def test_requires_available_unpressed_radially_neutral_sticks_before_return(self):
        values = [dict(y=1.), dict(yaw=1.), dict(left_available=False), dict(right_available=False),
                  dict(left_click=True), dict(right_click=True), dict(x=.15, y=.15), {}]
        clock, reads = [10.], []

        def read():
            result = frame(clock[0], **values[len(reads)])
            reads.append(result)
            return result

        with patch('vr_client.time.monotonic', lambda: clock[0]), \
             patch('vr_client.time.sleep', lambda seconds: clock.__setitem__(0, clock[0]+seconds)), \
             contextlib.redirect_stdout(io.StringIO()):
            result = wait_for_input(NS(read=read), require_chassis=True)
        self.assertEqual(len(reads), len(values))
        self.assertIs(result, reads[-1])

    def test_lift_option_adds_right_vertical_neutral_requirement(self):
        for require_lift, expected_reads in ((False, 1), (True, 2)):
            with self.subTest(require_lift=require_lift):
                clock, reads = [10.], []

                def read():
                    result = frame(clock[0], lift=.8 if not reads else 0.)
                    reads.append(result)
                    return result

                with patch('vr_client.time.monotonic', lambda: clock[0]), \
                     patch('vr_client.time.sleep', lambda seconds: clock.__setitem__(0, clock[0]+seconds)), \
                     contextlib.redirect_stdout(io.StringIO()):
                    wait_for_input(NS(read=read), require_chassis=True, require_lift=require_lift)
                self.assertEqual(len(reads), expected_reads)


if __name__ == '__main__':
    unittest.main()
