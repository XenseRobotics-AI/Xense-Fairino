"""Vertical FK invariants, body limits and actual VR receiver stopping; no robot."""
import ast
import base64
import contextlib
import copy
import io
import json
import math
from pathlib import Path
import queue
import shlex
import tempfile
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import zlib

import paths
import robot_probe as rp
from body_core import load_config
from tcp_core import MODEL_ID
from test_tcp import hardware, probe, STUBS
from vendor_kinematics import VendorKinematics
from vr_core import KIND, validate
from vr_client import operate, remote_command
from vr_robot import VRProbe
from vr_lift import LiftInput, LiftPlanner


class EndSimulation(Exception):
    pass


class LiftMathTests(unittest.TestCase):
    def setUp(self):
        self.model = load_config()
        self.planner = LiftPlanner(self.model.config)
        self.start = self.model.pose(dict(height=0, forward=12, roll=0, yaw=20), [0.]*20)

    def test_full_up_down_keeps_horizontal_position_attitude_and_other_axes(self):
        q = self.start[:]
        for direction in (1., -1.):
            self.planner.reset()
            for i in range(2500):
                previous = q[:]
                q = self.planner.sample(direction, q, 10.+i*.02)
                state = self.model.state(q)
                self.assertGreaterEqual(state['height'], -1e-7)
                self.assertLessEqual(state['height'], 100.+1e-7)
                self.assertAlmostEqual(state['forward_offset_mm'], 0., delta=1e-7)
                self.assertAlmostEqual(state['forward'], 12., places=8)
                self.assertEqual(q[:17], self.start[:17])
                self.assertLessEqual(max(abs(math.degrees(a-b)) for a,b in zip(q,previous))/.02,
                                     self.model.config['default_speed_deg_s']+1e-6)
            self.assertAlmostEqual(self.model.state(q)['height'], 100. if direction > 0 else 0., delta=1e-6)

    def test_first_sample_and_neutral_never_jump_or_replay_old_height(self):
        q = self.start[:]
        self.assertEqual(self.planner.sample(1., q, 10.), q)
        q = self.planner.sample(1., q, 10.02)
        for t in (10.04, 20., 50.):
            self.assertEqual(self.planner.sample(0., q, t), q)
        self.assertEqual(self.planner.sample(1., q, 60.), q)
        step = self.planner.sample(1., q, 65.)
        self.assertLessEqual(self.model.state(step)['height_mm']-self.model.state(q)['height_mm'], .75+1e-7)

    def test_vertical_path_starts_at_current_horizontal_offset_without_alignment(self):
        q = self.model.pose(dict(height=25, forward=12, roll=0, yaw=20), [0.]*20)
        q[18] += math.radians(.4)
        q[17] -= math.radians(.4)
        initial = self.model.state(q)
        self.planner.sample(1., q, 10.)
        for i in range(100):
            q = self.planner.sample(1., q, 10.+(i+1)*.02)
        state = self.model.state(q)
        self.assertAlmostEqual(state['forward_offset_mm'], initial['forward_offset_mm'], places=7)
        self.assertAlmostEqual(state['forward'], initial['forward'], places=8)
        self.assertGreater(state['height_mm'], initial['height_mm'])

    def test_tilt_gate_bad_axes_and_missing_calibration_block_motion(self):
        q = self.model.pose(dict(height=90, forward=0, roll=10, yaw=0), [0.]*20)
        self.planner.sample(-1., q, 10.)
        with self.assertRaisesRegex(ValueError, '侧倾'):
            self.planner.sample(-1., q, 10.02)
        for axis in (True, float('nan'), 1.01, 'up'):
            with self.assertRaises(ValueError):
                self.planner.sample(axis, self.start, 10.)
        with self.assertRaisesRegex(ValueError, '参考未标定'):
            LiftPlanner({**self.model.config, 'upright_confirmed': False})

    def test_receiver_only_accepts_scalar_lift_not_arbitrary_body_joints(self):
        request = dict(kind=KIND, model=MODEL_ID, id=1, created=10., arms={}, lift=.5)
        self.assertEqual(validate(hardware(), request, 10.), {})
        for mutation in ({'lift': [0.]*20}, {'lift': True}, {'lift': 0.}, {'created': 9.7},
                         {'arms': {'body': {}}}, {'lift': float('nan')}):
            with self.assertRaises(ValueError):
                validate(hardware(), {**request, **mutation}, 10.)


class LiftInputTests(unittest.TestCase):
    def frame(self, value=0., click=False, available=True):
        return {'right': dict(axis_y=value, axis_click=click, joystick_available=available)}

    def test_neutral_required_after_start_pause_click_or_missing_data(self):
        gate = LiftInput()
        self.assertEqual(gate.value(self.frame(1), True), 0)
        self.assertEqual(gate.value(self.frame(), True), 0)
        self.assertEqual(gate.value(self.frame(.7), True), .7)
        for frame, active in ((self.frame(), False), (None, True),
                              (self.frame(click=True), True), (self.frame(available=False), True)):
            self.assertEqual(gate.value(frame, active), 0)
            self.assertEqual(gate.value(self.frame(1), True), 0)
            gate.value(self.frame(), True)
            self.assertEqual(gate.value(self.frame(-.7), True), -.7)


class ReceiverTests(unittest.TestCase):
    def simulate(self, stale=False, enabled=True):
        p = probe(); p.__class__ = VRProbe
        model = load_config()
        q = model.pose(dict(height=25, forward=0, roll=0, yaw=0), p.hw['position'])
        p.hw['position'] = q[:]; p.hw['command_position'] = q[:]
        p.hold_positions = q[:]; p.baseline = q[:]
        p.lift = LiftPlanner(model.config) if enabled else None
        p.lift_axis_active = False; p.lift_stop_deadline = None
        clock, ticks, published = [10.], [0], []
        p.node = NS(create_publisher=lambda *a: NS(get_subscription_count=lambda: 1, publish=published.append))
        def prepare():
            p.hw_time = p.received = clock[0]; p.packet['ack'] = clock[0]
            rp.Probe.enable_all(p); p.ready = True
        p.prepare = prepare
        def pump(seconds=.02, guarded=True):
            clock[0] += seconds; ticks[0] += 1
            if getattr(p, 'path', None):
                end = p.path[-1][1][:]
                p.hw['velocity'] = [(b-a)/seconds for a,b in zip(p.hw['position'], end)]
                p.hw['position'] = end[:]; p.hw['command_position'] = end[:]
            p.hw_time = p.received = clock[0]; p.packet['ack'] = clock[0]
            if ticks[0] > 120:
                raise EndSimulation()
            if ticks[0] < 65:
                p.packet['request'] = dict(kind=KIND, model=MODEL_ID, id=ticks[0], created=clock[0],
                                          arms={}, lift=1., _wrist_guard=p.wrist.active)
            elif not stale:
                p.packet['request'] = None
            if guarded:
                p.guard()
        p.pump = pump
        stubs = {**STUBS, 'JointTrajectory': lambda: NS(joint_names=[], points=[], header=NS(stamp=None))}
        with patch.dict(rp.__dict__, stubs), patch('vr_robot.time.monotonic', lambda:clock[0]):
            with self.assertRaises(EndSimulation):
                p.run()
        return p, model, published

    def test_release_or_stale_input_holds_the_new_height_without_returning_to_start(self):
        for stale in (False, True):
            p, model, messages = self.simulate(stale)
            height = model.state(p.hw['position'])['height']
            self.assertGreater(height, 25.05)
            self.assertIsNone(p.path)
            self.assertEqual(p.hw['velocity'], [0.]*20)
            self.assertTrue(all(p.hw['enabled']))
            if stale:
                self.assertTrue(any('jog_rejected' in e for e in p.events))
            else:
                self.assertFalse(any('jog_rejected' in e for e in p.events))
            for message in messages:
                self.assertEqual(message.points[-1].positions[:17], p.baseline[:17])
                self.assertEqual(len(message.points[-1].positions), 20)

    def test_body_motion_not_enabled_by_an_ordinary_arm_session(self):
        p, model, messages = self.simulate(enabled=False)
        self.assertAlmostEqual(model.state(p.hw['position'])['height'], 25.)
        self.assertTrue(any('未启用身体升降' in e.get('jog_rejected', '') for e in p.events))


class ClientTests(unittest.TestCase):
    def test_right_stick_works_with_both_grips_released_and_home_does_not_lift(self):
        hw, sent, clock = hardware(), [], [10.]
        with tempfile.TemporaryDirectory() as directory:
            connection = NS(events=queue.Queue(), done=threading.Event(), last_status=10.,
                latest={'hardware':hw, 'stamp':10., 'ready':True}, log_path=Path(directory)/'client.jsonl')
            from test_probe import connected_wrist
            connected_wrist(connection)
            def send(**fields):
                sent.append(copy.deepcopy(fields.get('request')))
                if len(sent) >= 60:
                    raise EndSimulation()
            connection.send = send
            def sleep(_):
                clock[0] += .02
                connection.last_status = connection.latest['stamp'] = clock[0]
            def read():
                controller = dict(pose=[0.,0.,0.,0.,0.,0.,1.], grip=0., primary=False, secondary=False,
                                  axis_y=0., axis_click=False, joystick_available=True)
                n = len(sent)
                return dict(stamp=n+1, left=controller.copy(), right={**controller,
                    'axis_y': .8 if 10 <= n < 25 or 32 <= n < 45 else 0., 'axis_click': n == 30})
            args = NS(execute=True, sensitivity=.25, no_auto_home=True, body_lift=True, lift_speed_mm_s=15.)
            with patch('vr_client.time.monotonic', lambda:clock[0]), patch('vr_client.time.sleep',sleep), \
                 patch('vr_client.key',return_value=''), patch('vr_client.keyboard',contextlib.nullcontext), \
                 contextlib.redirect_stdout(io.StringIO()), self.assertRaises(EndSimulation):
                operate(args, VendorKinematics(), NS(read=read), connection)
        self.assertTrue(any(r and r.get('lift') == .8 and r['arms'] == {} for r in sent[10:25]))
        self.assertTrue(all(r is None or 'lift' not in r for r in sent[25:]))

    def test_remote_bundle_reuses_body_model_and_current_calibration(self):
        args = NS(execute=True, body_lift=True, lift_speed_mm_s=15.)
        command = remote_command(args)
        loader = shlex.split(shlex.split(command)[2].split('exec python3 -u -c ',1)[1])[0]
        tree = ast.parse(loader)
        encoded = next(n.args[0].value for n in ast.walk(tree) if isinstance(n,ast.Call)
                       and ast.unparse(n.func) == 'base64.b64decode')
        bundle = json.loads(zlib.decompress(base64.b64decode(encoded)))
        self.assertEqual(bundle['body_core'], (paths.SHARED/'body_control/body_core.py').read_text())
        argv = next(ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign)
                    and ast.unparse(n.targets[0]) == 'sys.argv')
        config = json.loads(argv[argv.index('--body-config-json')+1])
        self.assertEqual(config, load_config().config)


if __name__ == '__main__':
    unittest.main()
