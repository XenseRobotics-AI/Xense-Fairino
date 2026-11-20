"""Exercise the real whole-body client and receiver with in-memory input, ROS I/O and clock."""
import ast
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

import paths
import robot_probe as rp
from body_core import load_config
from test_probe import STUBS, connected_wrist
from test_tcp import hardware, probe
from test_vr_demo import frame, hand
from test_vr_demo_fairino import isolated
from test_vr_fairino_packet import pose
from tool_frames import tool_config
from tools.ik_experiments.setup_fairino_ik import extension_path
from tcp_core import MODEL_ID
from vendor_kinematics import VendorKinematics
import vr_client
from vr_core import TORSO_JOINTS
from vr_fairino_ik import FairinoQPIK
from vr_home import load_home
from vr_link_check import synthetic_pose
from vr_robot import VRProbe, session_body


def initial_state(body):
    q = synthetic_pose(body, [0.]*20)
    home = load_home()['target']
    q[:14] = home['left'] + home['right']
    q[14], q[16] = .17, math.radians(.05)
    result = hardware()
    result.update(position=q[:], command_position=q[:], velocity=[0.]*20)
    return result


def worker_args(backend):
    args = NS(execute=True, body_lift=False, ik_backend=backend, no_wrist_guard=False,
              command_hz=None, receiver_hz=None)
    vr_client.configure_ik(args)
    setup = shlex.split(vr_client.remote_command(args))[2]
    loader = shlex.split(setup.split('exec python3 -u -c ', 1)[1])[0]
    return next(ast.literal_eval(node.value) for node in ast.parse(loader).body
                if isinstance(node, ast.Assign) and ast.unparse(node.targets[0]) == 'sys.argv')


class ReplayEnded(Exception):
    pass


def receiver(initial, requests, body, *, proof=None, speed=None, body_lift=False, guard_enabled=True):
    p = probe()
    p.__class__ = VRProbe
    p.body, p.tools = body, tool_config()
    p.args.body_lift = body_lift
    p.args.command_hz, p.args.receiver_hz, p.args.feedback_hz = 100., 500., 100.
    p.hw = copy.deepcopy(initial)
    p.baseline, p.hold_positions = p.hw['position'][:], p.hw['command_position'][:]
    if not guard_enabled:
        from wrist_guard import policy
        p.packet['wrist_guard'] = policy(False, 2)
        p.wrist.filter(p.packet, p.hw, idle=True)
    clock, index, published = [10.], [0], []

    def publish(message):
        assert message.joint_names == rp.JOINTS and len(message.points) == 1
        point = message.points[0]
        assert point.time_from_start == 0. and not point.velocities and not point.accelerations
        published.append((index[0], clock[0], tuple(p.moving_joints or ()), point.positions[:]))

    def feedback(dt=.005, guarded=True):
        clock[0] += dt
        if published:
            q = published[-1][3]
            p.hw['position'], p.hw['command_position'] = q[:], q[:]
        p.hw_time = p.received = p.packet['ack'] = clock[0]
        if guarded:
            p.guard()

    def prepare():
        p.packet['kinematics'] = proof or dict(model=MODEL_ID, reference=p.hw['position'][:14],
                                              body_reference=p.hw['command_position'][15:20])
        feedback(0., False)
        p.enable_all()
        p.ready = True

    def read(_):
        if index[0] == len(requests):
            raise ReplayEnded()
        feedback(.01)
        request = copy.deepcopy(requests[index[0]])
        if request is not None:
            request.update(created=clock[0], _wrist_guard=p.wrist.active.copy())
        index[0] += 1
        p.packet['request'] = request
        if speed and index[0] == speed[0]:
            p.hw['velocity'][speed[1]-1] = math.radians(speed[2])
            p.guard()

    p.prepare, p.pump, p.pump_input = prepare, feedback, read
    p.node = NS(create_publisher=lambda *_: NS(publish=publish, get_subscription_count=lambda: 1))
    stubs = dict(STUBS, JointTrajectory=lambda: NS(points=[], joint_names=[]))
    with patch.dict(rp.__dict__, stubs), patch('vr_robot.time.monotonic', lambda: clock[0]):
        try:
            p.run()
        except ReplayEnded:
            pass
    return p, published


def client_sequence(kin, initial):
    clock = [10.]
    planner = FairinoQPIK(kin.tools, clock=lambda: clock[0])
    requests, proofs, finished = [], [], False
    hw = copy.deepcopy(initial)
    with tempfile.TemporaryDirectory() as directory:
        connection = connected_wrist(NS(events=queue.Queue(), done=threading.Event(), last_status=clock[0],
            latest=dict(hardware=hw, stamp=clock[0], ready=True, body_start_ready=True),
            log_path=Path(directory)/'offline'))
        def send(request, kinematics):
            nonlocal finished
            requests.append(request)
            proofs.append(kinematics)
            if request:
                q = hw['command_position'][:]
                for side, entry in request['arms'].items():
                    offset = 0 if side == 'left' else 7
                    q[offset:offset+7] = entry['target']
                for j, value in zip(TORSO_JOINTS, request.get('torso', {}).get('target', ())):
                    q[j-1] = value
                hw['position'], hw['command_position'] = q[:], q[:]
            elif len(requests) > 205 and requests[-2] and requests[-2].get('source') == 'home':
                finished = True

        def read():
            index = len(requests)
            if finished or index > 6000:
                raise ReplayEnded()
            distance = .3 * min(1., max(0., index-1)/100)
            hands = {side: hand(grip=float(1 <= index <= 201), position=(0., -distance, -distance))
                     for side in ('left', 'right')}
            hands['right']['primary'] = index == 203
            return frame(clock[0], **hands)

        def sleep(_):
            clock[0] += .01
            connection.last_status = connection.latest['stamp'] = clock[0]

        connection.send = send
        args = NS(ik_backend='fairino_qp', fairino_ik=planner, sensitivity=1., execute=True,
                  no_auto_home=True, freeze_buttons=False, mode='first_person', interpolation_delay_ms=0.,
                  command_hz=100., receiver_hz=500., feedback_hz=100.)
        with (patch('vr_client.time.monotonic', lambda: clock[0]), patch('vr_client.time.sleep', sleep),
              patch('vr_client.key', return_value=''), patch('vr_client.keyboard', contextlib.nullcontext),
              contextlib.redirect_stdout(io.StringIO())):
            try:
                vr_client.operate(args, kin, NS(read=read), connection)
            except ReplayEnded:
                pass
        logs = [json.loads(s) for s in connection.log_path.with_suffix('.actions.jsonl').read_text().splitlines()]
    assert finished and any(e.get('event') == 'home_completed' for e in logs)
    assert proofs[0]['body_reference'] == initial['command_position'][15:20]
    return requests, proofs[0]


class SessionTests(unittest.TestCase):
    def test_stop_waits_for_each_torso_command_and_measured_velocity(self):
        from test_vr_stop import StopConfirmationTests
        for j in TORSO_JOINTS:
            for failed in ('command', 'velocity'):
                with self.subTest(joint=j, failed=failed), StopConfirmationTests().simulation(
                        joint=j, fresh_after=.01, match_after=math.inf if failed == 'command' else .01,
                        stopped_after=math.inf if failed == 'velocity' else .01) as (p, clock, target, messages):
                    p.body, p.lift = load_config(live=True), None
                    p.moving_joints = [*range(1, 15), *TORSO_JOINTS]
                    p.hold_positions[j-1] = target[j-1]
                    with self.assertRaises(RuntimeError):
                        p.stop_stream('释放 Grip')
                    self.assertEqual(messages[0].points[0].positions, target)
                    evidence = p.events[-1]['stop_confirmation']
                    self.assertEqual(evidence['result'], 'failed')
                    self.assertEqual(evidence['command_error_joint' if failed == 'command' else 'velocity_joint'], j)
                    self.assertAlmostEqual(clock[0]-10., 3., delta=.011)

    def test_remote_args_and_conflicts(self):
        native = worker_args('native')
        qp = worker_args('fairino_qp')
        self.assertNotIn('--body-config-json', native)
        self.assertNotIn('--ik-backend', native)
        self.assertEqual(native[native.index('--command-hz')+1], '500.0')
        self.assertEqual(qp[qp.index('--command-hz')+1], '200.0')
        self.assertEqual(qp[qp.index('--receiver-hz')+1], '500.0')
        self.assertEqual(json.loads(qp[qp.index('--body-config-json')+1]), load_config(live=True).config)
        self.assertNotIn('--body-lift', qp)
        for guard, lift in ((False, True), (True, False)):
            args = NS(ik_backend='fairino_qp', command_hz=None, receiver_hz=None, no_wrist_guard=guard, body_lift=lift)
            with self.assertRaises(ValueError):
                vr_client.configure_ik(args)
        for args in (NS(ik_backend='fairino_qp'), NS(ik_backend='fairino_qp', body_lift=True)):
            with self.assertRaises(ValueError):
                session_body(args)


@unittest.skipUnless(extension_path().is_file(), 'Optional Fairino QP build absent')
class WholeBodyTests(unittest.TestCase):
    @isolated
    def test_client_follow_release_A_and_home_through_receiver(self):
        initial = initial_state(load_config(live=True))
        kin = VendorKinematics()
        requests, proof = client_sequence(kin, initial)
        p, output = receiver(initial, requests, load_config(live=True), proof=proof)
        rejects = [e['jog_rejected'] for e in p.events if 'jog_rejected' in e]
        self.assertEqual(rejects, [])
        self.assertEqual(p.accepted_rate.total, sum(r is not None for r in requests))
        self.assertEqual(sum(e.get('stop_confirmation', {}).get('result') == 'confirmed' for e in p.events), 2)
        start = initial['command_position']
        last, last_t = start, None
        peaks = {}
        for index, stamp, moving, q in output:
            request = requests[index-1]
            self.assertEqual([q[14], q[16]], [start[14], start[16]])
            phase = ('hold' if request is None else 'follow' if request.get('source') != 'home'
                     else 'return' if 'torso' in request else 'arm_home')
            speeds = peaks.setdefault(phase, [0.]*4)
            for k, j in enumerate(TORSO_JOINTS):
                if last_t is not None:
                    speeds[k] = max(speeds[k], abs(math.degrees(q[j-1]-last[j-1]))/(stamp-last_t))
            if any(q[j-1] != last[j-1] for j in TORSO_JOINTS):
                self.assertEqual(moving, (*range(1, 15), *TORSO_JOINTS))
            if phase == 'return':
                self.assertEqual(last[:14], q[:14])
            last, last_t = q, stamp
        self.assertEqual(last[14:], start[14:])
        self.assertGreater(max(peaks['follow']), .1)
        self.assertGreater(max(peaks['return']), .1)
        self.assertLessEqual(max(peaks['follow']), 5.+1e-8)
        self.assertLessEqual(max(peaks['return']), 8.+1e-8)
        print('OFFLINE_REPLAY', json.dumps(dict(accepted=p.accepted_rate.total, peaks=peaks,
            posted=len(output), moving_sets=sorted(set(item[2] for item in output)),
            home_arms=sum(bool(r and r.get('source') == 'home' and 'torso' not in r) for r in requests),
            home_torso=sum(bool(r and r.get('source') == 'home' and 'torso' in r) for r in requests),
            held=[start[14], start[16]], final_body_difference=[last[j-1]-start[j-1] for j in TORSO_JOINTS],
            hold=[e['stop_confirmation'] for e in p.events if 'stop_confirmation' in e])))

    @isolated
    def test_receiver_rejections_and_last_good_hold(self):
        k = VendorKinematics()
        solver = FairinoQPIK(k.tools)
        hw = initial_state(solver.body)
        q = hw['command_position']
        goal = solver.teleop_tcp('left', q, k.forward_tcp(q[:7], 'left')[0])
        valid = solver.request(k, hw, {'left': goal}, 1, 10.)
        for mode, reason in (('offset', '偏移'), ('speed', '速率'), ('no_config', '标定'),
                             ('lift', '--body-lift'), ('lift_mode', '互斥'), ('no_torso', '躯干'),
                             ('other_backend', '躯干'), ('guard_off', '腕部')):
            bad = copy.deepcopy(valid)
            bad['id'] = 2
            if mode == 'offset':
                bad['torso']['path_offset_mm'] += .001
            elif mode == 'speed':
                bad['torso']['target'][0] += math.radians(.09)
            elif mode == 'lift_mode':
                bad.update(lift=0., lift_epoch=1, lift_mode='preset_v1')
            elif mode == 'no_torso':
                del bad['torso']
            elif mode == 'other_backend':
                bad['ik_backend'] = 'native'
            steps = [valid, bad] if mode in ('offset', 'speed') else [bad]
            p, output = receiver(hw, steps, None if mode in ('no_config', 'other_backend') else solver.body,
                                 body_lift=mode=='lift', guard_enabled=mode!='guard_off')
            rejection = next(e['jog_rejected'] for e in p.events if 'jog_rejected' in e)
            self.assertIn(reason, rejection)
            self.assertEqual(p.accepted_rate.total, len(steps)-1)
            if output:
                self.assertEqual(output[0][3], output[-1][3])
                self.assertEqual(len(output), 2)
            print('OFFLINE_REJECT', mode, rejection)
        # Hold feedback can lag. Use the last published target and the local receive clock.
        forged = copy.deepcopy(valid)
        forged['torso']['target'][0] += math.radians(.021)
        p, output = receiver(hw, [forged], solver.body)
        self.assertIn('速率', p.events[-1]['jog_rejected'])
        self.assertFalse(output)
        slight = copy.deepcopy(valid)
        slight['id'] = 2
        slight['torso']['target'][0] += math.radians(.04)
        with self.assertRaisesRegex(RuntimeError, 'J18.*15°/s') as rejected:
            receiver(hw, [valid, slight, None], solver.body, speed=(3, 18, -15.01))
        print('OFFLINE_REJECT measured_speed', rejected.exception)
        with self.assertRaisesRegex(ValueError, 'bash run.sh body') as rejected:
            low = pose(solver.body, h=16.2, forward=.59, x=1.02).tolist()
            vr_client.wait_for_body_start(NS(latest=dict(body_start_ready=True, hardware=dict(
                position=low, command_position=low)), done=threading.Event(),
                last_status=vr_client.time.monotonic()), solver)
        print('OFFLINE_REJECT before_enable', rejected.exception)

    @isolated
    def test_enable_requires_validated_current_torso(self):
        p = probe()
        p.__class__ = VRProbe
        p.body, p.tools = load_config(live=True), tool_config()
        p.hw = initial_state(p.body)
        proof = dict(model=MODEL_ID, reference=p.hw['position'][:14])
        self.assertFalse(p.kinematics_matches(proof))
        proof['body_reference'] = p.hw['command_position'][15:20]
        self.assertTrue(p.kinematics_matches(proof))
        p.hw['command_position'][16] += .001
        self.assertFalse(p.kinematics_matches(proof))
