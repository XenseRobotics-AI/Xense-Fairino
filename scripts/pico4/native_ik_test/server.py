#!/usr/bin/env python3
"""Standalone robot-side native IK owner; imports never connect or enable.

Cartesian intent can be replaced in transit. Joint history is generated locally
and committed only after a complete independently checked 20-axis publication.
"""
from __future__ import annotations

import argparse
from collections import deque
import fcntl
import json
import math
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
for relative in ('', 'scripts/pico4/fairino', 'apps/fairino_tcp_probe',
                 'apps/fairino_upper_probe', 'apps/fairino_upper_probe/body_control'):
    location = str(ROOT/relative)
    if location not in sys.path:
        sys.path.insert(0, location)

import robot_probe as rp
from tcp_core import MODEL_ID
from tool_frames import tool_config, validate_runtime
from vr_core import KIND as VERIFIED_KIND, matrix, validate, direct_positions
from vr_robot import VRProbe, load_ros, STOP_SPEED_RAD_S, HOLD_COMMAND_TOLERANCE_RAD
from wrist_guard import validate_policy
from scripts.pico4.native_ik_test.protocol import KIND

STOP_SETTLE_DWELL_S = .1


def validate_intent(request, now, tools, timeout):
    """Only Cartesian transforms cross this protocol, never joint solutions."""
    if not isinstance(request, dict) or request.get('kind') != KIND or request.get('model') != MODEL_ID:
        raise ValueError('native TCP protocol/model mismatch')
    if request.get('tool_id') != tools['id']:
        raise ValueError('native TCP tool configuration mismatch')
    if type(request.get('id')) is not int or request['id'] <= 0:
        raise ValueError('invalid native TCP request id')
    created = request.get('created')
    if (type(created) not in (int, float) or not math.isfinite(created)
            or not 0 <= now-created <= timeout):
        raise ValueError('native TCP input expired')
    goals = request.get('goals')
    if not isinstance(goals, dict) or not goals or not set(goals) <= {'left', 'right'}:
        raise ValueError('native TCP goals must contain left/right transforms')
    if any(name in request for name in ('arms', 'target', 'body_lift', 'lift', 'lift_axis')):
        raise ValueError('joint/body targets are not accepted by native TCP protocol')
    return {side: matrix(value) for side, value in goals.items()}


class NativeProbe(VRProbe):
    def __init__(self, args, planner):
        self.planner = planner
        self.native_active = False
        self.shutdown_requested = False
        self.input_closed = False
        self.mode = 'initializing'
        self.need_rearm = True
        self.command_stopped = True
        self.settled = False
        self.stop_reason = ''
        self.stop_started = self.stop_published = None
        self.stop_feedback = None
        self.settle_since = None
        self.stop_completed = math.inf
        self.last_intent = self.last_cancel = None
        self.input_timing = {}
        self.tick_id = 0
        self.actual_dt = 0.
        self.solve_ms = self.validate_publish_ms = 0.
        self.native_envelope_checked = False
        self.last_result = None
        self.published_history = deque()
        self.command_mismatch_since = None
        self.command_feedback_error = 0.
        self.actual_max_speed = 0.
        super().__init__(args)

    def read_input(self):
        # Streaming EOF/exit holds the last published position and confirms
        # measured stop before disable. Startup cancellation aborts preparation.
        try:
            for line in sys.stdin:
                now = time.monotonic()
                try:
                    packet = json.loads(line)
                    if not isinstance(packet, dict):
                        raise ValueError('input packet must be an object')
                except ValueError as exc:
                    packet = {'request': None, '_native_input_error': str(exc)}
                packet['server_read_monotonic'] = now
                with self.input_lock:
                    self.packet, self.received = packet, now
                    self.input_event.set()
                if packet.get('exit'):
                    self.shutdown_requested = True
                    if not self.native_active:
                        self.exit.set()
        except (OSError, ValueError) as exc:
            self.fault = str(exc)
        finally:
            self.input_closed = self.shutdown_requested = True
            if not self.native_active:
                self.exit.set()
            self.input_event.set()

    def wrist_request(self, packet):
        try:
            wanted = validate_policy((packet or {}).get('wrist_guard'))
            if not wanted['enabled']:
                raise ValueError('native IK requires the wrist guard to remain enabled')
            if self.wrist.active is not None and wanted != self.wrist.active:
                raise ValueError('native IK wrist policy is locked for this session')
        except ValueError as exc:
            self.wrist.error = str(exc)
            return None
        return super().wrist_request(packet)

    def guard(self, link=True, cancel=True):
        # Keep inherited hardware faults/GPIO/held-axis/enable guards. The local
        # input lease starts position hold; it replaces the old link cutoff.
        super().guard(link=link and not self.native_active,
                      cancel=cancel and not self.native_active)
        if self.native_active or self.session_enabled:
            self._fresh_hardware()
        if self.native_active:
            self._command_health(time.monotonic())

    def _command_health(self, now):
        self.actual_max_speed = max(abs(math.degrees(v)) for v in self.hw['velocity'][:14])
        if not self.published_history:
            return
        age_limit = self.args.feedback_age_ms/1000
        latest = self.published_history[-1][1]
        command = self.hw['command_position']
        self.command_feedback_error = max(abs(a-b) for a, b in zip(command, latest))
        # A fresh 100 Hz hardware message may acknowledge an earlier 100 Hz
        # command. Accept any recently published complete target, never claim
        # that publisher.publish itself is controller acknowledgement.
        matched = any(stamp >= self.hw_time-age_limit
                      and max(abs(a-b) for a, b in zip(command, target)) <= HOLD_COMMAND_TOLERANCE_RAD
                      for stamp, target in self.published_history)
        # A final held target stays valid after its publication ages out.
        matched |= self.command_feedback_error <= HOLD_COMMAND_TOLERANCE_RAD
        if matched:
            self.command_mismatch_since = None
        elif self.command_mismatch_since is None:
            self.command_mismatch_since = now
        elif now-self.command_mismatch_since > age_limit:
            self.emit(native_event='command_feedback_mismatch',
                      max_command_error_rad=self.command_feedback_error,
                      hardware_command_position=command,
                      last_published_target=latest,
                      hardware_monotonic=self.hw_time,
                      hardware_age_ms=1000*(now-self.hw_time))
            raise RuntimeError('native controller command feedback did not match recent publications')
        while len(self.published_history) > 1 and self.published_history[0][0] < now-2*age_limit:
            self.published_history.popleft()

    def report(self, force=False, **extra):
        now = time.monotonic()
        # Probe.prepare's generic read-only success precedes our stricter
        # initial-envelope check. Do not let the client finish prematurely.
        if not self.native_envelope_checked:
            extra.pop('checked', None)
        if force or now-getattr(self, 'last_status', 0.) >= 1/self.args.feedback_hz:
            rp.Probe.report(self, True, control_protocol=KIND, jogging=self.mode == 'tracking',
                hardware_observed_hz=self.hardware_rate.hz(now) if hasattr(self, 'hardware_rate') else 0.,
                native_test=dict(mode=self.mode, need_rearm=self.need_rearm,
                    command_stopped=self.command_stopped, settled=self.settled,
                    stop_reason=self.stop_reason, tick_id=self.tick_id,
                    stop_feedback=self.stop_feedback,
                    actual_dt_ms=1000*self.actual_dt, command_hz=self.args.command_hz,
                    command_tracking='direct_constrained_ik',
                    stop_strategy='hold_last_published',
                    speed_deg_s=None, acceleration_deg_s2=None, braking_deg_s2=None,
                    feedback_age_ms=self.args.feedback_age_ms,
                    input_timeout_ms=self.args.input_timeout_ms,
                    input_timing=self.input_timing,
                    actual_max_speed_deg_s=self.actual_max_speed,
                    measured_speed_trip_deg_s=None,
                    command_feedback_error_rad=self.command_feedback_error,
                    solve_ms=self.solve_ms, validate_publish_ms=self.validate_publish_ms,
                    command_trace=getattr(self, 'command_trace', None),
                    result=self.last_result), **extra)

    def enable_all(self):
        self._feedback_preflight()
        self._fresh_hardware()
        self.planner.reset(self.hw)
        self.emit(native_event='initial_envelope_checked')
        super().enable_all()

    def _feedback_preflight(self, duration=1.):
        """Refuse an unsuitable feedback stream before any enable attempt.

        Existing startup guards still run while collecting. The predictive
        bound is assessed over the entire window, never silently widened to
        accommodate historically slow hardware feedback.
        """
        if self.enable_attempted or self.session_enabled or any(self.hw['enabled']):
            raise RuntimeError('feedback preflight requires all joints disabled')
        started = time.monotonic()
        frames = set()
        ages = []
        while True:
            now = time.monotonic()
            ages.append(now-self.hw_time)
            frames.add(self.hw_time)
            if now-started >= duration:
                break
            self.pump(min(.005, duration-(now-started)))
        observed = time.monotonic()-started
        bound = self.args.feedback_age_ms/1000
        exceeded = sum(not math.isfinite(age) or not 0 <= age <= bound for age in ages)
        required = max(2, math.ceil(observed/bound-1e-9))
        accepted = exceeded == 0 and len(frames) >= required
        self.emit(native_event='feedback_preflight', result='passed' if accepted else 'rejected',
                  duration_ms=1000*observed, observations=len(ages), unique_frames=len(frames),
                  required_frames=required, min_age_ms=1000*min(ages), max_age_ms=1000*max(ages),
                  limit_ms=self.args.feedback_age_ms, exceed_count=exceeded)
        if not accepted:
            raise RuntimeError('native feedback preflight rejected before enable: '
                f'max age {1000*max(ages):.1f} ms, limit {self.args.feedback_age_ms:.1f} ms; '
                f'{len(frames)} unique frames, {required} required; {exceeded} aged observations')

    def _fresh_hardware(self):
        age = time.monotonic()-self.hw_time
        if self.hw is None or not 0 <= age <= self.args.feedback_age_ms/1000:
            raise RuntimeError(f'native hardware age {1000*age:.1f} ms exceeds feedback age bound')

    def _begin_stop(self, reason, now):
        if self.mode not in ('stopping', 'settling'):
            self.mode = 'stopping'
            self.stop_started = now
            self.stop_published = None
            self.stop_feedback = None
            self.settle_since = None
            self.stop_reason = reason
            self.need_rearm = True
            self.settled = self.ready = False
            self.phase = '原生 IK 保持最后指令：'+reason
            self.emit(native_event='hold_started', reason=reason, server_monotonic=now,
                      input_timing=dict(self.input_timing))
            self.report(True)

    def _packet_goals(self, now):
        with self.input_lock:
            packet, received = dict(self.packet or {}), self.received
            # The receiver can update after the control cycle starts (including
            # while guard() runs). Sample the clock AFTER this snapshot; using
            # the cycle start makes a fresh packet appear to come from the future.
            checked = time.monotonic()
        request = packet.get('request')

        def age_ms(stamp):
            if type(stamp) in (int, float) and math.isfinite(stamp):
                return 1000*(checked-stamp)
            return None

        self.input_timing = dict(cycle_started_monotonic=now, checked_monotonic=checked,
            received_monotonic=received, heartbeat_age_ms=age_ms(received),
            round_trip_age_ms=age_ms(packet.get('ack')),
            target_age_ms=age_ms(request.get('created')) if isinstance(request, dict) else None,
            limit_ms=self.args.input_timeout_ms)
        now = checked
        if self.shutdown_requested or self.input_closed or self.exit.is_set():
            return {}, 'shutdown_requested', False, received
        if packet.get('_native_input_error'):
            return {}, 'malformed_input: '+packet['_native_input_error'], False, received
        if not 0 <= now-received <= self.args.input_timeout_ms/1000:
            return {}, 'input_heartbeat_timeout', False, received
        ack = packet.get('ack')
        if (type(ack) not in (int, float) or not math.isfinite(ack)
                or not 0 <= now-ack <= self.args.input_timeout_ms/1000):
            return {}, 'input_round_trip_timeout', False, received
        if not packet.get('focused'):
            return {}, 'input_unfocused', False, received
        cancel = packet.get('cancel', 0)
        if self.last_cancel is not None and cancel != self.last_cancel:
            self.last_cancel = cancel
            return {}, 'input_cancelled', False, received
        self.last_cancel = cancel
        request = self.wrist_request(packet)
        if self.wrist.error or self.wrist.active is None:
            return {}, 'wrist_handshake: '+self.wrist.error, False, received
        if packet.get('request') is None:
            return {}, '', True, received
        if request is None:
            return {}, 'wrist_request_rejected', False, received
        try:
            goals = validate_intent(request, now, self.tools, self.args.input_timeout_ms/1000)
            if request['id'] < self.last_request:
                raise ValueError('old native TCP request id')
            if request['id'] == self.last_request and request != self.last_intent:
                raise ValueError('native TCP request id reused with different content')
            self.last_intent, self.last_request = request, request['id']
            return goals, '', False, received
        except ValueError as exc:
            return {}, str(exc), False, received

    def _publish_proposal(self, proposal, now):
        work_started = time.perf_counter()
        self._fresh_hardware()
        self.tick_id += 1
        checked = dict(kind=VERIFIED_KIND, model=MODEL_ID, tool_id=self.tools['id'],
                       id=self.tick_id, created=now, arms=proposal['arms'])
        targets = validate(self.hw, checked, now, self.tools)
        if set(targets) != {'left', 'right'}:
            raise ValueError('native proposal must contain both arms atomically')
        current = self.hold_positions[:] if self.path is None else self.path[-1][1][:]
        for side, offset in (('left', 0), ('right', 7)):
            reference = proposal['arms'][side]['reference']
            if max(abs(a-b) for a, b in zip(reference, current[offset:offset+7])) > 1e-10:
                raise ValueError('native reference differs from last published command')
            if proposal['results'][side]['requested']:
                for start in (current[offset:offset+7], self.hw['position'][offset:offset+7]):
                    self.wrist.guard.arm_segment(start, targets[side], self.hw['velocity'][offset:offset+7])
            else:
                # Match VRProbe.stop_stream: holding cannot chase a new target
                # or be blocked by a new velocity reserve on an already-issued
                # position. Hardware, static limits, FK and exact identity are
                # still checked; measured stop remains a separate handshake.
                if targets[side] != current[offset:offset+7]:
                    raise ValueError('hold proposal changed the last published position')
                self.wrist.guard.arm_segment(targets[side], targets[side], [0.]*7)
        points = direct_positions(current, targets, self.hold_positions)
        self.guard()
        self.moving_joints = list(range(1, 15))
        self.publish_positions(points, now)
        self.planner.commit(proposal)
        published = time.monotonic()
        self.published_history.append((published, points[-1][1][:]))
        self.command_trace = dict(id=self.tick_id, intent_id=self.last_request, server_published=published)
        self.trace_target = points[-1][1][:]
        self.command_stopped = bool(proposal['command_stopped'])
        self.last_result = {side: {key: result.get(key) for key in
            ('success', 'status', 'position_error_mm', 'orientation_error_deg', 'command_stopped')}
            for side, result in proposal['results'].items()}
        self.validate_publish_ms = 1000*(time.perf_counter()-work_started)
        return published

    def _propose(self, goals, dt):
        started = time.perf_counter()
        try:
            return self.planner.propose(goals, self.hw, dt)
        finally:
            self.solve_ms = 1000*(time.perf_counter()-started)

    def _confirm_settled(self, now):
        state = self.hold_feedback(self.path[-1][1], self.stop_published)
        self.stop_feedback = state
        candidate = (state['fresh_feedback']
                and state['max_command_error_rad'] <= HOLD_COMMAND_TOLERANCE_RAD
                and state['max_velocity_rad_s'] <= STOP_SPEED_RAD_S)
        if not candidate:
            self.settle_since = None
            return
        if self.settle_since is None:
            self.settle_since = self.hw_time
        # Advance the dwell using distinct hardware-frame times, not repeated
        # status ticks. A residual-velocity/command excursion restarts the dwell.
        dwell = self.hw_time-self.settle_since
        if dwell+1e-12 >= STOP_SETTLE_DWELL_S:
            self.emit(native_event='measured_stop_confirmed', reason=self.stop_reason,
                      dwell_ms=1000*dwell, **state)
            self.hold_positions = self.hw['command_position'][:]
            self.planner.reset(self.hw)
            self.path = None
            self.moving_joints = None
            self.mode = 'idle'
            self.settled = self.command_stopped = True
            self.stop_completed = now
            self.phase = '已确认停止；等待双 Grip 松开后重新启动'
            self.report(True)

    def cycle(self, now, actual_dt):
        """One local scheduled tick; unit tests provide simulated feedback."""
        self.actual_dt = actual_dt
        self.guard()
        goals, reason, neutral, received = self._packet_goals(now)
        if actual_dt > self.args.max_cycle_gap_ms/1000:
            reason = 'control_cycle_gap'
        if reason:
            if self.mode != 'idle' or not self.settled:
                self._begin_stop(reason, now)
            else:
                changed = not self.need_rearm or self.stop_reason != reason
                self.need_rearm, self.ready = True, False
                self.stop_completed, self.stop_reason = now, reason
                self.phase = '已使能保持；输入暂停：'+reason+'；松开双 Grip 后重新启动'
                if changed:
                    self.emit(native_event='input_paused', reason=reason,
                              input_timing=dict(self.input_timing))
        elif not goals and self.mode == 'tracking':
            self._begin_stop('input_released', now)
        if self.mode == 'idle':
            if not reason and neutral and received > self.stop_completed:
                self.need_rearm, self.ready = False, True
                self.phase = '已使能就绪；握住对应 Grip 开始跟随'
            if goals and not self.need_rearm:
                self.mode = 'tracking'
                self.settled = False
                self.phase = '原生 IK 双臂遥操作；松开 Grip 保持最后指令'
            else:
                return not (self.shutdown_requested or self.input_closed or self.exit.is_set())
        if self.mode == 'settling':
            self._confirm_settled(now)
        else:
            # Diagnostic interval only: it no longer defines a velocity or
            # acceleration budget for direct IK joint-position targets.
            dt = actual_dt
            if not math.isfinite(dt) or dt <= 0:
                raise RuntimeError('invalid local control period')
            try:
                proposal = self._propose(goals if self.mode == 'tracking' else {}, dt)
                if self.mode == 'tracking' and time.monotonic()-now > self.args.max_cycle_gap_ms/1000:
                    raise ValueError('native planning missed control deadline')
                published = self._publish_proposal(proposal, time.monotonic())
            except ValueError as exc:
                if self.mode != 'tracking':
                    raise RuntimeError('local position hold failed: '+str(exc)) from exc
                self._begin_stop('proposal_rejected: '+str(exc), now)
                proposal = self._propose({}, dt)
                published = self._publish_proposal(proposal, time.monotonic())
            if self.mode == 'stopping' and self.command_stopped:
                self.mode, self.stop_published = 'settling', published
                self.phase = '已保持最后位置指令；等待新硬件反馈确认停止'
        if self.mode in ('stopping', 'settling') and now-self.stop_started > self.args.stop_timeout_s:
            raise RuntimeError('native local stop/settle deadline exceeded')
        return True

    def run(self):
        self.moving_joints = self.path = None
        self.prepare()
        if self.args.check or self.args.prepare_check:
            self._fresh_hardware()
            self.planner.reset(self.hw)
            self.native_envelope_checked = True
            self.report(True, checked=True, native_envelope_checked=True)
            return
        self._fresh_hardware()
        self.stationary_enabled()
        self.planner.reset(self.hw)
        self.publisher = self.node.create_publisher(rp.JointTrajectory,
            f'/{rp.CONTROLLER}/joint_trajectory', 1)
        deadline = time.monotonic()+3
        while not self.publisher.get_subscription_count():
            if time.monotonic() > deadline:
                raise RuntimeError('native dual-arm trajectory subscriber is not ready')
            self.pump()
        self.native_active = True
        self.mode, self.ready, self.settled = 'idle', False, True
        self.stop_completed = time.monotonic()
        self.report(True)
        period = 1/self.args.command_hz
        previous = time.monotonic()
        deadline = previous+period
        while True:
            while time.monotonic() < deadline:
                self.pump(min(.001, max(.0001, deadline-time.monotonic())))
            now = time.monotonic()
            if not self.cycle(now, now-previous):
                return
            previous = now
            deadline += period
            if deadline < time.monotonic():
                deadline = time.monotonic()+period

    def request_shutdown(self):
        self.shutdown_requested = True
        if not self.native_active:
            self.exit.set()


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true')
    mode.add_argument('--execute', action='store_true')
    mode.add_argument('--prepare-check', action='store_true')
    parser.add_argument('--tools-json')
    for name, default in (('command-hz', 100.), ('feedback-hz', 100.),
                          ('feedback-age-ms', 60.), ('input-timeout-ms', 150.),
                          ('max-cycle-gap-ms', None), ('stop-timeout-s', 3.)):
        parser.add_argument('--'+name, type=float, default=default)
    args = parser.parse_args(argv)
    args.check = args.check or not (args.execute or args.prepare_check)
    if any(not math.isfinite(value) or value <= 0 for value in vars(args).values()
           if isinstance(value, float)):
        parser.error('timing limits must be finite positive values')
    if args.max_cycle_gap_ms is None:
        args.max_cycle_gap_ms = max(30., 3000/args.command_hz)
    if (args.command_hz > 200 or args.feedback_hz > 200
            or args.max_cycle_gap_ms < 1000/args.command_hz
            or args.stop_timeout_s <= STOP_SETTLE_DWELL_S
            or args.input_timeout_ms >= 400):
        parser.error('invalid native test timing configuration')
    return args


def _main(argv=None):
    args = arguments(argv)
    tools = validate_runtime(json.loads(args.tools_json)) if args.tools_json else tool_config()
    print(rp.PREFIX+json.dumps(dict(native_event='compiling_native_before_ros')), flush=True)
    from scripts.pico4.native_ik_test.planner import NativePlanner
    planner = NativePlanner(tools=tools)
    print(rp.PREFIX+json.dumps(dict(native_event='native_ready_before_ros')), flush=True)
    code = 0
    with open('/home/fairino/.fairino_upper_identify.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        load_ros()
        rp.rclpy.init()
        probe = None
        try:
            probe = NativeProbe(args, planner)
            signal.signal(signal.SIGTERM, lambda *_: probe.request_shutdown())
            signal.signal(signal.SIGINT, lambda *_: probe.request_shutdown())
            try:
                probe.run()
            except Exception as exc:
                code = 1
                probe.emit(native_event='fail_safe_cleanup', error=str(exc), mode=probe.mode,
                           stop_reason=probe.stop_reason)
        finally:
            try:
                if probe is not None:
                    try:
                        if not probe.cleanup():
                            code = 1
                    finally:
                        deadline = time.monotonic()+1
                        while probe.output.unfinished_tasks and time.monotonic() < deadline:
                            time.sleep(.01)
                        probe.node.destroy_node()
            finally:
                rp.rclpy.shutdown()
    return code


def main(argv=None):
    try:
        return _main(argv)
    except Exception as exc:
        # Compilation/dependency/ownership/ROS-init failures happen before
        # Probe's output thread exists; preserve a structured client diagnostic.
        print(rp.PREFIX+json.dumps(dict(native_event='startup_or_cleanup_failed',
                                       error=str(exc)), ensure_ascii=False), flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
