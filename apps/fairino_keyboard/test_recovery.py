"""Exercise stale-controller recovery with real input and hardware guards."""
import threading
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

import robot_worker as rw


class RecoveryRig:
    def __init__(self, controls=None):
        self.now = 100.
        self.controls = dict(controls if controls is not None else {
            rw.CHASSIS: 'active', rw.ANGLE: 'inactive',
            'fairino_keyboard_chassis': 'unconfigured'})
        self.unloads, self.messages = [], []
        self.reads = self.pumps = 0
        self.refresh_hw = self.refresh_upper = True
        self.unload_ok, self.remove = True, True
        self.on_unload = self.on_pump = self.on_list = None
        self.worker = w = rw.Worker.__new__(rw.Worker)
        w.args = NS(linear=rw.DEFAULT_LINEAR, angular=rw.DEFAULT_ANGULAR)
        w.hw = dict(error=0, errorcode=[0]*8, enabled=[False]*8,
                    position=[0.]*8, velocity=[0.]*8,
                    command_position=[0.]*8, command_velocity=[0.]*8)
        w.hw_time = w.upper_time = w.received = self.now
        w.upper, w.input_error = 0, ''
        w.homing = w.ready = False
        w.stop, w.input_lock = threading.Event(), threading.Lock()
        w.packet = dict(keys=[], focused=True, stop=False, ack=self.now)
        w.controllers, w.service, w.pump = self.list_controllers, self.service, self.pump
        w.status = lambda *a, **kw: self.messages.append(kw)
        w.emit = lambda **kw: self.messages.append(kw)

    def list_controllers(self, guarded=True):
        self.reads += 1
        if self.on_list:
            self.on_list(self)
        if guarded:
            self.worker.guard()
        return dict(self.controls)

    def pump(self, seconds=.02, guarded=True):
        self.pumps += 1
        self.now += seconds
        w = self.worker
        if self.refresh_hw:
            w.hw_time = self.now
        if self.refresh_upper:
            w.upper_time = self.now
        w.received = w.packet['ack'] = self.now
        if self.on_pump:
            self.on_pump(self)
        if guarded:
            w.guard()

    def service(self, typ, path, request, guarded=True):
        if path != '/controller_manager/unload_controller':
            raise AssertionError('Recovery may only unload: '+path)
        self.unloads.append(request.name)
        if self.unload_ok and self.remove:
            self.controls.pop(request.name, None)
        if self.on_unload:
            self.on_unload(self)
        # A real guarded ROS future checks the live guard after its response.
        if guarded:
            self.worker.guard()
        return NS(ok=self.unload_ok)


class RecoveryTests(unittest.TestCase):
    def recover(self, rig):
        with patch.object(rw.time, 'monotonic', side_effect=lambda: rig.now), \
                patch.dict(rw.__dict__, {'UnloadController': NS(Request=NS)}):
            return rig.worker.recover_stale_controllers()

    def assert_refused(self, rig, message):
        with self.assertRaisesRegex(RuntimeError, message):
            self.recover(rig)
        self.assertFalse(any(m.get('recovery_complete') for m in rig.messages))

    def test_reclaims_both_inactive_and_unconfigured_but_keeps_vendor_chassis(self):
        rig = RecoveryRig()
        result = self.recover(rig)
        self.assertEqual(set(rig.unloads), {rw.ANGLE, 'fairino_keyboard_chassis'})
        self.assertEqual(result, {rw.CHASSIS: 'active'})
        self.assertNotIn(rw.CHASSIS, rig.unloads)
        self.assertGreaterEqual(rig.pumps, 3)  # New feedback before each and after both.
        self.assertTrue(rig.messages[-1]['recovery_complete'])

    def test_no_residual_controller_only_reads_current_list(self):
        controls = {rw.CHASSIS: 'inactive', 'joint_state_broadcaster': 'active'}
        rig = RecoveryRig(controls)
        self.assertEqual(self.recover(rig), controls)
        self.assertEqual(rig.reads, 1)
        self.assertEqual(rig.pumps, 0)
        self.assertEqual(rig.unloads, [])
        self.assertEqual(rig.messages, [])

    def test_any_active_or_abnormal_candidate_rejects_before_first_unload(self):
        for name in (rw.ANGLE, 'fairino_keyboard_chassis'):
            for state in ('active', 'finalized', 'unknown', ''):
                with self.subTest(name=name, state=state):
                    rig = RecoveryRig()
                    rig.controls[name] = state
                    self.assert_refused(rig, '拒绝自动回收')
                    self.assertEqual(rig.unloads, [])

    def test_unknown_active_controller_rejects_before_first_unload(self):
        rig = RecoveryRig()
        rig.controls['another_owner'] = 'active'
        self.assert_refused(rig, '未知的活动控制器')
        self.assertEqual(rig.unloads, [])

    def test_real_guards_refuse_unsafe_hardware_or_input(self):
        cases = {
            'hardware fault': (lambda w: w.hw.update(error=1), '底盘故障'),
            'joint fault': (lambda w: w.hw['errorcode'].__setitem__(5, 5), '底盘故障'),
            'upper estop': (lambda w: setattr(w, 'upper', 1), '上身急停'),
            'enabled': (lambda w: w.hw['enabled'].__setitem__(0, True), '全部下使能且静止'),
            'measured moving': (lambda w: w.hw['velocity'].__setitem__(7, -.031), '全部下使能且静止'),
            'command moving': (lambda w: w.hw['command_velocity'].__setitem__(0, -.006), '速度指令近零'),
            'keyboard': (lambda w: w.packet.update(keys=['w']), '按键.*摇杆回中'),
            'joystick': (lambda w: w.packet.update(axes=[0., 0., .5]), '按键.*摇杆回中'),
            'hardware stale': (lambda w: setattr(w, 'hw_time', 99.), '底盘反馈'),
            'upper stale': (lambda w: setattr(w, 'upper_time', 99.), '上身急停反馈'),
            'heartbeat stale': (lambda w: setattr(w, 'received', 99.), '心跳中断'),
            'ack stale': (lambda w: w.packet.update(ack=99.), '往返通信超时'),
            'stop latch': (lambda w: w.stop.set(), '用户停止'),
            'stop packet': (lambda w: w.packet.update(stop=True), '用户停止'),
        }
        for name, (change, message) in cases.items():
            with self.subTest(name=name):
                rig = RecoveryRig()
                change(rig.worker)
                self.assert_refused(rig, message)
                self.assertEqual(rig.unloads, [])

    def test_existing_stationary_thresholds_and_neutral_axes_are_accepted(self):
        rig = RecoveryRig()
        rig.worker.hw['velocity'] = [.03, -.03]*4
        rig.worker.hw['command_velocity'] = [.005, -.005]*4
        rig.worker.packet['axes'] = [0., 0., 0.]
        self.assertEqual(self.recover(rig), {rw.CHASSIS: 'active'})

    def test_both_feedback_streams_must_advance_not_just_pump_once(self):
        for stream in ('hw', 'upper'):
            with self.subTest(stream=stream):
                rig = RecoveryRig()
                setattr(rig, 'refresh_'+stream, False)
                self.assert_refused(rig, '反馈.*未更新')
                self.assertGreater(rig.pumps, 1)
                self.assertEqual(rig.unloads, [])

    def test_unload_failure_or_persistent_object_prevents_success(self):
        for attribute, message in [('unload_ok', '回收上次临时控制器失败'),
                                   ('remove', '临时控制器仍存在')]:
            with self.subTest(attribute=attribute):
                rig = RecoveryRig()
                setattr(rig, attribute, False)
                self.assert_refused(rig, message)
                self.assertEqual(len(rig.unloads), 1)

    def test_service_exception_prevents_second_unload(self):
        rig = RecoveryRig()
        def fail(_):
            raise RuntimeError('ROS 请求超时')
        rig.on_unload = fail
        self.assert_refused(rig, 'ROS 请求超时')
        self.assertEqual(len(rig.unloads), 1)

    def test_feedback_or_input_changes_during_service_prevent_further_recovery(self):
        cases = {
            'fault': (lambda w: w.hw.update(error=1), '底盘故障'),
            'enabled': (lambda w: w.hw['enabled'].__setitem__(3, True), '全部下使能且静止'),
            'speed': (lambda w: w.hw['velocity'].__setitem__(1, .1), '全部下使能且静止'),
            'command': (lambda w: w.hw['command_velocity'].__setitem__(6, .02), '速度指令近零'),
            'joystick': (lambda w: w.packet.update(axes=[.1, 0., 0.]), '按键.*摇杆回中'),
            'stop': (lambda w: w.stop.set(), '用户停止'),
        }
        for name, (change, message) in cases.items():
            with self.subTest(name=name):
                rig = RecoveryRig()
                rig.on_unload = lambda current: change(current.worker)
                self.assert_refused(rig, message)
                self.assertEqual(len(rig.unloads), 1)

    def test_changed_controller_state_is_rechecked_before_unload(self):
        rig = RecoveryRig()
        def activate(current):
            if current.reads == 2:
                current.controls[rw.ANGLE] = 'active'
        rig.on_list = activate
        self.assert_refused(rig, '拒绝自动回收')
        self.assertEqual(rig.unloads, [])

    def test_a_controller_reappearing_after_unload_blocks_takeover(self):
        rig = RecoveryRig()
        def reappear(current):
            if len(current.unloads) == 2:
                current.controls[rw.ANGLE] = 'inactive'
        rig.on_pump = reappear
        self.assert_refused(rig, '临时控制器再次出现')
        self.assertEqual(len(rig.unloads), 2)


if __name__ == '__main__':
    unittest.main()
