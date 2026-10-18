#!/usr/bin/env python3
"""ROS backend for coordinated torso height/attitude targets."""
import argparse
import fcntl
import signal
import time

import robot_probe as rp
from probe_core import StepCancelled, fresh_request
from body_core import BODY, BodyModel, plan


class BodyProbe(rp.JointProbe):
    allow_unfocused = True

    def __init__(self, args, model):
        self.model = model
        self.moving_joints = BODY
        super().__init__(args)

    def report(self, force=False, **extra):
        body = self.model.state(self.hw['position']) if self.hw else None
        super().report(force, body=body, upright_confirmed=self.model.config['upright_confirmed'], **extra)

    @staticmethod
    def goal_for(plan_data):
        goal = rp.FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = rp.JOINTS
        for source in plan_data['points']:
            point = rp.JointTrajectoryPoint()
            point.positions = source['positions']
            point.velocities = source['velocities']
            point.accelerations = source['accelerations']
            point.time_from_start = rp.Duration(seconds=source['time']).to_msg()
            goal.trajectory.points.append(point)
        for name in rp.JOINTS:
            # These are following-error tolerances, not joint angle limits.
            # The user requested no reinstatement of the previous soft trips.
            goal.path_tolerance.append(rp.JointTolerance(name=name, position=-1., velocity=-1., acceleration=-1.))
            goal.goal_tolerance.append(rp.JointTolerance(name=name, position=-1., velocity=-1., acceleration=-1.))
        return goal

    def execute(self, planned):
        self.guard()
        self.ready = False
        self.goal = self.pending_goal = None
        self.phase = ' → '.join(planned['stages'])+f"；约 {planned['duration']:.1f} 秒"
        self.emit(body_plan=planned)
        self.report(True)
        self.pending_goal = self.trajectory_client.send_goal_async(self.goal_for(planned))
        self.goal = self.wait(self.pending_goal, 3)
        if not self.goal.accepted:
            raise RuntimeError('身体轨迹被控制器拒绝')
        result = self.wait(self.goal.get_result_async(), planned['duration']+4)
        if result.status != 4 or result.result.error_code != 0:
            raise RuntimeError('身体轨迹执行失败：'+result.result.error_string)
        self.hold_positions = planned['final'][:]

    def run(self):
        self.prepare()
        if self.args.check or self.args.prepare_check:
            return
        self.phase = '全部关节已使能并保持；点击执行才调节身体'
        self.ready = True
        self.report(True, references=self.model.references())
        while True:
            self.pump()
            with self.input_lock:
                packet = self.packet.copy()
            request = packet.get('request') if packet.get('focused') else None
            if not request or request.get('id', 0) <= self.last_request:
                continue
            self.last_request = request['id']
            try:
                fresh_request(request, time.monotonic())
                if request.get('kind') != 'body_target':
                    raise ValueError('无效的身体控制请求')
                planned = plan(self.model, self.hw['position'], self.hold_positions,
                               request['target'], request['speed_deg_s'])
            except (ValueError, KeyError, TypeError) as exc:
                self.phase = str(exc)
                self.report(True, rejected=str(exc))
                continue
            self.cancel_count = packet.get('cancel', 0)
            try:
                self.execute(planned)
                message = '轨迹完成，关节继续使能并保持；当前角度见实测反馈'
            except StepCancelled:
                self.phase = '正在中止运动并保持'
                self.cancel_step()
                message = '已中止运动，关节继续使能并保持'
            self.cancel_count = None
            self.goal = self.pending_goal = None
            self.ready = True
            self.phase = message
            with self.input_lock:
                queued = self.packet.get('request')
                if queued:
                    self.last_request = max(self.last_request, queued.get('id', 0))
            self.report(True)


def load_ros():
    import rclpy
    import yaml
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter
    from rclpy.duration import Duration
    from hardware_state_broadcaster_msgs.msg import HardwareState
    from fairino_hardware_command_msgs.action import StringCommand
    from control_msgs.action import FollowJointTrajectory
    from control_msgs.msg import JointTolerance
    from trajectory_msgs.msg import JointTrajectoryPoint
    from controller_manager_msgs.srv import (ListControllers, LoadController, ConfigureController,
                                            SwitchController, UnloadController)
    from rcl_interfaces.srv import SetParameters
    for name, value in locals().copy().items():
        setattr(rp, name, value)


def main():
    parser = argparse.ArgumentParser()
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--check', action='store_true')
    modes.add_argument('--prepare-check', action='store_true')
    args = parser.parse_args()
    model = BodyModel(CONFIG)
    if not (args.check or args.prepare_check) and not model.config.get('upright_confirmed'):
        raise ValueError('竖直姿态基准尚未确认，拒绝启动实机控制')
    lock = None
    if not args.check:
        lock = open('/home/fairino/.fairino_upper_identify.lock', 'a')
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    load_ros()
    rp.rclpy.init()
    probe = BodyProbe(args, model)
    signal.signal(signal.SIGTERM, lambda *_: probe.exit.set())
    signal.signal(signal.SIGINT, lambda *_: probe.exit.set())
    code = 0
    try:
        probe.run()
    except Exception as exc:
        code = 1
        probe.emit(error=str(exc))
    finally:
        if not probe.cleanup():
            code = 1
        deadline = time.monotonic()+1
        while probe.output.unfinished_tasks and time.monotonic() < deadline:
            time.sleep(.01)
        probe.node.destroy_node()
        rp.rclpy.shutdown()
        if lock:
            lock.close()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
