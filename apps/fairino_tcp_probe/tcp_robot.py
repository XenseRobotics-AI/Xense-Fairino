#!/usr/bin/env python3
"""Guarded ROS executor for small targets calculated by the vendor IK library."""
import argparse
import fcntl
import math
import signal
import time

import robot_probe as rp
from probe_core import StepCancelled,fresh_request
from tcp_core import MODEL_ID,ARMS,finite_vector,validate_plan
from tcp_jog import (PERIOD, LEASE, validate_velocity, sample, trajectory, braking)
from wrist_guard import WristBlocked


class TCPProbe(rp.Probe):
    moving_speed_limit = 8.0
    allow_unfocused = True  # Pause the stream on focus loss; keep position hold.

    def kinematics_matches(self, proof):
        return isinstance(proof, dict) and proof.get('model') == MODEL_ID

    def report(self, force=False, **extra):
        super().report(force, jogging=bool(getattr(self,'moving_joints',None)), **extra)

    def guard(self, link=True, cancel=True):
        super().guard(link,cancel)
        if getattr(self,'moving_joints',None) and self.hw is not None:
            for j in self.moving_joints:
                if abs(self.hw['position'][j-1]-self.hw['command_position'][j-1]) > .004:
                    raise RuntimeError(f'J{j} 轨迹跟踪误差超过 0.004 rad，结束控制')
            if getattr(self,'path',None) is not None:
                predicted,_=sample(self.path,self.hw_time-self.path_time)
                if max(abs(self.hw['command_position'][j-1]-predicted[j-1]) for j in self.moving_joints) > .006:
                    raise RuntimeError('控制器指令反馈未跟随连续轨迹，结束控制')

    def enable_all(self):
        self.phase = '等待两臂厂家正逆解校验，尚未使能'
        self.report(True)
        deadline = time.monotonic()+10
        while True:
            self.pump()
            with self.input_lock:
                proof = self.packet.get('kinematics')
            if self.kinematics_matches(proof):
                ref = finite_vector(proof.get('reference'),14,'运动学校验参考角度')
                if max(abs(math.degrees(a-b)) for a,b in zip(ref,self.hw['position'][:14])) <= .05:
                    break
            if time.monotonic() > deadline:
                raise RuntimeError('未收到与当前姿态匹配的厂家正逆解校验，拒绝使能')
        super().enable_all()

    def perform_tcp_step(self,request):
        self.stationary_enabled()
        plan = validate_plan(self.hw,self.hold_positions,request)
        self.wrist.guard.segment(plan['start'], plan['target'])
        self.ready = False
        self.moving_joints = plan['joints']
        self.goal = self.pending_goal = None
        self.phase = ('左臂' if plan['arm']=='left' else '右臂')+' TCP 小步移动中，其余关节保持'
        self.emit(tcp_plan=plan)
        self.report(True)
        goal = rp.FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = rp.JOINTS
        for positions,seconds in ((plan['start'],.1),(plan['target'],plan['duration']+.1)):
            p = rp.JointTrajectoryPoint()
            p.positions=positions;p.velocities=[0.]*20;p.accelerations=[0.]*20
            p.time_from_start=rp.Duration(seconds=seconds).to_msg()
            goal.trajectory.points.append(p)
        for name in rp.JOINTS:
            goal.path_tolerance.append(rp.JointTolerance(name=name,position=.004))
            goal.goal_tolerance.append(rp.JointTolerance(name=name,position=.001))
        goal.goal_time_tolerance.sec=1
        self.pending_goal=self.trajectory_client.send_goal_async(goal)
        self.goal=self.wait(self.pending_goal,3)
        if not self.goal.accepted:raise RuntimeError('TCP 点动轨迹被拒绝')
        result=self.wait(self.goal.get_result_async(),plan['duration']+3)
        if result.status != 4 or result.result.error_code != 0:
            raise RuntimeError('TCP 点动轨迹失败：'+result.result.error_string)
        if max(abs(a-b) for a,b in zip(self.hw['position'],plan['target'])) > .0015:
            raise RuntimeError('TCP 点动的关节目标未到位')
        self.hold_positions=plan['target'][:]

    def publish_points(self, points, now):
        self.wrist.guard.trajectory(points)
        msg=rp.JointTrajectory();msg.joint_names=rp.JOINTS
        # A robot-clock stamp aligns the explicit t=0 desired state with JTC.
        msg.header.stamp=self.node.get_clock().now().to_msg()
        for seconds,positions,velocities in points:
            p=rp.JointTrajectoryPoint();p.positions=positions;p.velocities=velocities
            p.time_from_start=rp.Duration(seconds=seconds).to_msg()
            msg.points.append(p)
        self.publisher.publish(msg)
        self.path=points;self.path_time=now

    def stop_stream(self, reason):
        self.ready=False;self.phase=reason+'；正在减速并保持'
        self.report(True)
        if self.path is not None:
            now=time.monotonic();q,v=sample(self.path,now-self.path_time)
            stop=braking(q,v)
            self.publish_points(stop,now)
            end=now+stop[-1][0]+.6
            self.pump(stop[-1][0]+.05)
            while max(abs(v) for v in self.hw['velocity']) > .005:
                if time.monotonic()>end:raise RuntimeError('松键后未确认关节停止，结束控制')
                self.pump()
            if max(abs(a-b) for a,b in zip(self.hw['command_position'],stop[-1][1])) > .004:
                raise RuntimeError('控制器未执行停止目标，结束控制')
            # Keep the original thirteen held axes, adopting only the active arm.
            for j in self.moving_joints:self.hold_positions[j-1]=self.hw['command_position'][j-1]
        self.path=None;self.moving_joints=None;self.stream_identity=None
        self.stationary_enabled()
        self.phase=reason+'；全部关节保持位置';self.ready=True;self.report(True)

    def run(self):
        self.moving_joints=None;self.path=None;self.stream_identity=None
        self.prepare()
        if self.args.check or self.args.prepare_check:return
        self.publisher=self.node.create_publisher(rp.JointTrajectory,
            f'/{rp.CONTROLLER}/joint_trajectory',1)
        end=time.monotonic()+3
        while not self.publisher.get_subscription_count():
            if time.monotonic()>end:raise RuntimeError('连续轨迹接口未就绪')
            self.pump()
        self.phase='全部关节已使能；长按一个方向键连续运动，松键停止'
        self.report(True)
        desired=[0.]*20;accepted=None;blocked=False;cancel_epoch=None
        while True:
            self.pump(PERIOD)
            with self.input_lock:packet=self.packet.copy()
            # Even a queued nonzero request must not continue while unfocused.
            request=self.wrist_request(packet if packet.get('focused') else {**packet, 'request': None})
            now=time.monotonic()
            if request is None:
                if self.path is not None:self.stop_stream('已松键' if packet.get('focused') else '窗口失焦，暂停运动')
                accepted=None;blocked=False;continue
            if blocked:continue
            try:
                if accepted is not None and packet.get('cancel',0)!=cancel_epoch:
                    raise ValueError('已停止，请松键后重按')
                if not isinstance(request,dict):raise ValueError('无效连续控制请求')
                if request.get('id') != (accepted or {}).get('id'):
                    if type(request.get('id')) is not int or request['id'] <= self.last_request:
                        raise ValueError('已丢弃旧控制请求，请松键后重按')
                    desired=validate_velocity(self.hw,request,now,previous=accepted)
                    identity=(request['arm'],request['axis'],request['direction'])
                    if self.stream_identity is not None and identity != self.stream_identity:
                        raise ValueError('切换方向或手臂前必须先松键')
                    if self.path is None:
                        self.stationary_enabled()
                        self.moving_joints=list(ARMS[request['arm']])
                        self.stream_identity=identity
                    accepted=request;self.last_request=request['id'];cancel_epoch=packet.get('cancel',0)
                if now-accepted['created'] > LEASE:raise ValueError('连续指令超过 0.25 秒未更新')
                if self.path is None:q,v=self.hold_positions[:],[0.]*20
                else:q,v=sample(self.path,now-self.path_time)
                points, scale = guarded_trajectory(self.wrist.guard, q, v, desired,
                                                   self.moving_joints, self.hold_positions)
                self.wrist.message = ('腕部接近边界：已减速 / 保持，可向内收腕'
                                      if scale < 1 else ('腕部联合限位已启用'
                                      if self.wrist.guard.enabled else '腕部联合限位已停用'))
                if self.path is None:self.emit(jog_started={'arm':accepted['arm'],'axis':accepted['axis'],
                    'direction':accepted['direction'],'joint_speed':accepted['joint_speed'],
                    'model':MODEL_ID,'tcp_definition':'flange_face_center',
                    'coordinate_frame':accepted['frame'],
                    'rotation_anchor':accepted.get('rotation_anchor')})
                self.publish_points(points,now)
                self.phase=('左臂' if accepted['arm']=='left' else '右臂')+' TCP 连续运动中；松键减速停止'
                self.ready=True
            except ValueError as exc:
                self.emit(jog_rejected=str(exc))
                self.stop_stream(str(exc));accepted=None;blocked=True


def guarded_trajectory(guard, q, v, desired, joints, held):
    """Retain the TCP IK velocity direction and reduce its speed as a whole."""
    scale = 1.
    for offset in (0, 7):
        projected = [a+b*.28 for a, b in zip(q[offset:offset+7], desired[offset:offset+7])]
        scale = min(scale, guard.approach_scale(q[offset:offset+7], projected))
    for factor in (1., .5, .25, .125, .0625, 0.):
        candidate = trajectory(q, v, [d*scale*factor for d in desired], joints, held)
        try:
            guard.trajectory(candidate)
            return candidate, scale*factor
        except WristBlocked:
            continue
    # The previously accepted curve included a complete braking tail. If a
    # replacement cannot fit, use minimum-acceleration braking from its state.
    stop = braking(q, v)
    guard.trajectory(stop)
    return stop, 0.


def load_ros():
    # Share the already-tested lifecycle/cleanup implementation, without
    # importing ROS on the laptop or executing robot_probe's standalone main.
    import rclpy
    import yaml
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter
    from rclpy.duration import Duration
    from hardware_state_broadcaster_msgs.msg import HardwareState
    from fairino_hardware_command_msgs.action import StringCommand
    from control_msgs.action import FollowJointTrajectory
    from control_msgs.msg import JointTolerance
    from trajectory_msgs.msg import JointTrajectoryPoint,JointTrajectory
    from controller_manager_msgs.srv import (ListControllers,LoadController,ConfigureController,
                                            SwitchController,UnloadController)
    from rcl_interfaces.srv import SetParameters
    for name,value in locals().copy().items():setattr(rp,name,value)


def main():
    parser=argparse.ArgumentParser()
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--check',action='store_true')
    mode.add_argument('--prepare-check',action='store_true')
    args=parser.parse_args()
    lock=open('/home/fairino/.fairino_upper_identify.lock','a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    load_ros();rp.rclpy.init();probe=TCPProbe(args)
    signal.signal(signal.SIGTERM,lambda *_:probe.exit.set())
    signal.signal(signal.SIGINT,lambda *_:probe.exit.set())
    code=0
    try:probe.run()
    except Exception as exc:code=1;probe.emit(error=str(exc))
    finally:
        if not probe.cleanup():code=1
        deadline=time.monotonic()+1
        while probe.output.unfinished_tasks and time.monotonic()<deadline:time.sleep(.01)
        probe.node.destroy_node();rp.rclpy.shutdown()
    return code


if __name__=='__main__':
    raise SystemExit(main())
