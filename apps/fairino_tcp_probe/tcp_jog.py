"""Held-key velocity requests and finite, braking joint trajectories (no ROS)."""
import math
from tcp_core import (ARMS, MODEL_ID, BOUNDS_DEG, finite_vector, check_arm_limits,
                      tcp_forward_matrix, target_matrix, pose_error)
from tcp_frames import CHEST_FRAME_ID, chest_target_in_base

JOG_KIND = 'tcp_hold_chest_v2'
PERIOD = .02
LOOKAHEAD = .08
BRAKE_TIME = .20
LEASE = .25
IK_INTERVAL = .10
MAX_SPEED = 4.0
DEFAULT_SPEED = 2.0
MAX_LINEAR = 8.0
MAX_ANGULAR = 2.0
ACCEL_DEG = 20.0
ROTATION_HOLD_GAIN = 2.0  # 1/s; bounded correction avoids amplifying delayed feedback.
KEYS = {'w':(0,1),'s':(0,-1),'a':(1,1),'d':(1,-1),'r':(2,1),'f':(2,-1),
        'i':(3,1),'k':(3,-1),'j':(4,1),'l':(4,-1),'u':(5,1),'o':(5,-1)}
KEY_PAIRS = (('s','w'),('d','a'),('f','r'),('k','i'),('l','j'),('o','u'))


def speed_value(value, maximum, label):
    if type(value) not in (int,float) or not math.isfinite(value) or not .1 <= value <= maximum:
        raise ValueError(f'{label}必须在 0.1–{maximum:g} 之间')
    return value


def make_request(kin, hw, arm, axis, direction, linear, angular, joint_speed, rotation_anchor=None):
    """Vendor IK gives a local joint velocity direction; receiver rechecks FK."""
    speed_value(joint_speed,MAX_SPEED,'关节速度')
    speed_value(linear,MAX_LINEAR,'平移速度')
    speed_value(angular,MAX_ANGULAR,'旋转速度')
    reference = [hw['command_position'][j-1] for j in ARMS[arm]]
    matrix,arm_angle = kin.forward_tcp(reference)
    # The existing target constructor requires >= .1 units. dt records the
    # intended velocity even at low requested speeds; actual joint rate is capped.
    rate = linear if axis < 3 else angular
    amount = max(.1,rate*IK_INTERVAL)
    desired = chest_target_in_base(arm,matrix,axis,direction,amount)
    anchor = None
    if axis >= 3:
        anchor = finite_vector(rotation_anchor if rotation_anchor is not None else
                               [row[3] for row in matrix[:3]],3,'旋转法兰保持点')
        if math.dist(anchor,[row[3] for row in matrix[:3]]) > .5:
            raise ValueError('法兰中心偏离本次旋转保持点超过 0.5 mm，请松键后检查')
        gain = min(1.,ROTATION_HOLD_GAIN*amount/rate)
        for i in range(3):desired[i][3] += gain*(anchor[i]-matrix[i][3])
    target = kin.inverse_tcp(desired,arm_angle,reference)
    return {'kind':JOG_KIND,'model':MODEL_ID,'frame':CHEST_FRAME_ID,'arm':arm,'axis':axis,'direction':direction,
            'step':amount,'rate':rate,'reference':reference,'target':target,
            'joint_speed':joint_speed,'rotation_anchor':anchor}


def validate_velocity(hw, request, now, previous=None):
    if not isinstance(request,dict) or request.get('kind') != JOG_KIND or request.get('model') != MODEL_ID:
        raise ValueError('连续控制协议或模型不匹配')
    if request.get('frame') != CHEST_FRAME_ID:
        raise ValueError('胸部坐标系版本不匹配')
    if type(request.get('id')) is not int or request['id'] <= 0:
        raise ValueError('无效的连续控制序号')
    created = request.get('created')
    if type(created) not in (int,float) or not math.isfinite(created) or not 0 <= now-created <= LEASE:
        raise ValueError('连续控制指令过期，已停止；请松键后重按')
    arm = request.get('arm')
    if arm not in ARMS:raise ValueError('只能控制左臂或右臂')
    reference = check_arm_limits(request.get('reference'))
    target = check_arm_limits(request.get('target'))
    speed = speed_value(request.get('joint_speed'),MAX_SPEED,'关节速度')
    actual = [hw['command_position'][j-1] for j in ARMS[arm]]
    # At 4 deg/s a 250 ms lease can span 1 degree. Freshness and FK are
    # checked separately; this is only the stale-reference rejection bound.
    if max(abs(math.degrees(a-b)) for a,b in zip(actual,reference)) > 1.1:
        raise ValueError('解算参考位置过期，已停止；请松键后重按')
    start = tcp_forward_matrix(reference)
    expected = chest_target_in_base(arm,start,request.get('axis'),
                             request.get('direction'),request.get('step'))
    rate = speed_value(request.get('rate'),MAX_LINEAR if request['axis']<3 else MAX_ANGULAR,'TCP 速度')
    if request['axis'] >= 3:
        anchor = finite_vector(request.get('rotation_anchor'),3,'旋转法兰保持点')
        if math.dist(anchor,[row[3] for row in start[:3]]) > .5:
            raise ValueError('法兰中心偏离旋转保持点超过 0.5 mm')
        if previous is not None:
            if anchor != previous.get('rotation_anchor'):
                raise ValueError('连续旋转期间不能更改法兰保持点')
        elif math.dist(anchor,[row[3] for row in start[:3]]) > .05:
            raise ValueError('首次旋转保持点必须与当前法兰中心一致')
        gain = min(1.,ROTATION_HOLD_GAIN*request['step']/rate)
        for i in range(3):expected[i][3] += gain*(anchor[i]-start[i][3])
    mm,deg = pose_error(expected,tcp_forward_matrix(target))
    if mm > .02 or deg > .01:raise ValueError('连续控制未通过机器人端独立正解校验')
    if max(abs(math.degrees(a-b)) for a,b in zip(reference,target)) > 2.:
        raise ValueError('逆解跳变超过 2°，已停止；请减小速度或换方向')
    duration = request['step']/rate
    velocity = [(b-a)/duration for a,b in zip(reference,target)]
    peak = max(abs(math.degrees(v)) for v in velocity)
    scale = min(1.,speed/peak) if peak else 1.
    result = [0.]*20
    for j,v in zip(ARMS[arm],velocity):result[j-1] = v*scale
    return result


def sample(points, elapsed):
    """Exact cubic Hermite sampling, matching the JTC position+velocity fields."""
    if elapsed <= 0:return points[0][1][:],points[0][2][:]
    for (t0,p0,v0),(t1,p1,v1) in zip(points,points[1:]):
        if elapsed <= t1:
            dt=t1-t0;u=(elapsed-t0)/dt
            p=[a+(-2*u**3+3*u**2)*(b-a)+(u**3-2*u**2+u)*dt*c+
               (u**3-u**2)*dt*d for a,b,c,d in zip(p0,p1,v0,v1)]
            v=[(-6*u*u+6*u)/dt*(b-a)+(3*u*u-4*u+1)*c+
               (3*u*u-2*u)*d for a,b,c,d in zip(p0,p1,v0,v1)]
            return p,v
    return points[-1][1][:],[0.]*20


def braking(q, velocity):
    duration=max(.02,max(abs(v) for v in velocity)/math.radians(ACCEL_DEG))
    return [(0.,q[:],velocity[:]),
            (duration,[p+v*duration*.5 for p,v in zip(q,velocity)],[0.]*20)]


def trajectory(q, velocity, desired, joints, held):
    """80 ms velocity ramp followed by a 200 ms fail-safe braking tail.

    Endpoints use the trapezoid integral, so JTC cubic interpolation has
    exactly linear velocity, with no hidden velocity overshoot. A fresh
    trajectory replaces this one every 20 ms while the key is held.
    """
    q=finite_vector(q,20,'轨迹位置');velocity=finite_vector(velocity,20,'轨迹速度')
    desired=finite_vector(desired,20,'目标速度')
    active=set(joints);v1=[];p1=[];p2=[]
    dv=math.radians(ACCEL_DEG)*LOOKAHEAD
    for i,(p,v,d) in enumerate(zip(q,velocity,desired)):
        if i+1 not in active:
            q[i]=held[i];velocity[i]=0.;v1.append(0.);p1.append(held[i]);p2.append(held[i]);continue
        if max(abs(v),abs(d)) > math.radians(MAX_SPEED)+1e-9:
            raise ValueError('轨迹速度超出 4°/s')
        new=v+max(-dv,min(dv,d-v))
        v1.append(new);p1.append(p+(v+new)*.5*LOOKAHEAD)
        p2.append(p1[-1]+new*.5*BRAKE_TIME)
    points=[(0.,q,velocity),(LOOKAHEAD,p1,v1),(LOOKAHEAD+BRAKE_TIME,p2,[0.]*20)]
    # Check not just endpoints: reversal can have an interior position extremum.
    for arm,indices in ARMS.items():
        if active.intersection(indices):
            for left,right in zip(points,points[1:]):
                t0,a,v0=left;t1,b,v1=right
                for joint,(lo,hi) in zip(indices,BOUNDS_DEG):
                    i=joint-1;values=[a[i],b[i]]
                    if v0[i]*v1[i] < 0:
                        t=-v0[i]*(t1-t0)/(v1[i]-v0[i])
                        values.append(a[i]+v0[i]*t+.5*(v1[i]-v0[i])/(t1-t0)*t*t)
                    if any(not lo <= math.degrees(value) <= hi for value in values):
                        raise ValueError('已接近模型关节限位，停止并保持；请换方向')
    return points
