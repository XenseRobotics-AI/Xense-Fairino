"""ART7 model math and independent receiver-side TCP request validation.

Lengths/angles are from the manufacturer's ART7 DH table. Joint bounds are
the narrower limits in the supplied 20260615 URDF, not measured hard stops.
No inverse solver lives here: IK comes exclusively from libkine_FR_7Axis.so.
"""
import math

MODEL_ID = 'ART7-R7-20260615-flange-x-106mm-v2'
# In the supplied wrist2_Link frame, the outer tool mounting face is x=-106 mm.
# Translate the TCP origin only; keep the established orientation-axis convention.
# This is the flange face center, not a gripper fingertip or a ground-aligned frame.
FLANGE_OFFSET_MM = (-106.0, 0.0, 0.0)
ARMS = {'left': tuple(range(1, 8)), 'right': tuple(range(8, 15))}
BOUNDS_DEG = ((-175.,175.),(-110.,110.),(-175.,175.),(-75.,110.),
              (-175.,175.),(-70.,70.),(-85.,85.))
D_MM = (187.,0.,290.,0.,310.,0.,0.)
ALPHA = (-math.pi/2,math.pi/2,-math.pi/2,math.pi/2,-math.pi/2,math.pi/2,0.)
THETA = (0.,0.,0.,0.,0.,math.pi/2,0.)
MAX_JOINT_STEP_DEG = 2.0
REFERENCE_TOL_DEG = .05


def finite_vector(value, count, name):
    if (not isinstance(value, (list, tuple)) or len(value) != count
            or any(type(x) not in (int,float) or not math.isfinite(x) for x in value)):
        raise ValueError('无效的'+name)
    return list(value)


def identity():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def multiply(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def rotation(axis, radians):
    c,s = math.cos(radians),math.sin(radians)
    t = identity()
    i,j = ((1,2),(2,0),(0,1))[axis]
    t[i][i]=t[j][j]=c; t[i][j]=-s; t[j][i]=s
    return t


def forward_matrix(q_rad):
    """Raw DH wrist origin, intentionally distinct from the user-facing TCP."""
    q = finite_vector(q_rad, 7, '七轴角度')
    t = identity()
    for v,d,a,offset in zip(q,D_MM,ALPHA,THETA):
        # Multiply by the standard DH transform directly. This remains an
        # independent stdlib FK; expanding the sparse rotation/translation
        # avoids fourteen generic 4x4 products per high-rate validation.
        c,s = math.cos(v+offset),math.sin(v+offset)
        ca,sa = math.cos(a),math.sin(a)
        for row in t[:3]:
            x,y,z,p = row
            lateral = -x*s+y*c
            row[:] = [x*c+y*s, lateral*ca+z*sa, -lateral*sa+z*ca, p+z*d]
    return t


def wrist_to_flange(wrist):
    flange = [row[:] for row in wrist]
    for i in range(3):
        flange[i][3] += sum(wrist[i][j]*FLANGE_OFFSET_MM[j] for j in range(3))
    return flange


def flange_to_wrist(flange):
    wrist = [row[:] for row in flange]
    for i in range(3):
        wrist[i][3] -= sum(flange[i][j]*FLANGE_OFFSET_MM[j] for j in range(3))
    return wrist


def tcp_forward_matrix(q_rad):
    return wrist_to_flange(forward_matrix(q_rad))


def matrix_pose(t):
    # R = Rz(yaw) Ry(pitch) Rx(roll); rotations displayed in degrees.
    pitch = math.atan2(-t[2][0], math.hypot(t[0][0],t[1][0]))
    if abs(math.cos(pitch)) < 1e-8:
        roll = 0.; yaw = math.atan2(-t[0][1],t[1][1])
    else:
        roll = math.atan2(t[2][1],t[2][2]); yaw = math.atan2(t[1][0],t[0][0])
    return [t[i][3] for i in range(3)] + [math.degrees(a) for a in (roll,pitch,yaw)]


def pose_matrix(pose):
    p = finite_vector(pose,6,'六维位姿')
    r,pitch,y = map(math.radians,p[3:])
    t = multiply(multiply(rotation(2,y),rotation(1,pitch)),rotation(0,r))
    for i in range(3):t[i][3] = p[i]
    return t


def pose_error(a,b):
    distance = math.sqrt(sum((a[i][3]-b[i][3])**2 for i in range(3)))
    trace = sum(a[i][k]*b[i][k] for i in range(3) for k in range(3))
    angle = math.degrees(math.acos(max(-1., min(1., (trace-1.)/2.))))
    return distance,angle


def check_arm_limits(q_rad):
    q = finite_vector(q_rad,7,'七轴角度')
    for i,(v,(lo,hi)) in enumerate(zip(q,BOUNDS_DEG)):
        if not lo <= math.degrees(v) <= hi:
            raise ValueError(f'手臂第 {i+1} 轴超出模型限制 [{lo:g}, {hi:g}]°')
    return q


def target_matrix(start, axis, direction, step):
    if type(axis) is not int or not 0 <= axis < 6 or type(direction) is not int or direction not in (-1,1):
        raise ValueError('无效的 TCP 方向')
    maximum = 2.0 if axis < 3 else 1.0
    if type(step) not in (int,float) or not math.isfinite(step) or not .1 <= step <= maximum:
        raise ValueError('平移步长范围 0.1–2 mm，旋转步长范围 0.1–1°')
    if axis < 3:
        t = [r[:] for r in start];t[axis][3] += direction*step
    else:
        # Rotate about a fixed input-frame axis through the CURRENT TCP point.
        t = multiply(rotation(axis-3,math.radians(direction*step)),start)
        for i in range(3):t[i][3] = start[i][3]
    return t


def validate_plan(hw, held, request):
    """Never trust client-supplied joint indices, durations or other-arm targets."""
    if request.get('model') != MODEL_ID or request.get('kind') != 'tcp_step':
        raise ValueError('TCP 请求或运动学模型不匹配')
    arm = request.get('arm')
    if arm not in ARMS:raise ValueError('只能选择左臂或右臂')
    indices = [j-1 for j in ARMS[arm]]
    reference = finite_vector(request.get('reference'),7,'解算参考角度')
    measured = [hw['position'][i] for i in indices]
    if max(abs(math.degrees(a-b)) for a,b in zip(reference,measured)) > REFERENCE_TOL_DEG:
        raise ValueError('姿态已变化，请依据新反馈重新点按')
    check_arm_limits(measured)
    q = check_arm_limits(request.get('target'))
    delta = max(abs(math.degrees(a-b)) for a,b in zip(q,measured))
    if delta > MAX_JOINT_STEP_DEG:
        raise ValueError('逆解要求单轴移动超过 2°，请减小 TCP 步长')
    expected = target_matrix(tcp_forward_matrix(reference), request.get('axis'),
                             request.get('direction'), request.get('step'))
    actual_target = tcp_forward_matrix(q)
    mm,deg = pose_error(expected,actual_target)
    if mm > .02 or deg > .01:
        raise ValueError('逆解目标未通过机器人端独立正解校验')
    speed = request.get('joint_speed',.5)
    if type(speed) not in (int,float) or not math.isfinite(speed) or not .1 <= speed <= 1.0:
        raise ValueError('关节最高速度范围为 0.1–1°/s')
    distance,angle = pose_error(tcp_forward_matrix(measured),actual_target)
    # Quintic peak velocity is 1.875 times average velocity. These are also
    # bounded by joint speed; Cartesian path is joint-interpolated, not linear.
    duration = max(.5,1.875*delta/speed,1.875*distance/2.,1.875*angle/.5)
    start = finite_vector(held,20,'二十轴保持目标');target=start[:]
    for i,v,now in zip(indices,q,measured):start[i]=now;target[i]=v
    return {'arm':arm,'joints':list(ARMS[arm]),'start':start,'target':target,
            'duration':duration,'joint_speed':speed,'tcp_start':matrix_pose(tcp_forward_matrix(measured)),
            'tcp_target':matrix_pose(actual_target),'axis':request['axis'],
            'direction':request['direction'],'step':request['step']}
