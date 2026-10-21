"""Offline ctypes access to the manufacturer's pure seven-axis FK/IK library."""
import ctypes as C
import math
from pathlib import Path
from tcp_core import (ALPHA,THETA,D_MM,BOUNDS_DEG,MODEL_ID,check_arm_limits,
                      forward_matrix,pose_error,target_matrix,wrist_to_flange,flange_to_wrist,multiply)
from tool_frames import tool_config, validate_runtime, for_arm, inverse_rigid

LIBRARY = (Path(__file__).resolve().parents[2]/'vendor/fairino/libkine_FR_7Axis.so')
D7 = C.c_double*7
M4 = (C.c_double*4)*4


class DH(C.Structure):
    _fields_ = [(k,D7) for k in ('a','d','alpha','beta','theta')]


class Limits(C.Structure):
    _fields_ = [('joints',(C.c_double*2)*7)]


class VendorKinematics:
    def __init__(self, path=LIBRARY, tools=None):
        self.tools = validate_runtime(tools if tools is not None else tool_config())
        self.lib = C.CDLL(str(path))
        self.lib.forwardKinematic.argtypes = [DH,DH,C.POINTER(C.c_double),C.c_int,
                                             C.POINTER(C.c_double*4),C.POINTER(C.c_double)]
        self.lib.forwardKinematic.restype = C.c_int
        self.lib.inverseKinematic.argtypes = [DH,DH,Limits,C.c_int,C.c_double,
            C.POINTER(C.c_double*4),C.POINTER(C.c_double),C.POINTER(C.c_double),C.POINTER(D7)]
        self.lib.inverseKinematic.restype = C.c_int
        self.dh = DH();self.compensation = DH();self.limits = Limits()
        self.dh.d[:] = [v/1000 for v in D_MM]
        self.dh.alpha[:] = ALPHA;self.dh.theta[:] = THETA
        for i,b in enumerate(BOUNDS_DEG):self.limits.joints[i][:] = b

    def forward(self,q):
        q = check_arm_limits(q)
        out = M4();arm = C.c_double()
        rc = self.lib.forwardKinematic(self.dh,self.compensation,
            D7(*map(math.degrees,q)),7,out,C.byref(arm))
        matrix = [list(row) for row in out]
        if rc != 0 or not all(math.isfinite(v) for row in matrix for v in row) or not math.isfinite(arm.value):
            raise ValueError(f'厂家正解失败，返回码 {rc}')
        mm,deg = pose_error(matrix,forward_matrix(q))
        if mm > .01 or deg > .01:
            raise ValueError('厂家正解与独立 ART7 模型不一致')
        return matrix,arm.value

    def inverse(self,matrix,arm_angle,reference):
        check_arm_limits(reference)
        out = D7(*([float('nan')]*7));allout = (D7*16)()
        rc = self.lib.inverseKinematic(self.dh,self.compensation,self.limits,7,
            arm_angle,M4(*[(C.c_double*4)(*r) for r in matrix]),
            D7(*map(math.degrees,reference)),out,allout)
        if rc != 0 or not all(math.isfinite(v) for v in out):
            raise ValueError(f'厂家逆解失败，返回码 {rc}；请换方向或减小步长')
        q = check_arm_limits([math.radians(v) for v in out])
        mm,deg = pose_error(matrix,self.forward(q)[0])
        if mm > .02 or deg > .01:
            raise ValueError('厂家逆解的末端误差过大')
        return q

    def verify(self,q):
        matrix,arm_angle = self.forward(q)
        restored = self.inverse(matrix,arm_angle,q)
        error = max(abs(math.degrees(a-b)) for a,b in zip(q,restored))
        if error > .01:raise ValueError('当前姿态正逆解不连续，拒绝使能')
        return {'matrix':matrix,'arm_angle':arm_angle,'roundtrip_deg':error}

    def forward_tcp(self,q,arm=None):
        wrist,arm_angle = self.forward(q)
        return multiply(wrist_to_flange(wrist), for_arm(self.tools, arm)),arm_angle

    def inverse_tcp(self,tcp,arm_angle,reference,arm=None):
        flange = multiply(tcp, inverse_rigid(for_arm(self.tools, arm)))
        q = self.inverse(flange_to_wrist(flange),arm_angle,reference)
        mm,deg = pose_error(tcp,self.forward_tcp(q,arm)[0])
        if mm > .02 or deg > .01:
            raise ValueError('所选 TCP 逆解校验失败')
        return q

    def verify_tcp(self,q,arm=None):
        matrix,arm_angle = self.forward_tcp(q,arm)
        restored = self.inverse_tcp(matrix,arm_angle,q,arm)
        error = max(abs(math.degrees(a-b)) for a,b in zip(q,restored))
        if error > .01:raise ValueError('所选 TCP 正逆解不连续，拒绝使能')
        return {'matrix':matrix,'arm_angle':arm_angle,'roundtrip_deg':error}

    def step(self,q,axis,direction,amount):
        matrix,arm_angle = self.forward_tcp(q)
        target = target_matrix(matrix,axis,direction,amount)
        out = self.inverse_tcp(target,arm_angle,q)
        if max(abs(math.degrees(a-b)) for a,b in zip(q,out)) > 2.0:
            raise ValueError('逆解要求单轴移动超过 2°，请减小 TCP 步长')
        return out
