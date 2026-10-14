"""Atomic controller packets from the existing vendor SDK, plus relative mapping.

Unlike the legacy separate getters, missing controllers never reuse cached grip
or pose values from another packet. This receiver does not send device commands.
"""
import ctypes as C
import json
import math
import threading
import time
from collections import deque
import numpy as np
from lerobot.utils.rotation import Rotation
from paths import ROOT
from mounts import BASE_FROM_TELEOP, MAPPING_ID

TIMEOUT = .25
# PICO tracking (right, up, back) -> teleop (forward, left, up).
FRAME = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
POSE_FRAMES = {
    "first_person": FRAME,
    # Anatomical mirror: reaching forward makes the robot reach forward too.
    "mirror": np.diag([1., -1., 1.]) @ FRAME,
    # Face to face, preserve directions in the room rather than mirror depth.
    "mirror_same_direction": np.diag([-1., -1., 1.]) @ FRAME,
}


def controller_sides(mode="first_person", hand_mapping="auto"):
    """Robot arm -> physical controller, including all that hand's buttons."""
    if mode not in POSE_FRAMES or hand_mapping not in ("auto", "same", "crossed"):
        raise ValueError("未知遥操方向或手柄对应关系")
    crossed = hand_mapping == "crossed" or (hand_mapping == "auto" and mode != "first_person")
    return {"left": "right", "right": "left"} if crossed else {"left": "left", "right": "right"}


def rotation_mapping(mode, rotation_mode="auto"):
    if rotation_mode == "auto":
        return "first_person" if mode == "first_person" else "mirror"
    if rotation_mode not in POSE_FRAMES:
        raise ValueError("未知旋转映射")
    return rotation_mode


def decode_packet(raw):
    outer = json.loads(raw)
    value = outer.get("value")
    if isinstance(value, str):
        value = json.loads(value)
    if not isinstance(value, dict) or "Controller" not in value:
        return None
    stamp = value.get("timeStampNs")
    if type(stamp) not in (int, float) or not math.isfinite(stamp) or stamp <= 0:
        raise ValueError("头显时间戳无效")
    result = {"stamp": stamp}
    for side in ("left", "right"):
        controller = value["Controller"].get(side)
        if not isinstance(controller, dict):
            raise ValueError(f"本帧缺少 {side} 手柄")
        for key in ("isTracked", "isActive", "connected", "isConnected"):
            if key in controller and not controller[key]:
                raise ValueError(f"{side} 手柄 {key}=false")
        pose = controller.get("pose")
        if isinstance(pose, str):
            pose = [float(x) for x in pose.split(",")]
        if (not isinstance(pose, list) or len(pose) != 7
                or not all(type(x) in (int, float) and math.isfinite(x) for x in pose)
                or not .9 < math.sqrt(sum(x*x for x in pose[3:])) < 1.1):
            raise ValueError(f"{side} 手柄姿态无效")
        grip = controller.get("grip")
        if type(grip) not in (int, float) or not math.isfinite(grip) or not 0 <= grip <= 1:
            raise ValueError(f"{side} Grip 无效")
        axes = [controller.get(k, 0.) for k in ("axisX", "axisY")]
        if any(type(v) not in (int, float) or not math.isfinite(v) or abs(v) > 1.001 for v in axes):
            raise ValueError(f"{side} 摇杆数据无效")
        click = controller.get("axisClick", False)
        if type(click) is not bool:
            raise ValueError(f"{side} 摇杆按压数据无效")
        result[side] = {"pose": pose, "grip": grip,
            "primary": bool(controller.get("primaryButton", False)),
            "secondary": bool(controller.get("secondaryButton", False)),
            "axis_x": float(axes[0]), "axis_y": float(axes[1]), "axis_click": click,
            "joystick_available": all(k in controller for k in ("axisX", "axisY", "axisClick"))}
    return result


class Stream:
    def __init__(self):
        self.lock = threading.Lock()
        self.frame = None
        self.pending = deque(maxlen=128)
        self.changed = 0.
        self.stamp = None
        self.serial = None
        self.error = "尚未收到双手柄数据，请勾选 Controller 和 Send"
        self.lib = C.CDLL(str(ROOT / "src/lerobot/teleoperators/pico4/xensevr-pc-service-pybind/lib/libPXREARobotSDK.so"))
        class State(C.Structure):
            _fields_ = [("devID", C.c_char*32), ("stateJson", C.c_char*16352)]
        self.State = State
        callback_type = C.CFUNCTYPE(None, C.c_void_p, C.c_int, C.c_int, C.c_void_p)
        self.callback = callback_type(self._callback)
        self.lib.PXREAInit.argtypes = [C.c_void_p, callback_type, C.c_uint]
        self.lib.PXREAInit.restype = C.c_int
        self.lib.PXREADeinit.argtypes = []
        self.lib.PXREADeinit.restype = C.c_int
        self.opened = False

    def _callback(self, context, kind, status, pointer):
        try:
            if kind in (1 << 3, 1 << 5):
                with self.lock:
                    self.frame = None
                    self.pending.clear()
                    self.error = "头显或 PC Service 已断开"
            elif kind == 1 << 25:
                state = C.cast(pointer, C.POINTER(self.State)).contents
                frame = decode_packet(state.stateJson)
                if frame is None:
                    return
                serial = bytes(state.devID).decode()
                with self.lock:
                    if self.serial is not None and serial != self.serial:
                        self.frame = None
                        self.error = "检测到多台头显，拒绝混用控制数据"
                        return
                    self.serial = serial
                    if frame["stamp"] != self.stamp:
                        self.stamp = frame["stamp"]
                        self.changed = time.monotonic()
                        frame["received_monotonic"] = self.changed
                        self.frame = frame
                        self.pending.append(frame)
                        self.error = ""
        except Exception as exc:
            with self.lock:
                self.frame = None
                self.pending.clear()
                self.error = str(exc)

    def open(self):
        if self.lib.PXREAInit(None, self.callback, 0xffffffff) != 0:
            raise RuntimeError("PICO SDK 初始化失败")
        self.opened = True

    def read(self):
        with self.lock:
            if self.frame is None:
                raise ValueError(self.error)
            if time.monotonic()-self.changed > TIMEOUT:
                raise ValueError("双手柄数据超过 0.25 秒未更新")
            result = {**self.frame, "samples": list(self.pending)}
            self.pending.clear()
            return result

    def close(self):
        if self.opened:
            self.lib.PXREADeinit()
            self.opened = False


def mapped_pose(pose, side, mode="first_person", rotation_mode=None):
    """Express tracking poses in a side's axes for relative differencing."""
    pose = np.asarray(pose, dtype=float)
    frame = BASE_FROM_TELEOP[side] @ POSE_FRAMES[mode]
    orientation_frame = BASE_FROM_TELEOP[side] @ POSE_FRAMES[rotation_mode or mode]
    return (frame @ pose[:3]*1000.,
            orientation_frame @ Rotation.from_quat(pose[3:]).as_matrix() @ orientation_frame.T)


class Clutch:
    """Grip references the current flange, never a virtual origin or old goal."""
    def __init__(self, side, mode="first_person", rotation_mode=None):
        if side not in BASE_FROM_TELEOP:
            raise ValueError(f"未知机械臂：{side}")
        self.side = side
        if mode not in POSE_FRAMES:
            raise ValueError(f"未知遥操方向：{mode}")
        self.mode = mode
        self.rotation_mode = rotation_mode or mode
        if self.rotation_mode not in POSE_FRAMES:
            raise ValueError("未知旋转映射")
        self.engaged = False
        self.reference = None
        self.start = None
        self.previous = None
        self.frozen_position = self.frozen_rotation = False

    def reset(self):
        self.engaged = False
        self.reference = self.start = self.previous = None
        self.frozen_position = self.frozen_rotation = False

    def target(self, controller, current_tcp, sensitivity, stationary=True,
               freeze_position=False, freeze_rotation=False):
        position, rotation = mapped_pose(controller["pose"], self.side, self.mode, self.rotation_mode)
        if controller["grip"] <= .3:
            self.reset()
            return None
        if not self.engaged:
            if controller["grip"] <= .5 or not stationary:
                return None
            self.engaged = True
            self.reference = (position.copy(), rotation.copy())
            self.start = np.array(current_tcp, dtype=float)
        # Freeze at the current commanded flange, discarding pending targets in
        # this component. Rebase throughout the hold AND on its release edge so
        # movement made while frozen can never be replayed after release.
        if freeze_position and not self.frozen_position:
            self.start[:3, 3] = np.asarray(current_tcp)[:3, 3]
        if freeze_rotation and not self.frozen_rotation:
            self.start[:3, :3] = np.asarray(current_tcp)[:3, :3]
        if freeze_position or self.frozen_position:
            self.reference = (position.copy(), self.reference[1])
        if freeze_rotation or self.frozen_rotation:
            self.reference = (self.reference[0], rotation.copy())
        self.frozen_position = bool(freeze_position)
        self.frozen_rotation = bool(freeze_rotation)
        self.previous = position.copy()
        target = self.start.copy()
        target[:3, 3] += sensitivity*(position-self.reference[0])
        delta = Rotation.from_matrix(rotation @ self.reference[1].T).as_rotvec()
        target[:3, :3] = Rotation.from_rotvec(sensitivity*delta).as_matrix() @ self.start[:3, :3]
        return target
