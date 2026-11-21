"""Exclusive A3 USB ownership, freshness at the USB writer, and Ruckig holds."""
import ctypes
import fcntl
import os
import sys
from pathlib import Path
import threading
import time

from retarget import LOCAL, Retarget
import ports as hand_ports
from a32_driver import A32Device, A32DriverError, NativeHand
from a32_driver.native import _SendTiming, MIT_GEAR_RATIOS, SDK_MIT_GEAR_RATIO
from online_control.online_motion_track.a32.control import A32Control

FRESH_NS = 200_000_000


class Held(BaseException):
    """Cancel this planner sample without upstream treating it as a motor fault."""


class RightDipBenchDevice(A32Device):
    """Bench-only zero-end allowance for the mounted right middle fingertip."""
    # Hand telemetry encodes the stroke as float32, including the boundary.
    _min_start_mm = ctypes.c_float(-0.30).value
    _reading_start_pose = False

    def starting_pose(self, fallback_pose=None):
        # Use the upstream single telemetry sample, validity checks and inversion.
        with self._lock:
            self._reading_start_pose = True
            try:
                return super().starting_pose(fallback_pose)
            finally:
                self._reading_start_pose = False

    def feedback_position(self, channel, value):
        position = super().feedback_position(channel, value)
        if (self._reading_start_pose and channel["joint"] == "right_hand_mid_DIP_joint"
                and channel["unit"] == "mm" and channel["limits"][0] == 0
                and self._min_start_mm <= ctypes.c_float(position).value < 0):
            return 0.0
        return position


class Client:
    def __init__(self):
        self.lock = threading.RLock()
        self.ready = False
        self.follow = False
        self.generation = 0
        self.heartbeat_until = 0
        self.ending = threading.Event()
        self.reason = ""
        self.errors = []

    def beat(self, sent_ns):
        with self.lock:
            now = time.monotonic_ns()
            if type(sent_ns) is not int or not 0 < sent_ns <= now:
                raise ValueError("心跳 sent_ns 必须是本机单调时钟的发送时间")
            until = sent_ns + FRESH_NS
            if until <= max(now, self.heartbeat_until):
                return  # Buffered or repeated heartbeats cannot revive a lease.
            if now >= self.heartbeat_until:
                self.generation += 1
            self.heartbeat_until = until

    def set_follow(self, enabled):
        with self.lock:
            if enabled and not self.ready:
                raise ValueError("手尚未就绪")
            if enabled != self.follow:
                self.generation += 1
                self.follow = enabled

    def stop(self, reason, error=False):
        with self.lock:
            if error:
                self.errors.append(reason)
            if not self.ending.is_set():
                self.reason = reason
                self.follow = False
                self.generation += 1
                self.ending.set()


class Lease:
    """Only a solved observation from the current follow interval may be written."""
    def __init__(self, client):
        self.client = client
        self.generation = -1
        self.frame_until = 0

    def deadline(self):
        c = self.client
        if self.generation == c.generation and c.follow and not c.ending.is_set():
            return min(self.frame_until, c.heartbeat_until)
        return 0


class Transport(NativeHand):
    def __init__(self, lease):
        super().__init__(LOCAL / "lib/liba32_native.so")
        self.lease = lease
        self._dll.dxhand_driver_off.argtypes = [ctypes.c_void_p]
        self._dll.dxhand_send_before.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),
            ctypes.c_int64, ctypes.POINTER(_SendTiming)]
        self.seed_batch = None

    def driver_off(self):
        self._call("dxhand_driver_off")

    def send_targets(self, own, mit):
        scaled = [p * ratio / SDK_MIT_GEAR_RATIO for p, ratio in zip(mit, MIT_GEAR_RATIOS, strict=True)]
        timing = _SendTiming()
        with self.lease.client.lock, self._lock:
            deadline = self.lease.deadline()
            if time.monotonic_ns() >= deadline:
                raise Held()
            result = self._dll.dxhand_send_before(
                self._handle, (ctypes.c_float * 18)(*own), (ctypes.c_float * 2)(*scaled),
                deadline, ctypes.byref(timing))
            if result == 2:
                raise Held()
            if not result:
                raise A32DriverError(self._text(self._dll.a32_last_error))
        return {name: getattr(timing, name) for name, _ in timing._fields_}

    # A32Control.arm uses a two-call seed write. Send it through the same gate.
    def set_targets(self, own, mit):
        self.seed_batch = (own, mit)

    def send_once(self):
        batch, self.seed_batch = self.seed_batch, None
        self.send_targets(*batch)

    def _startup_call(self, action, *args):
        with self.lease.client.lock:
            if self.lease.client.ending.is_set():
                raise InterruptedError("startup cancelled")
            return action(*args)

    def enable_all(self, on=True):
        return self._startup_call(super().enable_all, on)

    def home_slot(self, slot, iq_counts):
        return self._startup_call(super().home_slot, slot, iq_counts)

    def home_reset_slot(self, slot):
        return self._startup_call(super().home_reset_slot, slot)


class HandControl(A32Control):
    def __init__(self, solver, device, lease, side):
        self.lease = lease
        super().__init__(solver.pipeline.session.profile, side, "off", device=device, start=False)
        self.start_tracking()

    def freeze(self):
        with self.lock:
            self.lease.frame_until = 0
            if self.target_at or self.phase == "tracking":
                # Upstream publishes self.pose only after a successful device write.
                self._seed(self.pose)
                self.target = dict(self.pose)
                self.target_at = 0.
                self.hold()

    def accept(self, pose, revision, arrival, generation):
        c = self.lease.client
        with self.lock:
            with c.lock:
                now = time.monotonic_ns()
                if not c.follow or generation != c.generation or now >= min(c.heartbeat_until, arrival + FRESH_NS):
                    self.freeze()
                    return
                if now >= self.lease.deadline():
                    self.freeze()
                self.lease.generation = generation
                self.lease.frame_until = arrival + FRESH_NS
            try:
                if not self.armed:
                    if self.last_error:
                        raise RuntimeError(self.last_error)
                    self.arm(True)
                self.touch()
                self.push(pose, revision)
            except Held:
                self.freeze()

    def step(self, dt):
        with self.lock:
            if time.monotonic_ns() >= self.lease.deadline():
                self.freeze()
                return
            try:
                super().step(dt)
            except Held:
                self.freeze()
            if self.last_error:
                self.lease.client.stop(self.last_error, error=True)


PORT_STORE = LOCAL / "hand_ports.json"


def probe_side(path):
    """Connect read-only, return firmware hand_side (1/2) or None. No motor commands."""
    device = A32Device("right", backend=NativeHand(LOCAL / "lib/liba32_native.so"))
    try:
        device.connect_usb(path)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            firmware = device.get_state()["firmware"]
            if firmware.get("valid"):
                return firmware.get("hand_side")
            time.sleep(.02)
        return None
    finally:
        device.close()


def side_lock(side):
    directory = Path(os.environ.get("XDG_RUNTIME_DIR") or f"/tmp/dxhand-{os.getuid()}")
    directory.mkdir(mode=0o700, exist_ok=True)
    f = (directory / f"dxhand-{side}.lock").open("a")
    try:
        fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        f.close()
        raise RuntimeError(f"{side} 手已被另一进程占用") from None
    return f


class Hand:
    def __init__(self, side, client, *, allow_right_dip_offset=False):
        self.side = side
        self.lease = Lease(client)
        self.solver = Retarget(side)
        self.native = Transport(self.lease)
        device_type = RightDipBenchDevice if allow_right_dip_offset and side == "right" else A32Device
        self.device = device_type(side, model_id=self.solver.pipeline.session.profile.model_id,
                                  backend=self.native)
        self.control = HandControl(self.solver, self.device, self.lease, side)
        self.owned = False
        self.revision = None
        self.source_epoch = None
        self.status = dict(phase="connecting", driver_off_sent=False, disable_confirmed=False)

    def wait_state(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while not self.lease.client.ending.is_set():
            state = self.device.get_state()
            if not state["connected"] or state["last_error"]:
                raise RuntimeError(state["last_error"] or "USB disconnected")
            if predicate(state):
                return state
            if time.monotonic() >= deadline:
                raise TimeoutError(f"{self.side}: {description}")
            self.lease.client.ending.wait(.01)
        raise InterruptedError("startup cancelled")

    def connect(self, port):
        self.device.connect_usb(port)
        # The version/side reply is optional: upstream notes some firmware does
        # not answer the single query sent on connect (observed on the right
        # hand: answered once, then never on reconnects). Refuse a reported
        # mismatch; without a reply, trust the port the operator assigned.
        try:
            state = self.wait_state(lambda s: s["firmware"].get("valid"), 2, "固件手别超时")
        except TimeoutError:
            self.status["firmware_hand_side"] = None
            print(f"{self.side}: 固件未回复手别，按指定端口视为{self.side}手", file=sys.stderr, flush=True)
            return
        actual = state["firmware"]["hand_side"]
        if actual != {"right": 1, "left": 2}[self.side]:
            raise ValueError(f"{self.side}: firmware.hand_side={actual}")
        self.status["firmware_hand_side"] = actual
        serial = hand_ports.serial_of(port)
        if serial:
            hand_ports.remember(PORT_STORE, serial, self.side)

    def prepare(self):
        # The A3.2 homes itself at power-up (rotary swing slots DONE, linear
        # slots THETA). A host homeReset drops the swing slots to WAIT and
        # homeSlot does not bring them back (observed on the right hand,
        # 2026-11-20), so never reset/home here: require the power-up state.
        self.owned = True
        self.status["phase"] = "checking"
        try:
            self.wait_state(lambda s: s["ready"], 2, "未处于回零完成状态")
        except TimeoutError:
            state = self.device.get_state()
            homing = [m.get("homing") for m in state["self"]]
            raise RuntimeError(f"{self.side}: 手未处于上电回零完成状态（各电机 {homing}），请给手断电重启") from None
        self.control.thread.start()
        self.status["phase"] = "holding"

    def update(self, source):
        c = self.lease.client
        with c.lock:
            generation = c.generation
            active = c.follow and time.monotonic_ns() < c.heartbeat_until
        snapshot = source.latest(self.side)
        epoch = snapshot["source_metadata"]["source_epoch"]
        if epoch != self.source_epoch:
            self.control.freeze()
            self.solver.reset()
            self.source_epoch = epoch
            self.revision = None
        if not active or not snapshot["tracking"]:
            self.control.freeze()
            return
        key = generation, snapshot["revision"]
        if key == self.revision:
            return
        self.revision = key
        pose = self.solver.solve(snapshot["hand"])
        arrival = snapshot["raw_manus"]["host_arrival_ns"]
        if not source.commit_if_current(self.side, snapshot, lambda: self.control.accept(
                pose, snapshot["revision"], arrival, generation)):
            self.control.freeze()
