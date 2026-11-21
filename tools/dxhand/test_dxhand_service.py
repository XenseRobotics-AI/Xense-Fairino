"""Subprocess and real liba32_native tests against USB PTYs and synthetic Manus."""
import ctypes
import json
import os
from pathlib import Path
import pty
import queue
import select
import signal
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest

from unittest.mock import patch

from check_retarget import ROOT, sample

sys.path.insert(0, str(ROOT / "scripts/dxhand"))
import dxhand_service
from dxhand_service import Client, command
from hand import Hand, Lease, Transport, probe_side
from online_control.online_motion_track.pipeline_v3_2.pipeline import observation_depth_source
from a32_driver.native import _SendTiming


def frame(function, payload):
    data = b"\xaa\x55" + struct.pack("<HBB", len(payload), 3, function) + payload
    crc = 0xffff
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0xa001 if crc & 1 else 0)
    return data + struct.pack("<H", crc)


OFF = frame(0x21, b"\xff\xff\x02\x03")


class USB:
    def __init__(self, side=2, seek=True, homing=2):
        self.master, self.slave = pty.openpty()
        os.set_blocking(self.master, False)
        self.path = os.ttyname(self.slave)
        self.side = side
        self.seek = seek
        self.initial_homing = homing
        self.raw_positions = {}
        self.closed = False
        self.frames = []
        self.done = threading.Event()
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
        pending = bytearray()
        live = False
        homing = self.initial_homing  # Power-up DONE, retained after driver-off.
        homed_at = 0.
        sequence = 0
        while not self.done.is_set():
            if select.select([self.master], [], [], .006)[0]:
                pending.extend(os.read(self.master, 4096))
            while len(pending) >= 6:
                size = int.from_bytes(pending[2:4], "little") + 8
                if len(pending) < size:
                    break
                wire = bytes(pending[:size])
                del pending[:size]
                assert frame(wire[5], wire[6:-2]) == wire
                self.frames.append(wire)
                if wire[5] == 1:
                    os.write(self.master, frame(1, bytes([1, 0, 1, 2, 3, 4, self.side, 0])))
                    live = True
                if wire[5] == 0x22:
                    if wire[8] == 2:
                        homing = 0
                    else:
                        homing = 1 if self.seek else 2
                        homed_at = time.monotonic() + .1
                if wire == OFF:
                    live = False
            if live:
                if homing == 1 and time.monotonic() >= homed_at:
                    homing = 2
                sequence = (sequence + 1) % 65536
                data = bytearray(484)
                struct.pack_into("<HBBII", data, 0, sequence, 3, 0, sequence, 0x3ffff)
                for i in range(18):
                    data[15 + 24 * i] = homing
                    struct.pack_into("<f", data, 16 + 24 * i, self.raw_positions.get(i, 0.0))
                for i in range(2):
                    struct.pack_into("<IHH", data, 446 + 20 * i, 0x80000000, 32768, 32768)
                # A service that refused this hand no longer drains telemetry.
                if select.select([], [self.master], [], 0)[1]:
                    try:
                        os.write(self.master, frame(2, data))
                    except BlockingIOError:
                        pass

    def targets(self):
        return [self.output_values(w) for w in self.frames if w[5] == 0x20]

    @staticmethod
    def output_values(wire):
        values = list(struct.unpack_from("<20f", wire, 7))
        # MIT wire angles are motor-shaft degrees with the SDK's 50:1 gearing.
        values[18:] = [value / 50 for value in values[18:]]
        return values

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.done.set()
        self.thread.join(2)
        assert not self.thread.is_alive()
        os.close(self.master)
        os.close(self.slave)


class Manus:
    def __init__(self, mode):
        self.mode = Path(mode)
        self.arrival = time.monotonic_ns()

    def latest(self, side):
        mode = self.mode.read_text()
        if mode == "error":
            raise RuntimeError("fake Manus failed")
        if mode != "stale":
            self.arrival = time.monotonic_ns() // 16_000_000 * 16_000_000
        return dict(revision=self.arrival, tracking=True, source_metadata={"source_epoch": 0},
                    hand=sample(side, 15), raw_manus={"host_arrival_ns": self.arrival})

    def commit_if_current(self, side, snap, action):
        if time.monotonic_ns() - snap["raw_manus"]["host_arrival_ns"] >= 200_000_000:
            return False
        action()
        return True

    def close(self, *, wait=True):
        pass


class Process:
    def __init__(self, ports, directory, bench=False, extra_args=()):
        self.statuses = queue.Queue()
        self.results = []
        self.mode = Path(directory) / "manus"
        self.mode.write_text("fresh")
        self.err = tempfile.TemporaryFile(mode="w+")
        self.keyboard = None
        stdin = subprocess.PIPE
        if bench:
            self.keyboard, stdin = pty.openpty()
        self.p = subprocess.Popen(
            [sys.executable, "-B", __file__, "serve", json.dumps(ports), str(self.mode), str(int(bench)), *extra_args],
            stdin=stdin, stdout=subprocess.PIPE, stderr=self.err, text=True,
            env=dict(os.environ, XDG_RUNTIME_DIR=directory))
        if bench:
            os.close(stdin)
        self.reader = threading.Thread(target=self.read)
        self.reader.start()

    def read(self):
        for line in self.p.stdout:
            value = json.loads(line)
            self.results.append(value)
            self.statuses.put(value)
        self.statuses.put(None)

    def until(self, predicate):
        while True:
            status = self.statuses.get(timeout=8)
            if status is None or status["event"] == "stopped" and not predicate(status):
                self.err.seek(0)
                raise AssertionError(f"early exit: {self.results[-1:]} {self.err.read()}")
            if predicate(status):
                return status

    def send(self, data):
        self.p.stdin.write(data + "\n")
        self.p.stdin.flush()

    def final(self, code=0):
        status = self.until(lambda s: s["event"] == "stopped")
        assert self.p.wait(timeout=3) == code, status
        return status

    def close(self):
        if self.p.poll() is None:
            self.p.terminate()
            self.p.wait(timeout=5)
        self.reader.join(2)
        if self.p.stdin:
            self.p.stdin.close()
        if self.keyboard:
            os.close(self.keyboard)
        self.p.stdout.close()
        self.err.close()


class ServiceTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = directory.name

    def usb(self, side=2, seek=True, homing=2):
        device = USB(side, seek, homing)
        self.addCleanup(device.close)
        return device

    def start(self, ports, bench=False, extra_args=()):
        proc = Process(ports, self.directory, bench, extra_args)
        self.addCleanup(proc.close)
        return proc

    def ready(self, usb, **options):
        proc = self.start({"left": usb.path}, **options)
        status = proc.until(lambda s: s["ready"])
        self.assertFalse(status["follow"])
        self.assertEqual(set(status["hands"]), {"left"})
        self.assertFalse(usb.targets())
        return proc

    def test_profile_and_metric_manus_observation_on_both_hands(self):
        for side in ("left", "right"):
            with self.subTest(side=side):
                hand = Hand(side, Client())
                self.addCleanup(hand.control.close)
                solver = hand.solver
                pipeline = solver.pipeline
                self.assertEqual(pipeline.session.profile.model_id, "TsenseI_v3_2")
                self.assertEqual(pipeline.stroke_mapper.model_id, pipeline.session.profile.model_id)
                self.assertEqual(hand.device.model_id, pipeline.session.profile.model_id)
                self.assertEqual(hand.device.mapper.model_id, hand.device.model_id)
                with patch.object(pipeline, "process", wraps=pipeline.process) as process:
                    pose = solver.solve(sample(side, 15))
                observation, = process.call_args.args
                self.assertEqual((observation.source, observation_depth_source(observation)),
                                 ("manus", "manus_metric"))
                self.assertEqual(len(pose), 20)
                self.assertTrue(all(name.startswith(side + "_hand_") for name in pose))

    def test_probe_reads_firmware_with_installed_profile(self):
        usb = self.usb(side=1)
        self.assertEqual(probe_side(usb.path), 1)
        self.assertEqual([wire[5] for wire in usb.frames], [1])

    def test_hold_heartbeat_manus_and_resume(self):
        usb = self.usb()
        proc = self.ready(usb)

        def beat():
            proc.send(json.dumps({"cmd": "heartbeat", "sent_ns": time.monotonic_ns()}))

        def wait_motion():
            initial = len(usb.targets())
            for _ in range(35):
                beat()
                time.sleep(.025)
                if len(usb.targets()) >= initial + 8:
                    return
            self.fail(f"no motion: {proc.results[-2:]}")

        def quiet(duration=.09):
            previous = usb.targets()
            self.assertTrue(previous)
            time.sleep(duration)
            self.assertEqual(usb.targets(), previous)
            return previous

        proc.send('{"cmd":"follow","enabled":true}')
        wait_motion()
        proc.send('{"cmd":"follow","enabled":false}')
        proc.until(lambda s: not s["follow"] and s["ready"])
        stopped = quiet()
        proc.send('{"cmd":"follow","enabled":true}')
        wait_motion()
        # A stationary fake encoder always reads zero. Resume at the last
        # accepted command and begin a new zero-velocity Ruckig trajectory.
        import numpy as np
        self.assertGreater(np.max(np.abs(stopped[-1])), .001)
        np.testing.assert_allclose(usb.targets()[len(stopped)], stopped[-1], atol=.002)
        time.sleep(.23)
        quiet()
        proc.send(json.dumps({"cmd": "heartbeat", "sent_ns": time.monotonic_ns() - 210_000_000}))
        quiet()  # A heartbeat delayed in the pipe cannot restart the hand.
        wait_motion()
        proc.mode.write_text("stale")
        for _ in range(10):
            beat()
            time.sleep(.03)
        stopped = quiet(.05)
        proc.mode.write_text("fresh")
        wait_motion()
        np.testing.assert_allclose(usb.targets()[len(stopped)], stopped[-1], atol=.002)
        proc.send('{"cmd":"stop"}')
        self.assert_off(proc, usb)

    def assert_off(self, proc, usb, code=0):
        final = proc.final(code)
        self.assertTrue(final["hands"]["left"]["driver_off_sent"])
        self.assertFalse(final["hands"]["left"]["disable_confirmed"])
        self.assertIn("物理下使能需现场核验", final["message"])
        self.assertEqual(usb.frames[-1], OFF)
        return final

    def test_restart_after_driver_off_and_exit_paths(self):
        usb = self.usb()
        for event in ("eof", "SIGTERM", "bad JSON", "exception"):
            with self.subTest(event=event):
                begin = len(usb.frames)
                proc = self.ready(usb)
                commands = [w[8] for w in usb.frames[begin:] if w[5] == 0x22]
                self.assertEqual(commands, [])  # Power-up homing only; startup never resets or homes.
                if event == "eof":
                    proc.p.stdin.close()
                elif event == "SIGTERM":
                    proc.p.send_signal(signal.SIGTERM)
                elif event == "bad JSON":
                    proc.send("{")
                else:
                    proc.mode.write_text("error")
                self.assert_off(proc, usb, int(event in ("bad JSON", "exception")))

    def test_not_homed_at_power_up_is_refused_without_commands(self):
        usb = self.usb(homing=0)
        proc = self.start({"left": usb.path})
        self.assertIn("重启", str(proc.final(1)["errors"]))
        self.assertFalse([w for w in usb.frames if w[5] in (0x20, 0x22)])

    def test_mismatch_and_unset_prevent_dual_hand_start(self):
        left = self.usb()
        for wrong in (0, 2):
            right = self.usb(wrong)
            proc = self.start({"left": left.path, "right": right.path})
            self.assertIn("firmware.hand_side", str(proc.final(1)["errors"]))
            self.assertTrue(all(w[5] == 1 for w in left.frames + right.frames))

    def test_per_hand_lock_and_bench(self):
        usb = self.usb()
        proc = self.ready(usb, bench=True)
        competing = self.start({"left": usb.path})
        self.assertIn("占用", str(competing.final(1)["errors"]))
        os.write(proc.keyboard, b" ")
        proc.until(lambda s: s["hands"]["left"]["phase"] == "tracking")
        os.write(proc.keyboard, b" q")
        self.assert_off(proc, usb)

    def test_bench_unusable_dip_feedback_falls_back_to_software_pose(self):
        # Like upstream A32Control.arm: feedback that is missing or out of range
        # falls back to the current software pose instead of refusing to start.
        option = ("--allow-right-dip-offset",)
        for args, slot, stroke in (
                ((), 8, -.29),
                (option, 8, -.30),
                ((), 8, 12.6),
                ((), 9, -.29)):
            with self.subTest(args=args, slot=slot, stroke=stroke):
                usb = self.usb(side=1)
                usb.raw_positions = {slot: -stroke, 1: -1.0}
                proc = self.start({"right": usb.path}, bench=True, extra_args=args)
                proc.until(lambda s: s["ready"])
                os.write(proc.keyboard, b" ")
                proc.until(lambda s: s["hands"]["right"]["phase"] == "tracking")
                first = usb.targets()[0]
                self.assertAlmostEqual(first[1], 1.0, places=3)
                os.write(proc.keyboard, b"q")
                self.assertFalse(proc.final()["errors"])

    def test_unplug_reports_failed_broadcast(self):
        usb = self.usb(1)
        proc = self.start({"right": usb.path})
        proc.until(lambda s: s["ready"])
        usb.close()
        status = proc.final(1)
        self.assertFalse(status["hands"]["right"]["driver_off_sent"])
        self.assertFalse(status["hands"]["right"]["disable_confirmed"])
        self.assertTrue(status["errors"])

    def test_direct_done_after_reset_follows_upstream_criterion(self):
        # Real firmware may report DONE/THETA without a visible SEEK. Upstream
        # A32Device accepts that after >=0.3 s and a fresh telemetry frame.
        usb = self.usb(seek=False)
        proc = self.start({"left": usb.path})
        proc.until(lambda s: s["ready"])
        proc.send('{"cmd":"stop"}')
        status = proc.final(0)
        self.assertTrue(status["hands"]["left"]["driver_off_sent"])

    def test_expired_native_write_is_silent(self):
        usb = self.usb()
        transport = Transport(Lease(Client()))
        self.addCleanup(transport.close)
        transport.connect_usb(usb.path)
        # Bypass the outer Python gate to exercise the inner USB deadline.
        result = transport._dll.dxhand_send_before(
            transport._handle, (ctypes.c_float * 18)(), (ctypes.c_float * 2)(),
            time.monotonic_ns() - 1, ctypes.byref(_SendTiming()))
        self.assertEqual(result, 2)
        transport.driver_off()
        deadline = time.monotonic() + 1
        while OFF not in usb.frames and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertEqual([wire[5] for wire in usb.frames], [1, 0x21])

    def test_strict_command_schema(self):
        for line in (b'null', b'[]', b'{"cmd":"follow","enabled":1}', b'{"cmd":"stop","x":0}',
                     b'{"cmd":"stop","cmd":"heartbeat"}', b'\xff'):
            with self.subTest(line=line), self.assertRaises(ValueError):
                command(Client(), line)


if __name__ == "__main__" and sys.argv[1:2] == ["serve"]:
    ports, mode, bench = json.loads(sys.argv[2]), sys.argv[3], bool(int(sys.argv[4]))
    extra_args = sys.argv[5:]
    dxhand_service.ManusSource = lambda **options: Manus(mode)
    sys.argv = [__file__] + [arg for side, path in ports.items() for arg in ("--" + side, path)]
    if bench:
        sys.argv.append("--bench")
    sys.argv.extend(extra_args)
    sys.exit(dxhand_service.main())
