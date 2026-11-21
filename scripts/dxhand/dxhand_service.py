"""Standalone DXHand USB service. Startup checks firmware side and requires the
power-up homed state (no host homing), then announces ready with follow off.

stdin: UTF-8 JSON objects, one complete line per command (max 4096 bytes).
{"cmd":"heartbeat","sent_ns":123} supplies the sender's time.monotonic_ns()
at send time on this host. Send at e.g. 20 Hz; a heartbeat expires 200 ms
after sent_ns, even if delayed in the pipe. Future timestamps are invalid.
{"cmd":"follow","enabled":true} enables tracking once ready; false freezes.
{"cmd":"stop"} requests shutdown. Follow commands do not renew the lease.
Malformed/partial lines, unknown or duplicate fields and early follow-on
end the session with an error. EOF, SIGTERM and SIGINT also end the session.

stdout: one JSON status per line at 10 Hz, with ready, follow (requested),
heartbeat_fresh and hands keyed by side. Per-hand phase is connecting,
checking, holding, tracking or closed. Only fresh accepted Manus data and
a live heartbeat allow position writes. Hold retains the last transmitted
target; the next accepted frame restarts Ruckig there with zero derivatives.
The final event is "stopped", with reason, errors, and per-hand driver_off_sent
and disable_confirmed. An unclaimed hand receives no power commands.
driver_off_sent records the USB write; disable_confirmed is always false.
A failed broadcast or protocol error returns nonzero. Physical disable needs
on-site verification. A disconnected client may miss the final status.

--bench requires a terminal. Space toggles follow after ready; q quits.
The live keyboard loop supplies heartbeats. --left/--right take USB paths;
use auto for a single unambiguous USB port. Each selected hand homes at start.
"""
import argparse
from contextlib import ExitStack
import json
import os
import select
import signal
import sys
import termios
import threading
import time
import tty

from hand import Client, Hand, LOCAL, PORT_STORE, probe_side, side_lock
import ports as hand_ports
from online_control.online_motion_track.manus_source import ManusSource, load_sdk


def unique_fields(pairs):
    obj = dict(pairs)
    if len(obj) != len(pairs):
        raise ValueError("JSON 含重复字段")
    return obj


def command(client, line):
    obj = json.loads(line.decode("utf-8"), object_pairs_hook=unique_fields)
    if type(obj) is dict and set(obj) == {"cmd", "sent_ns"} and obj["cmd"] == "heartbeat":
        client.beat(obj["sent_ns"])
    elif obj == {"cmd": "stop"}:
        client.stop("stop")
    elif (type(obj) is dict and set(obj) == {"cmd", "enabled"}
          and obj["cmd"] == "follow" and type(obj["enabled"]) is bool):
        client.set_follow(obj["enabled"])
    else:
        raise ValueError("非法 dxhand 命令")


class Service:
    def __init__(self, ports, make_source, *, allow_right_dip_offset=False):
        self.client = Client()
        self.ports = ports
        self.make_source = make_source
        self.allow_right_dip_offset = allow_right_dip_offset
        self.hands = {}

    def run(self):
        c = self.client
        source = None
        with ExitStack() as locks:
            try:
                source = self.make_source()
                resolved = hand_ports.resolve(self.ports, PORT_STORE, probe_side)
                for side, port in resolved.items():
                    if c.ending.is_set():
                        return
                    locks.enter_context(side_lock(side))
                    hand = Hand(side, c, allow_right_dip_offset=self.allow_right_dip_offset)
                    self.hands[side] = hand
                    hand.connect(port)
                # Verify every configured hand before either receives power.
                for hand in self.hands.values():
                    hand.prepare()
                with c.lock:
                    c.ready = not c.ending.is_set()
                next_check = 0
                while not c.ending.wait(.002):
                    check = time.monotonic() >= next_check
                    if check:
                        next_check = time.monotonic() + .1
                    for hand in self.hands.values():
                        if check and not hand.device.get_control_state()["connected"]:
                            raise RuntimeError(f"{hand.side}: USB disconnected")
                        hand.update(source)
            except InterruptedError as exc:
                if not c.ending.is_set():
                    c.stop(str(exc), error=True)
            except Exception as exc:
                c.stop(str(exc), error=True)
            finally:
                c.stop("shutdown")
                c.ready = False
                # Close the gate for both hands first. Attempt every shutdown
                # operation even if a USB cable has already been removed.
                for hand in self.hands.values():
                    self.cleanup_call(hand.control.freeze)
                for hand in self.hands.values():
                    if hand.owned:
                        hand.status["driver_off_sent"] = self.cleanup_call(hand.native.driver_off)
                    self.cleanup_call(hand.control.close)
                    hand.status["phase"] = "closed"
                if source is not None:
                    self.cleanup_call(source.close)

    def cleanup_call(self, action):
        try:
            action()
            return True
        except Exception as exc:
            self.client.stop(str(exc), error=True)
            return False

    def status(self, final=False):
        c = self.client
        with c.lock:
            hands = {}
            for side in self.ports:
                hand = self.hands.get(side)
                if hand is None:
                    hands[side] = dict(phase="connecting", driver_off_sent=False, disable_confirmed=False)
                else:
                    hands[side] = dict(hand.status)
                    if hand.status["phase"] == "holding" and time.monotonic_ns() < hand.lease.deadline():
                        hands[side]["phase"] = hand.control.phase
            status = dict(event="status", ready=c.ready, follow=c.follow,
                          heartbeat_fresh=time.monotonic_ns() < c.heartbeat_until, hands=hands)
            if c.ending.is_set() and not final:
                # Tell the client at once why the service is going down; it must not wait
                # for cleanup to finish before it stops the arms.
                status.update(stopping=True, reason=c.reason, errors=list(c.errors))
            if final:
                status.update(event="stopped", reason=c.reason, errors=list(c.errors),
                              message="物理下使能需现场核验")
            return status


def serve(service, bench, output):
    c = service.client
    worker = threading.Thread(target=service.run, name="DXHand")
    buffer = bytearray()
    next_status = 0.

    def emit(final=False):
        data = json.dumps(service.status(final), ensure_ascii=False, allow_nan=False).encode() + b"\n"
        # Keep the command watchdog live when a client stops draining stdout.
        if select.select([], [output], [], .2 if final else 0)[1]:
            os.write(output, data)
        elif final:
            sys.stderr.buffer.write(data)
            sys.stderr.buffer.flush()

    with ExitStack() as stack:
        for sig in (signal.SIGTERM, signal.SIGINT):
            previous = signal.signal(sig, lambda number, _: c.stop(signal.Signals(number).name))
            stack.callback(signal.signal, sig, previous)
        if bench:
            saved = termios.tcgetattr(0)
            stack.callback(termios.tcsetattr, 0, termios.TCSANOW, saved)
            tty.setcbreak(0)
        worker.start()
        try:
            while not c.ending.is_set():
                if bench:
                    c.beat(time.monotonic_ns())
                if time.monotonic() >= next_status:
                    emit()
                    next_status = time.monotonic() + .1
                if not select.select([0], [], [], min(.025, max(0., next_status - time.monotonic())))[0]:
                    continue
                data = os.read(0, 4096)
                if not data:
                    if buffer:
                        raise ValueError("stdin EOF 前有不完整的 JSON 行")
                    c.stop("eof")
                elif bench:
                    for key in data:
                        if key in (ord("q"), 4):
                            c.stop("bench quit")
                            break
                        if key == ord(" ") and c.ready:
                            c.set_follow(not c.follow)
                else:
                    buffer.extend(data)
                    while b"\n" in buffer and not c.ending.is_set():
                        line, _, buffer = buffer.partition(b"\n")
                        if len(line) > 4096:
                            raise ValueError("JSON 行超过 4096 字节")
                        command(c, line)
                    if len(buffer) > 4096:
                        raise ValueError("JSON 行超过 4096 字节")
        except Exception as exc:
            c.stop(str(exc), error=True)
        finally:
            c.stop("shutdown")
            while worker.is_alive():
                # Cleanup can take seconds; keep the client's status watchdog fed so it
                # reports the real stop reason instead of a status timeout.
                try:
                    emit()
                except OSError:
                    pass
                worker.join(.1)
            print("物理下使能需现场核验", file=sys.stderr, flush=True)
            try:
                emit(final=True)
            except BrokenPipeError:
                pass
    return int(bool(c.errors))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--left", metavar="USB")
    parser.add_argument("--right", metavar="USB")
    parser.add_argument("--left-calibration")
    parser.add_argument("--right-calibration")
    parser.add_argument("--bench", action="store_true")
    parser.add_argument("--allow-right-dip-offset", action="store_true",
                        help="台架起步时，允许右中指末节最低读到 -0.30 mm，按 0 mm 起步")
    args = parser.parse_args()
    ports = {side: port for side in ("left", "right") if (port := getattr(args, side))}
    if not ports:
        parser.error("至少指定 --left USB 或 --right USB")
    if args.allow_right_dip_offset and not (args.bench and args.right):
        parser.error("--allow-right-dip-offset 需要 --bench 和 --right USB")
    if args.bench and not os.isatty(0):
        parser.error("--bench 需要交互式终端")
    # Native libraries may write diagnostics via libc. Reserve the original
    # stdout file descriptor for protocol messages before loading the SDK.
    output = os.dup(1)
    os.dup2(2, 1)
    def make_source():
        module_dir = LOCAL / "vendor/manussdk/build/python"
        load_sdk(module_dir)
        return ManusSource(module_dir=module_dir, left_calibration=args.left_calibration,
                           right_calibration=args.right_calibration)

    try:
        service = Service(ports, make_source, allow_right_dip_offset=args.allow_right_dip_offset)
        return serve(service, args.bench, output)
    finally:
        os.close(output)


if __name__ == "__main__":
    sys.exit(main())
