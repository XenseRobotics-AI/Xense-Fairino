"""90 Hz cadence, remote timing propagation, and unchanged stopping contracts."""
import ast
import base64
import contextlib
import io
import json
import queue
import shlex
import tempfile
import threading
import unittest
import zlib
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

import paths
from vr_timing import Timing, Deadline
from vr_client import operate, remote_command, main
from vr_robot import VRProbe
from vr_core import direct_positions
from vr_input import decode_packet
from vr_plan import request
from vendor_kinematics import VendorKinematics
from test_tcp import hardware, probe


class TimingTests(unittest.TestCase):
    def test_link_check_exits_when_output_disconnects(self):
        from vr_link_check import run_server
        class BrokenOutput:
            def write(self, value):
                raise BrokenPipeError("closed")
            def flush(self):
                pass
        with patch("sys.stdin", io.StringIO('{"exit":true}\n')), patch("sys.stdout", BrokenOutput()):
            self.assertEqual(run_server(NS(duration=.01)), 1)

    def test_absolute_deadlines_do_not_accumulate_computation_time(self):
        clock = 10.
        deadline = Deadline(90., clock)
        for i in range(900):
            clock += .001+(i%3)*.0004
            clock += deadline.advance(clock)
        self.assertAlmostEqual(clock, 20., places=8)
        self.assertEqual(deadline.skipped, 0)

    def test_overrun_skips_expired_slots_instead_of_replaying_them(self):
        d = Deadline(100., 10.)
        self.assertAlmostEqual(d.advance(10.047), .003)
        self.assertEqual(d.skipped, 4)
        self.assertAlmostEqual(d.advance(10.051), .009)

    def test_invalid_timing_is_rejected_before_connection(self):
        for flag, value in (("--command-hz", "0"), ("--command-hz", "nan"),
                            ("--command-hz", "300"), ("--horizon-ms", "1")):
            with patch("vr_client.Connection") as connect, contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit): main([flag, value])
                connect.assert_not_called()

    def test_remote_bundle_has_same_timing_and_starts_no_ros_in_link_check(self):
        args = NS(execute=False, prepare_check=False, link_check=True, duration=1.,
                  command_hz=90., feedback_hz=100., receiver_hz=200., horizon_ms=0., interpolation_delay_ms=12.)
        command = remote_command(args)
        setup = shlex.split(command)[2]
        loader = shlex.split(setup.split("exec python3 -u -c ", 1)[1])[0]
        tree = ast.parse(loader)
        argv = next(ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign)
                    and ast.unparse(n.targets[0]) == "sys.argv")
        self.assertIn("--link-check", argv)
        self.assertEqual(argv[argv.index("--command-hz")+1], "90.0")
        encoded = next(n.args[0].value for n in ast.walk(tree) if isinstance(n, ast.Call)
                       and ast.unparse(n.func) == "base64.b64decode")
        bundle = json.loads(zlib.decompress(base64.b64decode(encoded)))
        self.assertIn("vr_timing", bundle)
        worker = bundle["vr_robot"]
        self.assertLess(worker.index("return run_server(args)"), worker.index("    load_ros()"))

    def test_all_command_rates_use_direct_targets_and_explicit_tcp_buffer(self):
        hw = hardware()
        points = direct_positions(hw["position"], {}, hw["position"])
        self.assertEqual(points, [(0., hw["position"])])
        for rate in (20., 90., 180.):
            self.assertEqual(Timing.from_args(NS(command_hz=rate)).horizon_ms, 0.)
            self.assertEqual(Timing.from_args(NS(command_hz=rate)).interpolation_delay_ms, 12.)

    def test_feedback_is_100hz_when_hardware_callbacks_arrive_at_500hz(self):
        p = probe(); p.__class__ = VRProbe
        del p.report
        p.timing = Timing()
        now = [10.]
        with patch("vr_robot.time.monotonic", lambda: now[0]):
            for tick in range(1000):
                now[0] = 10.+tick*.002
                p.report()
        reports = [v for v in p.events if "stamp" in v]
        self.assertTrue(198 <= len(reports) <= 201, len(reports))
        self.assertEqual(reports[-1]["timing"]["command_hz"], 90.)

    def test_new_input_interrupts_idle_poll_while_feedback_is_serviced(self):
        p = probe(); p.__class__ = VRProbe
        p.input_event = threading.Event()
        clock, ticks = [10.], [0]
        def pump(seconds):
            clock[0] += seconds
            ticks[0] += 1
            if ticks[0] == 2:
                p.input_event.set()
        p.pump = pump
        with patch("vr_robot.time.monotonic", lambda: clock[0]):
            p.pump_input(.005)
        self.assertAlmostEqual(clock[0], 10.002)
        self.assertEqual(ticks[0], 2)
        self.assertFalse(p.input_event.is_set())

    def test_real_client_generates_90hz_with_vendor_ik_and_nonzero_work_cost(self):
        clock = [10.]
        hw, sent = hardware(), []
        with tempfile.TemporaryDirectory() as directory:
            c = NS(events=queue.Queue(), done=threading.Event(), last_status=10.,
                   latest={"hardware": hw, "stamp": 10., "ready": True},
                   log_path=Path(directory)/"cadence.jsonl")
            def send(**kw):
                if kw.get("request"):
                    sent.append(kw["request"])
            def sleep(dt):
                clock[0] += dt
                c.last_status = c.latest["stamp"] = clock[0]
            def read():
                grip = 0. if clock[0] < 10.005 else 1.
                raw = {"timeStampNs": int(clock[0]*1e9), "Controller":
                       {s: {"pose": [0.,0.,0.,0.,0.,0.,1.], "grip": grip} for s in ("left","right")}}
                return decode_packet(json.dumps({"value": json.dumps(raw)}))
            def plan(*a, **kw):
                result = request(*a, **kw)
                clock[0] += .002
                return result
            c.send = send
            args = NS(execute=True, sensitivity=.25, no_auto_home=True)
            with patch("vr_client.time.monotonic", lambda: clock[0]), \
                 patch("vr_client.time.sleep", sleep), patch("vr_client.request", plan), \
                 patch("vr_client.keyboard", contextlib.nullcontext), \
                 patch("vr_client.key", lambda: "q" if clock[0] >= 15. else ""), \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(operate(args, VendorKinematics(), NS(read=read), c), 0)
            self.assertTrue(448 <= len(sent) <= 451, len(sent))
            logs = [json.loads(x) for x in c.log_path.with_suffix(".actions.jsonl").read_text().splitlines()]
            times = [v["plan_monotonic"] for v in logs if "plan_monotonic" in v]
            self.assertAlmostEqual((len(times)-1)/(times[-1]-times[0]), 90., places=6)


if __name__ == "__main__":
    unittest.main()
