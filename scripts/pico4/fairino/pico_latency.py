#!/usr/bin/env python3
"""Non-actuating PICO timestamp-to-SDK latency with USB ADB clock bounds."""
import argparse
import json
import math
import select
import subprocess
import time
from pathlib import Path
from paths import ROOT
from vr_input import Stream
from vr_link_check import distribution, intervals


class UsbClock:
    def __init__(self, adb, serial):
        self.process = subprocess.Popen([str(adb), "-s", serial, "shell", "-T",
            "while read line; do date +%s%N; done"], stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)

    def calibrate(self, count=24):
        samples = []
        for _ in range(count):
            t0 = time.monotonic_ns()
            self.process.stdin.write(b"clock\n")
            if not select.select([self.process.stdout], [], [], 3.)[0]:
                raise RuntimeError("PICO USB 校时响应超时")
            raw = self.process.stdout.readline().strip()
            t1 = time.monotonic_ns()
            if not raw.isdigit() or len(raw) < 18:
                raise RuntimeError(f"PICO 不支持纳秒 date 响应：{raw!r}")
            remote = int(raw)
            samples.append({"sent_ns": t0, "received_ns": t1, "headset_unix_ns": remote,
                "offset_lower_ns": remote-t1, "offset_upper_ns": remote-t0})
        lower = max(v["offset_lower_ns"] for v in samples)
        upper = min(v["offset_upper_ns"] for v in samples)
        if lower > upper:
            raise RuntimeError("PICO 时钟在校准中发生跳变，无法给出可靠单程延迟")
        return {"at_monotonic_ns": (samples[0]["sent_ns"]+samples[-1]["received_ns"])//2,
            "headset_unix_minus_pc_monotonic_ns": (lower+upper)//2,
            "uncertainty_ms": (upper-lower)/2e6,
            "rtt": distribution([(s["received_ns"]-s["sent_ns"])/1e6 for s in samples]),
            "samples": samples}

    def close(self):
        self.process.stdin.close()
        try:
            self.process.wait(timeout=2.)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            self.process.wait(timeout=2.)


def calibrated_ages(samples, before, after):
    values = []
    for sample in samples:
        received = sample["received_monotonic_ns"]
        u = min(1., max(0., (received-before["at_monotonic_ns"])/
                       (after["at_monotonic_ns"]-before["at_monotonic_ns"])))
        # Interpolate the small difference separately to retain nanosecond
        # precision despite the large Unix-versus-monotonic offset.
        a = before["headset_unix_minus_pc_monotonic_ns"]
        b = after["headset_unix_minus_pc_monotonic_ns"]
        offset = a+round(u*(b-a))
        values.append((received-sample["headset_unix_ns"]+offset)/1e6)
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=20.)
    parser.add_argument("--serial")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 < args.duration <= 300:
        parser.error("duration 须在 0–300 秒之间")
    adb = ROOT/".local/pico4/platform-tools/adb"
    devices = subprocess.check_output([str(adb), "devices"], text=True, timeout=10.)
    serials = [v.split()[0] for v in devices.splitlines() if v.endswith("\tdevice")]
    serial = args.serial or (serials[0] if len(serials) == 1 else None)
    if serial not in serials:
        raise RuntimeError("需要一台 USB 调试已授权的 PICO；多台设备请用 --serial")
    clock, stream = UsbClock(adb, serial), None
    samples = []
    error = ""
    try:
        before = clock.calibrate()
        print(f"USB 校时误差范围约 ±{before['uncertainty_ms']:.2f} ms；只接收 PICO，不连接机器人。", flush=True)
        stream = Stream()
        stream.open()
        deadline = time.monotonic()+args.duration
        while time.monotonic() < deadline:
            try:
                frame = stream.read()
                for value in frame["samples"]:
                    samples.append({"headset_unix_ns": int(value["stamp"]),
                        "received_monotonic_ns": round(value["received_monotonic"]*1e9)})
                error = ""
            except ValueError as exc:
                error = str(exc)
            time.sleep(.001)
        after = clock.calibrate()
    finally:
        if stream is not None:
            stream.close()
        clock.close()
    ages = calibrated_ages(samples, before, after)
    uncertainty = max(before["uncertainty_ms"], after["uncertainty_ms"])
    drift = (after["headset_unix_minus_pc_monotonic_ns"]-
             before["headset_unix_minus_pc_monotonic_ns"])/1e6
    result = {"mode": "pico_timestamp_to_sdk_no_robot", "serial": serial,
        "pc_arrival": intervals([v["received_monotonic_ns"]/1e9 for v in samples]),
        "source_intervals": intervals([v["headset_unix_ns"]/1e9 for v in samples]),
        "timestamp_to_pc_sdk_ms": distribution(ages), "clock_uncertainty_ms": uncertainty,
        "offset_change_ms": drift, "last_error": error,
        "negative_age_samples": sum(v < 0 for v in ages),
        "clock_stable_within_bounds": abs(drift) <= before["uncertainty_ms"]+after["uncertainty_ms"]+2.,
        "calibration_before": before, "calibration_after": after,
        "samples": samples,
        "note": "SDK 文档定义 timeStampNs 为采集时的 Unix 纳秒。结果包含头显发送、网络、PC Service 和 SDK，使用 USB 时钟区间校正；不含已知时间戳之前的传感器追踪/预测，不是手柄物理运动到机械臂响应延迟。"}
    directory = ROOT/"scripts/pico4/fairino/diagnostics"
    path = args.output or directory/(time.strftime("%Y%m%d-%H%M%S")+"-pico-latency.json")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps({k: v for k, v in result.items() if k not in
        ("samples", "calibration_before", "calibration_after")}, ensure_ascii=False, indent=2), flush=True)
    print("PICO 延迟记录：", path, flush=True)
    return 0 if samples and result["clock_stable_within_bounds"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
