"""Compare the service entry with TsenseIV2C-main's original Manus WebSession, offline."""
import argparse
import json
from pathlib import Path
import os
import subprocess
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
UPSTREAM = Path.home() / "Olorin/05_DXHand/TsenseIV2C-main"


def sample(side, index):
    # Sweep thumb opposition and finger flexion; wrist-relative coordinates (m).
    points = [[0., 0., 0.]]
    for finger in range(5):
        x = (.034 - .017 * finger) * (1 if side == "left" else -1)
        point = np.array([x, .035 if finger == 0 else .065, 0.])
        for joint in range(4):
            if joint:
                angle = joint * (.35 + .32 * np.sin(index * .09 + finger * .4))
                point = point + [(.003 if finger else .008) * np.sign(x),
                                 .022 * np.cos(angle), .022 * np.sin(angle)]
            points.append(point.tolist())
    return {"world": points, "normalized": points, "label": side.title()}


def verify(upstream, frames):
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    env.pop("PYTHONPATH", None)
    # Separate interpreter and module cache for the original WebSession path.
    reference = '''
import json, sys
from online_control.online_motion_track.main import WebSession
hands = json.load(sys.stdin)
out = []
for side, seq in hands.items():
    session = WebSession(side, recording=False, collision_mode="off", input_source="manus")
    assert session.pipeline.session.profile.model_id == "TsenseI_v3_2"
    for i, hand in enumerate(seq):
        result = session.process(dict(sequence=i, timestamp_ms=i*15., capture_unix_ms=i*15., hand=hand))
        if not result["tracking"]:
            raise ValueError(result["message"])
        assert (result["source"], result["depth_source"]) == ("manus", "manus_metric")
        out.append(result["pose"])
print(json.dumps(out))
'''
    warmup = 8
    hands = {side: [sample(side, i) for i in range(frames + warmup)] for side in ("left", "right")}
    result = subprocess.run([sys.executable, "-B", "-c", reference], cwd=upstream, env=env,
                            input=json.dumps(hands), capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError(result.stderr)
    expected = iter(json.loads(result.stdout))
    sys.path.insert(0, str(ROOT / "scripts/dxhand"))
    from retarget import Retarget

    durations = []
    max_error = 0.
    for side, seq in hands.items():
        solver = Retarget(side)
        if solver.pipeline.session.profile.model_id != "TsenseI_v3_2":
            raise AssertionError("unexpected service model")
        seen = []
        for i, hand in enumerate(seq):
            begin = time.perf_counter_ns()
            pose = solver.solve(hand)
            elapsed = (time.perf_counter_ns() - begin) / 1e6
            original = next(expected)
            if len(pose) != 20 or pose.keys() != original.keys():
                raise AssertionError("joint names differ")
            values = np.array([pose[n] for n in sorted(pose)])
            difference = np.abs(values - [original[n] for n in sorted(pose)])
            if not np.isfinite(difference).all():
                raise AssertionError("non-finite retarget")
            max_error = max(max_error, float(difference.max()))
            seen.append(values)
            if i >= warmup:
                durations.append(elapsed)
        if np.max(np.ptp(seen, axis=0)) < .01:
            raise AssertionError("synthetic motion did not change joint targets")
    print(f"TsenseI_v3_2 manus/manus_metric left+right parity frames={sum(map(len, hands.values()))} joints/frame=20 max_abs_rad={max_error:.12g}")
    print(f"latency ms ({len(durations)} frames; {warmup} warmup/side): " +
          " ".join(f"p{p}={np.percentile(durations, p):.3f}" for p in [50, 90, 95, 99, 100]))
    if max_error != 0:
        raise AssertionError("TsenseIV2C-main WebSession output mismatch")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upstream", type=Path, default=UPSTREAM)
    parser.add_argument("--frames", type=int, default=120)
    options = parser.parse_args()
    verify(options.upstream, options.frames)
