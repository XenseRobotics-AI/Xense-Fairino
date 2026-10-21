"""Select fresh measured jaw openings; never substitute a requested target."""
import math
import time

from .gripper_core import SIDES, opening


def measured_openings(*states, now=None, max_age=.5):
    now = time.monotonic() if now is None else now
    result, newest = {}, {}
    for state in states:
        if not isinstance(state, dict) or state.get('error'):
            continue
        received = state.get('received')
        if type(received) not in (int, float) or not math.isfinite(received) or not 0 <= now-received <= max_age:
            continue
        devices = state.get('devices')
        if not isinstance(devices, dict):
            continue
        for side, unit in devices.items():
            if side not in SIDES or not isinstance(unit, dict) or not unit.get('connected'):
                continue
            try:
                value = opening(unit.get('position'))
                age = unit['age_ms']/1000 + now-received
                if not math.isfinite(age) or not 0 <= age <= max_age:
                    continue
            except (ValueError, TypeError, KeyError):
                continue
            sample_time = now-age
            if sample_time > newest.get(side, -math.inf):
                result[side], newest[side] = value, sample_time
    return result
