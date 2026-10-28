"""ctypes wrapper for the C++17 ART7 constrained IK core.

Building uses only the installed g++ and caches the shared object under /tmp.
The standalone test and main PICO native adapter share this numerical core.
This module has no robot/network interfaces. Public units and result fields
match Art7ConstrainedIK; callers own validation, transport and lifecycle.
"""
from __future__ import annotations

import ctypes as C
import hashlib
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time

import numpy as np

from .art7_qp import Art7ConstrainedIK, vector, transform
from tcp_core import check_arm_limits
from wrist_guard import CONFIG_ID, PREDICTION_SECONDS


SOURCE = Path(__file__).with_suffix('.cpp')
FLAGS = ('-O3', '-std=c++17', '-DNDEBUG', '-shared', '-fPIC')
DoublePointer = C.POINTER(C.c_double)


class _Result(C.Structure):
    _fields_ = [(name, C.c_int) for name in ('feasible', 'success', 'status', 'iterations', 'qp_calls')] + [
        ('q', C.c_double*7), ('tcp', C.c_double*16),
        ('position_error_mm', C.c_double), ('orientation_error_deg', C.c_double)]


def build_native(force=False):
    """Compile this exact source to a content-addressed local temporary cache."""
    digest = hashlib.sha256(SOURCE.read_bytes()+repr(FLAGS).encode()).hexdigest()[:20]
    directory = Path(tempfile.gettempdir())/'art7-ik-native-cache'
    directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    library = directory/f'art7-{digest}.so'
    if force or not library.exists():
        temporary = directory/f'art7-{digest}-{os.getpid()}-{time.time_ns()}.so'
        try:
            completed = subprocess.run(['g++', *FLAGS, str(SOURCE), '-o', str(temporary)],
                                       capture_output=True, text=True, timeout=60, check=False)
            if completed.returncode:
                raise RuntimeError('ART7 native build failed: '+completed.stderr[-4000:])
            temporary.replace(library)
        finally:
            temporary.unlink(missing_ok=True)
    return library


_LIBRARIES = {}


def _library():
    path = build_native()
    key = str(path)
    if key not in _LIBRARIES:
        lib = C.CDLL(key)
        lib.art7_abi_version.argtypes = []; lib.art7_abi_version.restype = C.c_int
        lib.art7_wrist_config.argtypes = []; lib.art7_wrist_config.restype = C.c_char_p
        if lib.art7_abi_version() != 1 or lib.art7_wrist_config().decode() != CONFIG_ID:
            raise RuntimeError('ART7 native ABI or wrist-envelope configuration mismatch')
        lib.art7_fk.argtypes = [DoublePointer]*4; lib.art7_fk.restype = C.c_int
        lib.art7_rotation_log.argtypes = [DoublePointer]*2; lib.art7_rotation_log.restype = C.c_int
        lib.art7_box_qp.argtypes = [DoublePointer]*4+[C.c_int, DoublePointer]
        lib.art7_box_qp.restype = C.c_int
        lib.art7_run.argtypes = [DoublePointer]*8+[C.c_int, C.c_double, C.c_int, C.POINTER(_Result)]
        lib.art7_run.restype = C.c_int
        _LIBRARIES[key] = lib
    return _LIBRARIES[key], path


def _ptr(array):
    return array.ctypes.data_as(DoublePointer)


def solve_box_qp_native(hessian, gradient, lower, upper):
    """Independent native QP entry point for numerical cross-validation, n<=7."""
    h = np.ascontiguousarray(hessian, dtype=float)
    g = np.ascontiguousarray(gradient, dtype=float)
    lo = np.ascontiguousarray(lower, dtype=float); hi = np.ascontiguousarray(upper, dtype=float)
    n = g.size
    if (g.ndim != 1 or not 1 <= n <= 7 or h.shape != (n, n) or lo.shape != (n,)
            or hi.shape != (n,) or not all(np.isfinite(a).all() for a in (h, g, lo, hi))
            or np.any(lo > hi)):
        raise ValueError('invalid native box QP')
    out = np.empty(n); lib, _ = _library()
    rc = lib.art7_box_qp(_ptr(h), _ptr(g), _ptr(lo), _ptr(hi), n, _ptr(out))
    if rc:
        raise ValueError('native box QP invalid or did not converge')
    return out


class Art7NativeIK(Art7ConstrainedIK):
    def __init__(self, tools=None, side='left', bounds_deg=None):
        super().__init__(tools=tools, side=side, bounds_deg=bounds_deg)
        self.lib, self.library_path = _library()
        self.offset = np.ascontiguousarray(self.offset, dtype=float)

    def _native_fk(self, q, with_jacobian):
        q = np.ascontiguousarray(vector(q))
        tcp = np.empty((4, 4)); jacobian = np.empty((6, 7)) if with_jacobian else None
        rc = self.lib.art7_fk(_ptr(q), _ptr(self.offset), _ptr(tcp),
                             _ptr(jacobian) if with_jacobian else None)
        if rc:
            raise ValueError('native FK rejected invalid input')
        return tcp, jacobian

    def forward(self, q):
        return self._native_fk(q, False)[0]

    def jacobian(self, q):
        return self._native_fk(q, True)[1]

    def _joint_bounds(self, lower, upper, period_lower, period_upper):
        lower, upper = lower.copy(), upper.copy()
        if period_lower is not None:
            lower = np.maximum(lower, vector(period_lower, 'period_lower'))
        if period_upper is not None:
            upper = np.minimum(upper, vector(period_upper, 'period_upper'))
        return np.ascontiguousarray(lower), np.ascontiguousarray(upper)

    def _run_native(self, goal, reference, measured, measured_velocity, lower, upper,
                    max_iterations, allow_guard_retreat, reserve_velocity,
                    dt=None, previous_velocity=None):
        started = time.perf_counter()
        if type(max_iterations) is not int or max_iterations < 1:
            raise ValueError('max_iterations must be a positive integer')
        if type(allow_guard_retreat) is not bool:
            raise ValueError('allow_guard_retreat must be boolean')
        extra = measured_velocity.copy() if reserve_velocity is None else vector(reserve_velocity, 'reserve_velocity')
        # C++ retains actual velocity signs for the manual-retreat rule, while
        # the magnitude used for reserve is never less than real feedback.
        reserve = np.maximum(np.abs(measured_velocity[-2:]), np.abs(extra[-2:]))
        output = _Result()
        inputs = [np.ascontiguousarray(a, dtype=float) for a in
                  (goal, reference, measured, measured_velocity, extra, lower, upper, self.offset)]
        rc = self.lib.art7_run(*(_ptr(a) for a in inputs), max_iterations, .2,
                              int(allow_guard_retreat), C.byref(output))
        if rc:
            raise ValueError('native constrained IK rejected invalid input')
        statuses = ('converged', 'partial_progress', 'stalled', 'bounded_step_away_from_goal',
                    'initial_wrist_state_infeasible', 'empty_period_bounds',
                    'empty_wrist_region', 'no_feasible_candidate')
        status = statuses[output.status] if 0 <= output.status < len(statuses) else 'native_internal_error'
        result = dict(success=False, feasible=False, q=None, target=None, tcp=None,
                      status=status, reason=status, position_error_mm=None,
                      orientation_error_deg=None, iterations=output.iterations,
                      qp_calls=output.qp_calls, qp_solves=output.qp_calls,
                      velocity=None, acceleration=None, backend='native_cpp17')
        if output.feasible:
            q = np.array(output.q, dtype=float); tcp = np.array(output.tcp, dtype=float).reshape(4, 4)
            if (not np.isfinite(q).all() or not np.isfinite(tcp).all()
                    or np.any(q < lower-1e-12) or np.any(q > upper+1e-12)):
                raise RuntimeError('native output violated its fixed period bounds')
            check_arm_limits(q.tolist())
            # Always independently validate both original runtime guard paths.
            for start in (reference, measured):
                self.guard.arm_segment(start.tolist(), q.tolist(), measured_velocity.tolist())
                inward = bool(np.all(start[-2:]*measured_velocity[-2:] <= 1e-10))
                if not self.guard._pair_motion(start[-2:].tolist(), q[-2:].tolist(),
                                              tuple(np.degrees(reserve)*PREDICTION_SECONDS),
                                              allow_retreat=allow_guard_retreat and inward):
                    raise RuntimeError('native output failed independent conservative wrist check')
            _, mm, deg = self._error(goal, tcp)
            # Do not trust a native success flag without checking its reported
            # TCP error against the unchanged public convergence contract.
            success = mm <= .02 and deg <= .01
            if bool(output.success) != success:
                raise RuntimeError('native convergence status disagrees with independent error calculation')
            v = None if dt is None else (q-reference)/dt
            acceleration = None if dt is None else (v-previous_velocity)/dt
            result.update(success=success, feasible=True, q=q, target=q.copy(), tcp=tcp,
                          position_error_mm=mm, orientation_error_deg=deg,
                          velocity=v, acceleration=acceleration)
        result['elapsed_ms'] = 1000*(time.perf_counter()-started)
        return result

    def solve(self, goal, seed, max_iterations=60, *, measured=None, measured_velocity=None,
              period_lower=None, period_upper=None, allow_guard_retreat=False,
              reserve_velocity=None):
        goal, seed, measured, velocity = self._inputs(goal, seed, measured, measured_velocity)
        lower, upper = self._joint_bounds(self.lower, self.upper, period_lower, period_upper)
        return self._run_native(goal, seed, measured, velocity, lower, upper, max_iterations,
                                allow_guard_retreat, reserve_velocity)

    def step(self, goal, reference, dt, previous_velocity, measured=None, measured_velocity=None,
             *, max_iterations=8, joint_velocity_limit=math.radians(30.),
             joint_acceleration_limit=math.radians(120.), period_lower=None,
             period_upper=None, allow_guard_retreat=False, reserve_velocity=None):
        goal, reference, measured, velocity = self._inputs(goal, reference, measured, measured_velocity)
        previous_velocity = vector(previous_velocity, 'previous_velocity')
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be finite and positive')
        vmax = np.broadcast_to(np.asarray(joint_velocity_limit, dtype=float), (7,)).copy()
        amax = np.broadcast_to(np.asarray(joint_acceleration_limit, dtype=float), (7,)).copy()
        if not np.isfinite(vmax).all() or not np.isfinite(amax).all() or np.any(vmax <= 0) or np.any(amax <= 0):
            raise ValueError('velocity and acceleration limits must be positive and finite')
        lower = np.maximum.reduce([self.lower, reference-vmax*dt,
                                   reference+(previous_velocity-amax*dt)*dt])
        upper = np.minimum.reduce([self.upper, reference+vmax*dt,
                                   reference+(previous_velocity+amax*dt)*dt])
        lower, upper = self._joint_bounds(lower, upper, period_lower, period_upper)
        return self._run_native(goal, reference, measured, velocity, lower, upper, max_iterations,
                                allow_guard_retreat, reserve_velocity, dt, previous_velocity)


if __name__ == '__main__':
    print(build_native())
