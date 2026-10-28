"""Original NumPy constrained ART7 IK experiment, with no robot transport.

All public joint values and rates are radians; Cartesian positions are mm.
The solver uses its own FK/Jacobian and a strictly convex box QP. The original
wrist envelope remains an independent final path validator. This module is
deliberately separate from production control and makes no real-time claim.
"""
from __future__ import annotations

import math
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for directory in (ROOT / 'apps/fairino_tcp_probe', ROOT / 'apps/fairino_upper_probe'):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))
from tcp_core import ALPHA, THETA, D_MM, FLANGE_OFFSET_MM, BOUNDS_DEG, check_arm_limits
from tool_frames import tool_config, validate_runtime, for_arm
from wrist_guard import BOUNDARY, MARGIN_DEG, PREDICTION_SECONDS, WristGuard


def vector(value, name='joint vector'):
    a = np.asarray(value, dtype=float)
    if a.shape != (7,) or not np.isfinite(a).all():
        raise ValueError(f'{name} must contain seven finite values')
    return a.copy()


def transform(value):
    a = np.asarray(value, dtype=float)
    if a.shape != (4, 4) or not np.isfinite(a).all():
        raise ValueError('goal must be a finite 4 by 4 transform')
    if not np.allclose(a[3], [0., 0., 0., 1.], atol=1e-9, rtol=0.):
        raise ValueError('invalid homogeneous transform bottom row')
    r = a[:3, :3]
    if (not np.allclose(r.T @ r, np.eye(3), atol=1e-6, rtol=0.)
            or abs(np.linalg.det(r)-1.) > 1e-6):
        raise ValueError('goal rotation must be orthonormal and proper')
    return a.copy()


def rotation_log(r):
    """SO(3) log, including the pi branch (no Euler angle singularity)."""
    r = np.asarray(r, dtype=float)
    w = .5 * np.array([r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1]])
    sin_angle = float(np.linalg.norm(w))
    cos_angle = float(np.clip((np.trace(r)-1.)*.5, -1., 1.))
    angle = math.atan2(sin_angle, cos_angle)
    if angle < 1e-8:
        return w
    if math.pi-angle < 1e-5:
        _, eigenvectors = np.linalg.eigh((r+r.T)*.5)
        axis = eigenvectors[:, -1]
        if sin_angle > 1e-10:
            if float(axis @ w) < 0:
                axis = -axis
        else:
            first = np.flatnonzero(np.abs(axis) > 1e-8)
            if len(first) and axis[first[0]] < 0:
                axis = -axis
        return angle*axis
    return (angle/sin_angle)*w


def solve_box_qp(hessian, gradient, lower, upper, *, max_iterations=200):
    """Solve min 0.5*x'H*x+g'x subject to lower<=x<=upper.

    Primal active set: solve the free subproblem, step to the first bound, then
    release a bound with a violated KKT sign. H must be symmetric positive
    definite; equality bounds are kept active. Returns the minimizer only.
    """
    h = np.asarray(hessian, dtype=float)
    g = np.asarray(gradient, dtype=float)
    lo, hi = np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)
    n = g.size
    if (g.ndim != 1 or n == 0 or h.shape != (n, n) or lo.shape != (n,) or hi.shape != (n,)
            or not all(np.isfinite(a).all() for a in (h, g, lo, hi))
            or np.any(lo > hi) or not np.allclose(h, h.T, atol=1e-10, rtol=1e-12)):
        raise ValueError('invalid box QP dimensions, coefficients, or bounds')
    try:
        np.linalg.cholesky(h)
        unconstrained = np.linalg.solve(h, -g)
    except np.linalg.LinAlgError as exc:
        raise ValueError('box QP Hessian must be positive definite') from exc
    if np.all(unconstrained >= lo) and np.all(unconstrained <= hi):
        return unconstrained
    x = np.clip(unconstrained, lo, hi)
    active = np.zeros(n, dtype=int)
    active[x <= lo] = -1
    active[x >= hi] = 1
    fixed = hi-lo <= 1e-14
    active[fixed] = 2
    for _ in range(max_iterations):
        free = np.flatnonzero(active == 0)
        locked = np.flatnonzero(active != 0)
        if len(free):
            rhs = -g[free]
            if len(locked):
                rhs -= h[np.ix_(free, locked)] @ x[locked]
            proposed = np.linalg.solve(h[np.ix_(free, free)], rhs)
            direction = proposed-x[free]
            ratios = np.ones(len(free))
            positive = direction > 1e-15
            negative = direction < -1e-15
            ratios[positive] = (hi[free[positive]]-x[free[positive]])/direction[positive]
            ratios[negative] = (lo[free[negative]]-x[free[negative]])/direction[negative]
            amount = max(0., min(1., float(np.min(ratios))))
            x[free] += amount*direction
            x = np.clip(x, lo, hi)
            if amount < 1.-1e-12:
                hit = np.flatnonzero(ratios <= amount+1e-12)
                for local in hit:
                    index = free[local]
                    if direction[local] > 0:
                        x[index] = hi[index]; active[index] = 1
                    else:
                        x[index] = lo[index]; active[index] = -1
                continue
        derivative = h @ x+g
        tolerance = 1e-9*(1.+float(np.max(np.abs(g)))+float(np.max(np.abs(h @ x))))
        violation = np.where(active == -1, -derivative, np.where(active == 1, derivative, -np.inf))
        index = int(np.argmax(violation))
        if violation[index] <= tolerance:
            return np.clip(x, lo, hi)
        active[index] = 0
    raise RuntimeError('box QP active set did not converge within its iteration budget')


class Art7ConstrainedIK:
    def __init__(self, tools=None, side='left', bounds_deg=None):
        if side not in ('left', 'right'):
            raise ValueError('side must be left or right')
        self.side = side
        self.tools = validate_runtime(tool_config() if tools is None else tools)
        self.tool = np.asarray(for_arm(self.tools, side), dtype=float)
        nominal = np.asarray(BOUNDS_DEG, dtype=float)
        bounds = nominal.copy() if bounds_deg is None else np.asarray(bounds_deg, dtype=float)
        if (bounds.shape != (7, 2) or not np.isfinite(bounds).all()
                or np.any(bounds[:, 0] >= bounds[:, 1])
                or np.any(bounds[:, 0] < nominal[:, 0]) or np.any(bounds[:, 1] > nominal[:, 1])):
            raise ValueError('bounds_deg must be seven ordered ranges within the nominal limits')
        self.lower, self.upper = np.radians(bounds[:, 0]), np.radians(bounds[:, 1])
        self.offset = np.asarray(FLANGE_OFFSET_MM)+self.tool[:3, 3]
        self.guard = WristGuard()
        self.rotation_scale = 200.  # mm/rad, numerical scaling, not a safety limit

    def _fk_jacobian(self, q):
        r = np.eye(3); p = np.zeros(3)
        origins = np.empty((7, 3)); axes = np.empty((7, 3))
        for i, (value, d, alpha, theta) in enumerate(zip(q, D_MM, ALPHA, THETA)):
            origins[i] = p; axes[i] = r[:, 2]
            c, s = math.cos(value+theta), math.sin(value+theta)
            ca, sa = math.cos(alpha), math.sin(alpha)
            p = p+r[:, 2]*d
            r = r @ np.array([[c, -s*ca, s*sa], [s, c*ca, -c*sa], [0., sa, ca]])
        p = p+r @ self.offset
        result = np.eye(4); result[:3, :3] = r; result[:3, 3] = p
        jacobian = np.vstack((np.cross(axes, p-origins).T, axes.T))
        return result, jacobian

    def forward(self, q):
        return self._fk_jacobian(vector(q))[0]

    def jacobian(self, q):
        return self._fk_jacobian(vector(q))[1]

    def _error(self, goal, actual):
        position = goal[:3, 3]-actual[:3, 3]
        angle = rotation_log(goal[:3, :3] @ actual[:3, :3].T)
        return np.r_[position, self.rotation_scale*angle], float(np.linalg.norm(position)), math.degrees(float(np.linalg.norm(angle)))

    def _rectangles(self, lower, upper, velocity):
        reserve = np.abs(np.degrees(velocity[-2:]))*PREDICTION_SECONDS
        boxes = []; seen = set()
        for x, y in BOUNDARY:
            caps = np.radians(np.array([x, y])-MARGIN_DEG-reserve)
            if np.any(caps < 0):
                continue
            # Avoid lookup discontinuities from roundoff, but keep exactly-zero
            # intervals: q6=0 is a legitimate (lower dimensional) safe rectangle.
            caps = np.maximum(0., caps-1e-10)
            lo, hi = lower.copy(), upper.copy()
            lo[-2:] = np.maximum(lo[-2:], -caps)
            hi[-2:] = np.minimum(hi[-2:], caps)
            if np.any(lo > hi):
                continue
            key = tuple(np.round(np.r_[lo[-2:], hi[-2:]], 13))
            if key not in seen:
                seen.add(key); boxes.append((lo, hi))
        return boxes

    def _safe(self, q, reference, measured, velocity, lower, upper):
        if not np.isfinite(q).all() or np.any(q < lower-1e-12) or np.any(q > upper+1e-12):
            return False
        try:
            check_arm_limits(q.tolist())
            self.guard.arm_segment(reference.tolist(), q.tolist(), velocity.tolist())
            self.guard.arm_segment(measured.tolist(), q.tolist(), velocity.tolist())
            return True
        except ValueError:
            return False

    def _result(self, start, goal, q, iterations, qp_calls, status, reference, dt, previous_velocity):
        if q is None:
            return dict(success=False, feasible=False, q=None, target=None, tcp=None,
                        status=status, reason=status, position_error_mm=None, orientation_error_deg=None,
                        iterations=iterations, qp_calls=qp_calls, qp_solves=qp_calls,
                        elapsed_ms=1000*(time.perf_counter()-start), velocity=None, acceleration=None)
        tcp = self.forward(q); _, mm, deg = self._error(goal, tcp)
        success = mm <= .02 and deg <= .01
        velocity = None if dt is None else (q-reference)/dt
        acceleration = None if dt is None else (velocity-previous_velocity)/dt
        return dict(success=success, feasible=True, q=q.copy(), target=q.copy(), tcp=tcp,
                    status='converged' if success else status, reason='converged' if success else status,
                    position_error_mm=mm, orientation_error_deg=deg,
                    iterations=iterations, qp_calls=qp_calls, qp_solves=qp_calls,
                    elapsed_ms=1000*(time.perf_counter()-start), velocity=velocity, acceleration=acceleration)

    def _run(self, goal, reference, measured, velocity, lower, upper, *, max_iterations,
             dt=None, previous_velocity=None, trust_radius=.2):
        started = time.perf_counter(); q = reference.copy(); calls = 0
        if type(max_iterations) is not int or max_iterations < 1:
            raise ValueError('max_iterations must be a positive integer')
        # Do not manufacture a recovery command from a dynamically inadmissible
        # initial state. Existing manual-retreat policy belongs to the runtime.
        reserve = tuple(np.abs(np.degrees(velocity[-2:]))*PREDICTION_SECONDS)
        if (self.guard.clearance(reference[-2:].tolist(), reserve=reserve) < -1e-9
                or self.guard.clearance(measured[-2:].tolist(), reserve=reserve) < -1e-9):
            return self._result(started, goal, None, 0, calls, 'initial_wrist_state_infeasible', reference, dt, previous_velocity)
        try:
            self.guard.arm_segment(reference.tolist(), reference.tolist(), velocity.tolist())
            self.guard.arm_segment(measured.tolist(), measured.tolist(), velocity.tolist())
        except ValueError:
            return self._result(started, goal, None, 0, calls, 'initial_wrist_state_infeasible', reference, dt, previous_velocity)
        if np.any(lower > upper):
            return self._result(started, goal, None, 0, calls, 'empty_period_bounds', reference, dt, previous_velocity)
        rectangles = self._rectangles(lower, upper, velocity)
        if not rectangles:
            return self._result(started, goal, None, 0, calls, 'empty_wrist_region', reference, dt, previous_velocity)
        best = q.copy() if self._safe(q, reference, measured, velocity, lower, upper) else None
        reference_error = self._error(goal, self.forward(reference))[0]
        reference_merit = float(reference_error @ reference_error)
        best_merit = math.inf if best is None else reference_merit
        damping = .01
        for iteration in range(max_iterations):
            tcp, jacobian = self._fk_jacobian(q)
            error, mm, deg = self._error(goal, tcp)
            if mm <= .02 and deg <= .01 and self._safe(q, reference, measured, velocity, lower, upper):
                return self._result(started, goal, q, iteration, calls, 'converged', reference, dt, previous_velocity)
            jacobian[3:] *= self.rotation_scale
            hessian = jacobian.T @ jacobian+damping*np.eye(7)
            gradient = -jacobian.T @ error
            # Period bounds stay fixed. Trust bounds restrict only this internal
            # numerical step, and can never enlarge the total command limits.
            local_lo = np.maximum(lower, q-trust_radius)
            local_hi = np.minimum(upper, q+trust_radius)
            if np.any(local_lo > local_hi):
                break
            candidates = []
            try:
                calls += 1
                ordinary = q+solve_box_qp(hessian, gradient, local_lo-q, local_hi-q)
                if self._safe(ordinary, reference, measured, velocity, lower, upper):
                    candidates.append(ordinary)
            except RuntimeError:
                ordinary = None
            if not candidates:
                seen = set()
                for box_lo, box_hi in rectangles:
                    lo, hi = np.maximum(local_lo, box_lo), np.minimum(local_hi, box_hi)
                    if np.any(lo > hi):
                        continue
                    key = tuple(np.round(np.r_[lo[-2:], hi[-2:]], 13))
                    if key in seen:
                        continue
                    seen.add(key)
                    try:
                        calls += 1
                        candidate = q+solve_box_qp(hessian, gradient, lo-q, hi-q)
                    except RuntimeError:
                        continue
                    if self._safe(candidate, reference, measured, velocity, lower, upper):
                        candidates.append(candidate)
            current_merit = float(error @ error)
            next_q = None; next_merit = math.inf
            for candidate in candidates:
                for alpha in (1., .5, .25, .125, .0625):
                    trial = q+alpha*(candidate-q)
                    if not self._safe(trial, reference, measured, velocity, lower, upper):
                        continue
                    residual = self._error(goal, self._fk_jacobian(trial)[0])[0]
                    merit = float(residual @ residual)
                    if merit < next_merit:
                        next_q, next_merit = trial, merit
                    if merit <= current_merit:
                        break
            if next_q is None:
                break
            if next_merit < best_merit:
                best, best_merit = next_q.copy(), next_merit
            # A nonzero previous velocity may force motion away from an exact
            # target while braking. Return that only as an explicit partial step.
            if next_merit >= current_merit-1e-14 and self._safe(q, reference, measured, velocity, lower, upper):
                damping *= 10.
                if damping > 1e5 or np.max(np.abs(next_q-q)) < 1e-11:
                    break
                continue
            q = next_q
            damping = max(.001, damping*.5)
        if best is not None and self._safe(best, reference, measured, velocity, lower, upper):
            status = 'partial_progress' if np.max(np.abs(best-reference)) > 1e-11 else 'stalled'
            if best_merit > reference_merit+1e-12:
                status = 'bounded_step_away_from_goal'
            return self._result(started, goal, best, iteration+1, calls, status, reference, dt, previous_velocity)
        return self._result(started, goal, None, iteration+1, calls, 'no_feasible_candidate', reference, dt, previous_velocity)

    def _inputs(self, goal, seed, measured, measured_velocity):
        goal, seed = transform(goal), vector(seed, 'reference')
        measured = seed.copy() if measured is None else vector(measured, 'measured')
        velocity = np.zeros(7) if measured_velocity is None else vector(measured_velocity, 'measured_velocity')
        if np.any(seed < self.lower-1e-12) or np.any(seed > self.upper+1e-12):
            raise ValueError('reference is outside configured joint limits')
        check_arm_limits(measured.tolist())
        return goal, seed, measured, velocity

    def solve(self, goal, seed, max_iterations=60, *, measured=None, measured_velocity=None):
        goal, seed, measured, velocity = self._inputs(goal, seed, measured, measured_velocity)
        return self._run(goal, seed, measured, velocity, self.lower, self.upper,
                         max_iterations=max_iterations)

    def step(self, goal, reference, dt, previous_velocity, measured=None, measured_velocity=None,
             *, max_iterations=8, joint_velocity_limit=math.radians(30.),
             joint_acceleration_limit=math.radians(120.)):
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
        return self._run(goal, reference, measured, velocity, lower, upper,
                         max_iterations=max_iterations, dt=dt, previous_velocity=previous_velocity)
