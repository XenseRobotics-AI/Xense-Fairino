"""Thin Python interfaces for the shared C++ Pinocchio / eiquadprog IK kernel."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np


ASSET_ROOT = Path(__file__).resolve().parent / "assets" / "tron2"
DEFAULT_URDF = ASSET_ROOT / "robot_grasper.urdf"
DEFAULT_TRON2_URDF = DEFAULT_URDF

LEFT_ARM_JOINTS = (
    "proximal_pitch_L_Joint",
    "proximal_roll_L_Joint",
    "proximal_yaw_L_Joint",
    "elbow_L_Joint",
    "wrist_yaw_L_Joint",
    "wrist_pitch_L_Joint",
    "wrist_roll_L_Joint",
)
RIGHT_ARM_JOINTS = tuple(name.replace("_L_", "_R_") for name in LEFT_ARM_JOINTS)
ARM_JOINTS = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS

DEFAULT_LEFT_FRAME = "wrist_roll_L_Link"
DEFAULT_RIGHT_FRAME = "wrist_roll_R_Link"


class Tron2IKError(RuntimeError):
    """Base class for Tron2 IK errors."""


class Tron2ConfigurationError(Tron2IKError, ValueError):
    """Raised when the model, target pose, or options are invalid."""


@dataclass
class IKOptions:
    """Native QP and fixed-joint settings for :meth:`Tron2QPModel.solve_ik`.

    Pose targets are world-frame homogeneous matrices.  Position tolerances
    and costs use metres; orientation tolerances and costs use radians.
    ``posture_target`` follows the public arm order: left seven, right seven.
    When posture regularization is enabled without an explicit target, the
    seed becomes the target so successive solves favour continuous solutions.
    """

    max_iterations: int = 500
    qp_max_iterations: int = 100
    column_avoidance: bool = True
    column_activation_distance: float = 0.04
    column_min_distance: float = 0.01
    column_link_radius: float = 0.04
    column_avoidance_weight: float = 4.0
    column_avoidance_gain: float = 1.0
    tolerance: float = 1e-5
    position_tolerance: float | None = None
    orientation_tolerance: float | None = None

    dt: float = 0.02
    damping: float = 1e-5
    lm_damping: float = 1e-5
    position_cost: float = 1.0
    orientation_cost: float = 1.0
    frame_gain: float = 1.0
    step_size: float = 1.0
    max_step_norm: float = 0.15
    safety_break: bool = False

    posture_weight: float = 0.0
    posture_gain: float = 1.0
    posture_target: np.ndarray | None = None

    qp_solver: str | None = None
    qp_options: dict[str, Any] = field(default_factory=dict)

    # These coordinates exist in the full URDF but are locked in the C++ reduced model.
    head_position: tuple[float, float] = (0.0, 0.0)
    left_grasper_base_position: float = 0.0
    right_grasper_base_position: float = 0.0
    left_gripper_position: float = 0.0
    right_gripper_position: float = 0.0

    def validate(self) -> None:
        """Validate values in place and normalize array-like fields."""

        if isinstance(self.max_iterations, bool) or int(self.max_iterations) != self.max_iterations:
            raise Tron2ConfigurationError("max_iterations must be an integer")
        self.max_iterations = int(self.max_iterations)
        if self.max_iterations < 1:
            raise Tron2ConfigurationError("max_iterations must be positive")

        non_negative = (
            "damping",
            "lm_damping",
            "position_cost",
            "orientation_cost",
            "posture_weight",
        )
        for name in non_negative:
            value = float(getattr(self, name))
            if not np.isfinite(value) or value < 0.0:
                raise Tron2ConfigurationError(f"{name} must be finite and non-negative")
            setattr(self, name, value)

        positive = ("tolerance", "dt", "max_step_norm")
        for name in positive:
            value = float(getattr(self, name))
            if not np.isfinite(value) or value <= 0.0:
                raise Tron2ConfigurationError(f"{name} must be finite and positive")
            setattr(self, name, value)

        if self.position_cost == 0.0 and self.orientation_cost == 0.0:
            raise Tron2ConfigurationError(
                "position_cost and orientation_cost cannot both be zero"
            )
        for name in ("position_tolerance", "orientation_tolerance"):
            value = getattr(self, name)
            if value is None:
                continue
            value = float(value)
            if not np.isfinite(value) or value <= 0.0:
                raise Tron2ConfigurationError(f"{name} must be finite and positive")
            setattr(self, name, value)

        for name in ("frame_gain", "posture_gain", "step_size"):
            value = float(getattr(self, name))
            if not np.isfinite(value) or not 0.0 < value <= 1.0:
                raise Tron2ConfigurationError(f"{name} must be in (0, 1]")
            setattr(self, name, value)

        if not isinstance(self.safety_break, bool):
            raise Tron2ConfigurationError("safety_break must be a bool")
        if self.qp_solver is not None:
            self.qp_solver = str(self.qp_solver).strip().lower()
            if not self.qp_solver:
                raise Tron2ConfigurationError("qp_solver cannot be empty")
        if not isinstance(self.qp_options, Mapping):
            raise Tron2ConfigurationError("qp_options must be a mapping")
        self.qp_options = dict(self.qp_options)
        if self.qp_solver not in (None, "eiquadprog"):
            raise Tron2ConfigurationError("Only the native C++ eiquadprog backend is supported")
        if self.qp_options:
            raise Tron2ConfigurationError("qp_options is unsupported; use qp_max_iterations")
        if isinstance(self.qp_max_iterations, bool) or int(self.qp_max_iterations) != self.qp_max_iterations or self.qp_max_iterations < 1:
            raise Tron2ConfigurationError("qp_max_iterations must be a positive integer")
        self.qp_max_iterations = int(self.qp_max_iterations)

        head = np.asarray(self.head_position, dtype=float).reshape(-1)
        if head.shape != (2,) or not np.isfinite(head).all():
            raise Tron2ConfigurationError("head_position must contain two finite values")
        self.head_position = (float(head[0]), float(head[1]))
        for name in (
            "left_grasper_base_position",
            "right_grasper_base_position",
            "left_gripper_position",
            "right_gripper_position",
        ):
            value = float(getattr(self, name))
            if not np.isfinite(value):
                raise Tron2ConfigurationError(f"{name} must be finite")
            setattr(self, name, value)

        if self.posture_target is not None:
            posture = np.asarray(self.posture_target, dtype=float).reshape(-1)
            if posture.shape != (14,) or not np.isfinite(posture).all():
                raise Tron2ConfigurationError(
                    "posture_target must be a finite 14-element vector"
                )
            self.posture_target = posture.copy()


@dataclass
class IKResult:
    """Solution and diagnostics returned by C++ constrained QP IK."""

    q: np.ndarray
    success: bool
    iterations: int
    error: float
    left_position_error: float
    left_orientation_error: float
    right_position_error: float
    right_orientation_error: float
    status: str = ""
    message: str = ""
    solver: str = ""
    qp_iterations: int = 0
    qp_active_constraints: int = 0
    solve_time_ms: float = 0.0
    column_collision_free: bool = True
    column_min_distance: float = float("inf")
    column_nearest_segment: str = ""
    full_q: np.ndarray | None = field(default=None, repr=False)

    def __post_init__(self) -> None:
        self.q = np.asarray(self.q, dtype=float).reshape(-1).copy()
        if self.full_q is not None:
            self.full_q = np.asarray(self.full_q, dtype=float).reshape(-1).copy()

    @property
    def position_error(self) -> float:
        return float(max(self.left_position_error, self.right_position_error))

    @property
    def orientation_error(self) -> float:
        return float(max(self.left_orientation_error, self.right_orientation_error))

    @property
    def residual(self) -> float:
        return float(self.error)

    def as_dict(self) -> dict[str, Any]:
        return {
            "q": self.q.tolist(),
            "success": bool(self.success),
            "iterations": int(self.iterations),
            "error": float(self.error),
            "position_error": self.position_error,
            "orientation_error": self.orientation_error,
            "left_position_error": float(self.left_position_error),
            "left_orientation_error": float(self.left_orientation_error),
            "right_position_error": float(self.right_position_error),
            "right_orientation_error": float(self.right_orientation_error),
            "status": self.status,
            "message": self.message,
            "solver": self.solver,
            "qp_iterations": self.qp_iterations,
            "qp_active_constraints": self.qp_active_constraints,
            "solve_time_ms": self.solve_time_ms,
            "column_collision_free": self.column_collision_free,
            "column_min_distance": self.column_min_distance,
        }



def _native_module():
    from .runtime import _preload_native_dependencies

    _preload_native_dependencies()
    try:
        from . import _native
    except ImportError as exc:
        raise Tron2IKError(
            "C++ IK extension unavailable. The original native-loader error was: "
            f"{exc}. Reinstall the project after activating the intended environment "
            "with `pip install -e .`, and make sure its native library directories "
            "are visible in LD_LIBRARY_PATH."
        ) from exc
    return _native


def available_qp_solvers() -> tuple[str, ...]:
    _native_module()
    return ("eiquadprog",)


def _native_options(options: IKOptions | None):
    options = IKOptions() if options is None else options
    if not isinstance(options, IKOptions):
        raise TypeError("options must be IKOptions (legacy PinkIKOptions is an alias)")
    options.validate()
    native = _native_module().Options()
    for name in options.__dataclass_fields__:
        if not hasattr(native, name):
            continue
        value = getattr(options, name)
        if name == "posture_target":
            value = [] if value is None else np.asarray(value, dtype=float).reshape(-1).tolist()
        elif name in ("position_tolerance", "orientation_tolerance"):
            value = 0.0 if value is None else value
        setattr(native, name, value)
    native.damping = max(options.damping, options.lm_damping)
    return native


def _pose_values(value, name):
    array = np.asarray(value, dtype=float)
    if array.shape != (4, 4):
        raise Tron2ConfigurationError(f"{name} must be a 4x4 matrix")
    # SO(3), finite and homogeneous checks are performed in C++.
    return array.reshape(-1).tolist()


def _batch_pose_values(value, name):
    array = np.asarray(value, dtype=float)
    if array.ndim != 3 or array.shape[1:] != (4, 4):
        raise Tron2ConfigurationError(f"{name} must have shape (batch, 4, 4)")
    if not np.isfinite(array).all():
        raise Tron2ConfigurationError(f"{name} must contain finite values")
    return [array[index].reshape(-1).tolist() for index in range(array.shape[0])]


def _arm_values(value, name="q"):
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.shape != (14,) or not np.isfinite(array).all():
        raise Tron2ConfigurationError(f"{name} must be a finite 14-element vector")
    return array.tolist()


class Tron2QPModel:
    """14-DoF facade. FK, mimic expansion, Jacobians and QP iteration run in C++."""

    left_arm_joints = LEFT_ARM_JOINTS
    right_arm_joints = RIGHT_ARM_JOINTS

    def __init__(self, urdf_path=DEFAULT_URDF, *, left_frame=DEFAULT_LEFT_FRAME,
                 right_frame=DEFAULT_RIGHT_FRAME, base_pose=None):
        self.urdf_path = Path(urdf_path).expanduser().resolve()
        if not self.urdf_path.is_file():
            raise FileNotFoundError(f"Tron2 URDF does not exist: {self.urdf_path}")
        self.left_frame, self.right_frame = left_frame, right_frame
        self.base_pose = np.eye(4) if base_pose is None else np.asarray(base_pose, dtype=float).copy()
        try:
            self._native = _native_module().Model(str(self.urdf_path), left_frame, right_frame,
                                                  _pose_values(self.base_pose, "base_pose"))
        except ValueError as exc:
            raise Tron2ConfigurationError(str(exc)) from exc

    @property
    def nq(self): return self._native.nq

    @property
    def nv(self): return self._native.nv

    @property
    def full_nq(self): return self._native.full_nq

    @property
    def full_nv(self): return self._native.full_nv

    @property
    def joint_names(self): return ARM_JOINTS

    @property
    def joint_limits(self): return np.asarray(self._native.joint_limits)

    @property
    def velocity_limits(self): return np.asarray(self._native.velocity_limits)

    @property
    def frame_names(self): return tuple(self._native.frame_names)

    @property
    def joint_coordinates(self): return dict(self._native.joint_coordinates)

    def full_configuration(self, q, options=None):
        try:
            return np.asarray(self._native.full_configuration(_arm_values(q), _native_options(options)))
        except ValueError as exc:
            raise Tron2ConfigurationError(str(exc)) from exc

    def forward_kinematics(self, q, *, side="left", options=None):
        try:
            return np.asarray(self._native.forward_kinematics(
                _arm_values(q), str(side).lower(), _native_options(options))).reshape(4, 4)
        except ValueError as exc:
            raise Tron2ConfigurationError(str(exc)) from exc

    def end_effector_poses(self, q, options=None):
        placements = self.frame_placements(q, options)
        return placements[self.left_frame], placements[self.right_frame]

    def frame_placements(self, q, options=None):
        try:
            placements = self._native.frame_placements(_arm_values(q), _native_options(options))
        except ValueError as exc:
            raise Tron2ConfigurationError(str(exc)) from exc
        return {name: np.asarray(pose).reshape(4, 4) for name, pose in placements.items()}

    def solve_ik(self, left_target, right_target, q_seed=None, options=None, *, dt=None, solver=None):
        options = IKOptions() if options is None else options
        if not isinstance(options, IKOptions):
            raise TypeError("options must be IKOptions")
        overrides = {}
        if dt is not None: overrides["dt"] = dt
        if solver is not None: overrides["qp_solver"] = solver
        options = replace(options, **overrides)
        native_options = _native_options(options)
        seed = [0.0] * 14 if q_seed is None else _arm_values(q_seed, "q_seed")
        try:
            native_result = self._native.solve_ik(_pose_values(left_target, "left_target"),
                _pose_values(right_target, "right_target"), seed, native_options)
        except ValueError as exc:
            raise Tron2ConfigurationError(str(exc)) from exc
        return self._result_from_native(native_result, options)

    def _result_from_native(self, native_result, options):
        values = {name: getattr(native_result, name) for name in IKResult.__dataclass_fields__
                  if name != "full_q"}
        native_full_q = getattr(native_result, "full_q", None)
        # Keep compatibility with an older in-place extension while avoiding
        # the extra native call when the optimized extension is loaded.
        values["full_q"] = (
            native_full_q
            if native_full_q is not None and len(native_full_q) > 0
            else self.full_configuration(native_result.q, options)
        )
        return IKResult(**values)

    def solve_ik_batch(self, left_targets, right_targets, q_seeds=None,
                       options=None, *, dt=None, solver=None):
        """Solve a batch of independent dual-arm targets with the native QP.

        Each target still runs the same constrained QP iteration as
        :meth:`solve_ik`; the batch entry point only amortizes validation,
        model locking, and the Python/C++ boundary.
        """
        options = IKOptions() if options is None else options
        if not isinstance(options, IKOptions):
            raise TypeError("options must be IKOptions")
        overrides = {}
        if dt is not None:
            overrides["dt"] = dt
        if solver is not None:
            overrides["qp_solver"] = solver
        options = replace(options, **overrides)
        native_options = _native_options(options)
        left = _batch_pose_values(left_targets, "left_targets")
        right = _batch_pose_values(right_targets, "right_targets")
        if len(left) != len(right) or not left:
            raise Tron2ConfigurationError(
                "left_targets and right_targets must have the same non-zero batch size"
            )
        if q_seeds is None:
            seeds = [[0.0] * 14 for _ in left]
        else:
            seed_array = np.asarray(q_seeds, dtype=float)
            if seed_array.shape != (len(left), 14) or not np.isfinite(seed_array).all():
                raise Tron2ConfigurationError(
                    f"q_seeds must have shape ({len(left)}, 14) and contain finite values"
                )
            seeds = seed_array.tolist()
        if not hasattr(self._native, "solve_ik_batch"):
            # Keep the same validated API when an old extension is loaded.
            return [
                self.solve_ik(left_targets[index], right_targets[index],
                              seeds[index], options)
                for index in range(len(left))
            ]
        try:
            native_results = self._native.solve_ik_batch(
                left, right, seeds, native_options
            )
        except ValueError as exc:
            raise Tron2ConfigurationError(str(exc)) from exc
        return [self._result_from_native(result, options) for result in native_results]


# Preserve existing caller imports, without retaining a Python solver path.
PinkIKOptions = IKOptions
PinkIKResult = IKResult
Tron2PinkModel = Tron2QPModel


def solve_dual_arm_ik(left_target, right_target, q_seed=None, *, urdf_path=DEFAULT_URDF,
                     left_frame=DEFAULT_LEFT_FRAME, right_frame=DEFAULT_RIGHT_FRAME,
                     base_pose=None, options=None, dt=None, solver=None):
    return Tron2QPModel(urdf_path, left_frame=left_frame, right_frame=right_frame,
                       base_pose=base_pose).solve_ik(left_target, right_target, q_seed,
                                                    options, dt=dt, solver=solver)


__all__ = ["ARM_JOINTS", "LEFT_ARM_JOINTS", "RIGHT_ARM_JOINTS", "ASSET_ROOT",
           "DEFAULT_URDF", "DEFAULT_TRON2_URDF", "DEFAULT_LEFT_FRAME", "DEFAULT_RIGHT_FRAME",
           "IKOptions", "IKResult", "Tron2QPModel", "PinkIKOptions", "PinkIKResult",
           "Tron2PinkModel", "Tron2IKError", "Tron2ConfigurationError",
           "available_qp_solvers", "solve_dual_arm_ik"]
