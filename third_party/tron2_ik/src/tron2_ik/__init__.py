"""Independent C++ QP inverse kinematics for the Tron2 dual-arm robot."""

from .runtime import _preload_native_dependencies

_preload_native_dependencies()

from .solver import (
    IKOptions,
    IKResult,
    Tron2QPModel,
    ARM_JOINTS,
    ASSET_ROOT,
    DEFAULT_LEFT_FRAME,
    DEFAULT_RIGHT_FRAME,
    DEFAULT_TRON2_URDF,
    DEFAULT_URDF,
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
    PinkIKOptions,
    PinkIKResult,
    Tron2ConfigurationError,
    Tron2IKError,
    Tron2PinkModel,
    available_qp_solvers,
    solve_dual_arm_ik,
)

__version__ = "0.1.0"

__all__ = [
    "IKOptions", "IKResult", "Tron2QPModel",
    "ARM_JOINTS",
    "ASSET_ROOT",
    "DEFAULT_LEFT_FRAME",
    "DEFAULT_RIGHT_FRAME",
    "DEFAULT_TRON2_URDF",
    "DEFAULT_URDF",
    "LEFT_ARM_JOINTS",
    "PinkIKOptions",
    "PinkIKResult",
    "RIGHT_ARM_JOINTS",
    "Tron2ConfigurationError",
    "Tron2IKError",
    "Tron2PinkModel",
    "available_qp_solvers",
    "solve_dual_arm_ik",
]
