"""Continuous, anchored rotation gain shared by the native IK clients.

Multiplying the principal SO(3) logarithm by a fractional gain jumps at a half
turn. Retain the nearest equivalent logarithm branch instead of integrating
per-frame rotations: a held controller pose then has one stable target, without
accumulated integration drift. A single grip is limited to 270 degrees of
unscaled controller rotation to stay away from the full-turn axis singularity;
crossing it requires regrip. The vendor IK client keeps its original Clutch.
"""
import math

import numpy as np

import paths  # Existing shared input modules use their validated model paths.
from lerobot.utils.rotation import Rotation
from vr_input import Clutch, mapped_pose


MAX_CLUTCH_ROTATION_DEG = 270.


def continuous_rotation_vector(relative_rotation, previous):
    """Choose an equivalent SO(3) log nearest the prior anchored log.

The accepted state always lies below one full turn, so the principal branch
and its immediate +/-2pi neighbours cover every eligible continuation.  At
identity, the prior axis is only used to identify a prohibited full turn; it
is never used to publish a guessed full-turn target.
"""
    vector = Rotation.from_matrix(relative_rotation).as_rotvec()
    angle = float(np.linalg.norm(vector))
    previous = np.asarray(previous, dtype=float)
    if angle > 1e-10:
        axis = vector/angle
    else:
        length = float(np.linalg.norm(previous))
        axis = previous/length if length > 1e-10 else np.array([1., 0., 0.])
    candidates = [vector+2*math.pi*k*axis for k in (-1, 0, 1)]
    chosen = min(candidates, key=lambda candidate: float(np.linalg.norm(candidate-previous)))
    if np.linalg.norm(chosen) > math.radians(MAX_CLUTCH_ROTATION_DEG)+1e-10:
        raise ValueError('单次握持的手柄连续旋转超过 270°；正在暂停，请松开双侧 Grip 后重新握持')
    return chosen.copy()


class NativeClutch(Clutch):
    """Original translation/freeze semantics, with continuous anchored rotation."""
    def __init__(self, side, mode='first_person', rotation_mode=None):
        super().__init__(side, mode=mode, rotation_mode=rotation_mode)
        self.continuous_rotation = np.zeros(3)

    def reset(self):
        super().reset()
        self.continuous_rotation = np.zeros(3)

    def rebase(self, controller, accepted_tcp):
        super().rebase(controller, accepted_tcp)
        self.continuous_rotation = np.zeros(3)

    def target(self, controller, current_tcp, sensitivity, stationary=True,
               freeze_position=False, freeze_rotation=False):
        was_engaged, was_frozen = self.engaged, self.frozen_rotation
        result = super().target(controller, current_tcp, sensitivity, stationary,
                                freeze_position=freeze_position, freeze_rotation=freeze_rotation)
        if result is None:
            return None
        # The parent replaces its orientation reference on engagement, while
        # frozen, and on the unfreeze edge.  Discard the old log branch too.
        if not was_engaged or freeze_rotation or was_frozen:
            self.continuous_rotation = np.zeros(3)
        _, rotation = mapped_pose(controller['pose'], self.side, self.mode,
                                  self.rotation_mode, self.heading)
        vector = continuous_rotation_vector(rotation @ self.reference[1].T,
                                            self.continuous_rotation)
        result[:3, :3] = Rotation.from_rotvec(sensitivity*vector).as_matrix() @ self.start[:3, :3]
        self.continuous_rotation = vector
        return result
