#!/usr/bin/env python

# Copyright 2025 The XenseRobotics Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("bi_pico4")
@dataclass
class BiPico4Config(TeleoperatorConfig):
    """Configuration for BiPico4 dual-controller VR teleoperator.

    Uses both Pico4 VR controllers simultaneously:
    - Left controller  -> left arm  (left_tcp.*, left_gripper.pos)
    - Right controller -> right arm (right_tcp.*, right_gripper.pos)

    Output keys are the shared world-frame TCP schema (BiFlexivRizon4RT, BiEliteCS66RT).

    Attributes:
        control_mode: ``first_person`` keeps the existing controller-to-arm
            mapping. ``mirror`` mirrors the pose and swaps the two controllers,
            so an operator facing the robot sees it behave like a reflection.
            ``mirror_same_direction`` keeps the swapped controllers but maps
            translation and rotation in the same physical room direction as
            the operator's hand.
        pos_sensitivity: Shared position/orientation scale factor used when
            ``ori_sensitivity`` is omitted (1.0 = 1:1 mapping).
        ori_sensitivity: Optional initial orientation override for legacy or
            robot-specific use. X-button changes always recouple both values.
        position_sensitivity_levels: Translation/orientation sensitivity values
            cycled by the X button.
        filter_window_size: Moving average filter window for smoothing pose.
        left_gripper_width: Max gripper position for left arm control space.
        right_gripper_width: Max gripper position for right arm control space.
        grip_enable_threshold: Grip value to start control (hysteresis high).
        grip_disable_threshold: Grip value to stop control (hysteresis low).
        orientation_offset_warning_deg: Max allowed controller-robot orientation
            difference at enable time. If exceeded, orientation control is disabled.
        position_jump_threshold: Max allowed position change per frame (m).
    """

    id: str = "bi_pico4"

    control_mode: str = "first_person"
    pos_sensitivity: float = 1.0
    ori_sensitivity: float | None = None
    position_sensitivity_levels: tuple[float, ...] = (0.25, 1.0)
    button_double_click_interval_s: float = 0.4
    filter_window_size: int = 1

    # Per-arm output rate limiter: caps how fast the target pose may translate/
    # rotate regardless of controller input. Lower these for arms with tighter
    # joint-velocity limits (e.g. Elite CS66) so VR jitter can't drive the
    # controller into a protective stop. 0 disables the cap.
    max_pos_velocity: float = 1.0   # m/s
    max_rot_velocity: float = 6.28  # rad/s

    left_gripper_width: float = 1.0
    right_gripper_width: float = 1.0

    grip_enable_threshold: float = 0.5
    grip_disable_threshold: float = 0.3
    orientation_offset_warning_deg: float = 180.0
    position_jump_threshold: float = 0.1

    def __post_init__(self) -> None:
        if self.control_mode not in {
            "first_person",
            "mirror",
            "mirror_same_direction",
        }:
            raise ValueError(
                "control_mode must be 'first_person', 'mirror' or "
                "'mirror_same_direction', "
                f"got {self.control_mode!r}."
            )
        if self.pos_sensitivity <= 0.0:
            raise ValueError(f"pos_sensitivity must be positive, got {self.pos_sensitivity}.")
        if self.ori_sensitivity is not None and self.ori_sensitivity <= 0.0:
            raise ValueError(f"ori_sensitivity must be positive, got {self.ori_sensitivity}.")
        if not self.position_sensitivity_levels or any(
            value <= 0.0 for value in self.position_sensitivity_levels
        ):
            raise ValueError(
                "position_sensitivity_levels must contain positive values, "
                f"got {self.position_sensitivity_levels}."
            )
        if self.button_double_click_interval_s <= 0.0:
            raise ValueError(
                "button_double_click_interval_s must be positive, "
                f"got {self.button_double_click_interval_s}."
            )
