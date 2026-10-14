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

"""BiPico4: Bimanual Pico4 VR teleoperator for any bimanual robot using the shared
world-frame TCP schema (BiFlexivRizon4RT, BiEliteCS66RT, ...).

Uses both Pico4 VR controllers simultaneously via a single xrt SDK connection.
Delegates all per-arm tracking to two Pico4 instances (left and right), sharing
one xrt handle initialised in connect() and closed in disconnect(). Output is in
the world frame (x=forward, y=left, z=up), so it drives any robot that speaks it.

Output action keys (shared world-frame TCP schema; Flexiv & Elite):
    left_tcp.{x, y, z, r1-r6}   left_gripper.pos
    right_tcp.{x, y, z, r1-r6}  right_gripper.pos

Control scheme (same for each controller):
    Grip held  -> enable arm control; controller pose maps to TCP target pose
    Grip release -> freeze arm at current target
    Trigger    -> gripper position (0 = open, 1 = closed mapped to gripper_width)
    A button (right controller) -> reset both arms to initial pose (rising edge)
    X button (left controller) -> toggle translation + rotation sensitivity: 0.25/1.0
    B button double-click -> cycle left arm: normal/rotation-only/translation-only
    Y button double-click -> cycle right arm: normal/rotation-only/translation-only
    Joystick click -> flip the arm mapped to that controller by 180° (Grip released)
"""

import time
from typing import Any

import numpy as np

from lerobot.teleoperators.bi_pico4.config_bi_pico4 import BiPico4Config
from lerobot.teleoperators.pico4 import Pico4
from lerobot.teleoperators.pico4.config_pico4 import Pico4Config
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.utils.robot_utils import get_logger, normalize_quaternion


class BiPico4(Teleoperator):
    """Bimanual Pico4 VR teleoperator for any bimanual robot on the shared
    world-frame TCP schema (BiFlexivRizon4RT, BiEliteCS66RT, ...).

    Wraps two Pico4 instances (left and right) sharing a single xrt SDK
    connection.  BiPico4.connect() initialises the SDK once, injects the
    shared handle into both Pico4 instances, and seeds their initial poses.
    BiPico4.get_action() delegates to each Pico4 and prefixes the keys with
    "left_" / "right_".

    Output action keys (world-frame TCP; Flexiv & Elite use the same):
        left_tcp.{x,y,z,r1-r6}, left_gripper.pos
        right_tcp.{x,y,z,r1-r6}, right_gripper.pos

    Example:
        >>> config = BiPico4Config()
        >>> teleop = BiPico4(config)
        >>> robot = BiFlexivRizon4RT(robot_config)
        >>> robot.connect()
        >>> left_pose, right_pose = robot.get_current_tcp_pose_quat()
        >>> teleop.connect(left_tcp_pose_quat=left_pose, right_tcp_pose_quat=right_pose)
        >>> while True:
        ...     action = teleop.get_action()
        ...     robot.send_action(action)
    """

    config_class = BiPico4Config
    name = "bi_pico4"

    def __init__(self, config: BiPico4Config):
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self._xrt = None
        self.logger = get_logger("BiPico4")
        self._swap_controllers = config.control_mode in {
            "mirror",
            "mirror_same_direction",
        }

        # Edge/double-click detection for runtime controller bindings.
        self._was_reset_button_pressed: bool = False
        self._was_sensitivity_button_pressed: bool = False
        self._was_b_button_pressed: bool = False
        self._last_b_press_time: float | None = None
        self._was_y_button_pressed: bool = False
        self._last_y_press_time: float | None = None
        self._was_left_wrist_flip_button_pressed: bool = False
        self._was_right_wrist_flip_button_pressed: bool = False
        self._pending_wrist_flip_requests: set[str] = set()

        # Build per-arm Pico4 configs from shared BiPico4Config fields
        # The Pico4 object names always refer to the target robot arm. Mirror
        # mode swaps which physical controller supplies each target arm.
        initial_ori_sensitivity = (
            config.pos_sensitivity
            if config.ori_sensitivity is None
            else config.ori_sensitivity
        )
        left_config = Pico4Config(
            id="bi_pico4_left",
            use_left_controller=not self._swap_controllers,
            use_right_controller=self._swap_controllers,
            pos_sensitivity=config.pos_sensitivity,
            ori_sensitivity=initial_ori_sensitivity,
            filter_window_size=config.filter_window_size,
            max_pos_velocity=config.max_pos_velocity,
            max_rot_velocity=config.max_rot_velocity,
            gripper_width=config.left_gripper_width,
            grip_enable_threshold=config.grip_enable_threshold,
            grip_disable_threshold=config.grip_disable_threshold,
            orientation_offset_warning_deg=config.orientation_offset_warning_deg,
            position_jump_threshold=config.position_jump_threshold,
            pose_mapping=config.control_mode,
        )
        right_config = Pico4Config(
            id="bi_pico4_right",
            use_left_controller=self._swap_controllers,
            use_right_controller=not self._swap_controllers,
            pos_sensitivity=config.pos_sensitivity,
            ori_sensitivity=initial_ori_sensitivity,
            filter_window_size=config.filter_window_size,
            max_pos_velocity=config.max_pos_velocity,
            max_rot_velocity=config.max_rot_velocity,
            gripper_width=config.right_gripper_width,
            grip_enable_threshold=config.grip_enable_threshold,
            grip_disable_threshold=config.grip_disable_threshold,
            orientation_offset_warning_deg=config.orientation_offset_warning_deg,
            position_jump_threshold=config.position_jump_threshold,
            pose_mapping=config.control_mode,
        )
        self._left_pico4 = Pico4(left_config)
        self._right_pico4 = Pico4(right_config)

    @property
    def _left_controller_pico4(self) -> Pico4:
        return self._right_pico4 if self._swap_controllers else self._left_pico4

    @property
    def _right_controller_pico4(self) -> Pico4:
        return self._left_pico4 if self._swap_controllers else self._right_pico4

    def pre_init(self) -> None:
        """Initialize XenseVR SDK and wait for controllers early (without TCP poses).

        Call this in a background thread while the robot is connecting/moving to
        start position, then call connect() with the TCP poses once the robot is ready.
        The SDK init and controller validation (~3s) will overlap with robot startup.
        """
        if self._xrt is not None:
            return  # Already pre-initialized
        try:
            import xensevr_pc_service_sdk as xrt
        except ImportError as e:
            raise ImportError(
                "xensevr_pc_service_sdk is required. "
                "Please install it according to the Pico4 SDK documentation."
            ) from e

        xrt.init()
        self._xrt = xrt
        self.logger.info("XenseVR SDK pre-initialized.")

        time.sleep(0.5)
        max_retries = 25
        for attempt in range(max_retries):
            left_pose = xrt.get_left_controller_pose()
            right_pose = xrt.get_right_controller_pose()
            left_ok = any(abs(v) > 1e-6 for v in left_pose)
            right_ok = any(abs(v) > 1e-6 for v in right_pose)
            if left_ok and right_ok:
                self.logger.info(f"Both controllers ready after pre_init (attempt {attempt + 1})")
                return
            time.sleep(0.1)

        self._xrt = None
        raise DeviceNotConnectedError(
            "Pico4 controllers not detected after waiting. "
            "Ensure the VR client app is running and controllers are on."
        )

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return self._is_connected

    @property
    def action_features(self) -> dict[str, Any]:
        """Action features: bimanual world-frame TCP (Flexiv & Elite share this)."""
        return {
            "dtype": "float32",
            "shape": (20,),
            "names": {
                "left_tcp.x": 0,
                "left_tcp.y": 1,
                "left_tcp.z": 2,
                "left_tcp.r1": 3,
                "left_tcp.r2": 4,
                "left_tcp.r3": 5,
                "left_tcp.r4": 6,
                "left_tcp.r5": 7,
                "left_tcp.r6": 8,
                "left_gripper.pos": 9,
                "right_tcp.x": 10,
                "right_tcp.y": 11,
                "right_tcp.z": 12,
                "right_tcp.r1": 13,
                "right_tcp.r2": 14,
                "right_tcp.r3": 15,
                "right_tcp.r4": 16,
                "right_tcp.r5": 17,
                "right_tcp.r6": 18,
                "right_gripper.pos": 19,
            },
        }

    @property
    def feedback_features(self) -> dict[str, Any]:
        return {}

    def connect(
        self,
        calibrate: bool = True,
        left_tcp_pose_quat: np.ndarray | None = None,
        right_tcp_pose_quat: np.ndarray | None = None,
    ) -> None:
        """Connect to Pico4 VR via xrt SDK (single shared connection for both arms).

        Args:
            calibrate: Unused, kept for interface compatibility.
            left_tcp_pose_quat: Left arm current TCP pose [x,y,z,qw,qx,qy,qz,gripper_pos].
            right_tcp_pose_quat: Right arm current TCP pose [x,y,z,qw,qx,qy,qz,gripper_pos].
        """
        if left_tcp_pose_quat is None:
            left_tcp_pose_quat = np.zeros(8, dtype=np.float32)
        if right_tcp_pose_quat is None:
            right_tcp_pose_quat = np.zeros(8, dtype=np.float32)
        if self._is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected.")

        self.logger.info("Connecting to Pico4 VR (bimanual)...")
        try:
            import xensevr_pc_service_sdk as xrt
        except ImportError as e:
            raise ImportError(
                "xensevr_pc_service_sdk is required. "
                "Please install it according to the Pico4 SDK documentation."
            ) from e

        try:
            if self._xrt is None:
                # Not pre-initialized — do full init now (slower path)
                xrt.init()
                self._xrt = xrt
                self.logger.info("XenseVR SDK initialized.")

                # Wait for both controllers to report valid pose data
                time.sleep(0.5)
                max_retries = 25
                for attempt in range(max_retries):
                    left_pose = xrt.get_left_controller_pose()
                    right_pose = xrt.get_right_controller_pose()
                    left_ok = any(abs(v) > 1e-6 for v in left_pose)
                    right_ok = any(abs(v) > 1e-6 for v in right_pose)
                    if left_ok and right_ok:
                        self.logger.info(f"Both controllers ready (attempt {attempt + 1})")
                        break
                    time.sleep(0.1)
                else:
                    self._xrt = None
                    raise DeviceNotConnectedError(
                        "Pico4 controllers not detected after waiting. "
                        "Ensure the VR client app is running and controllers are on."
                    )
            else:
                # pre_init() was already called — reuse the existing xrt handle
                xrt = self._xrt
                self.logger.info("XenseVR SDK already pre-initialized, skipping init.")

            # Inject shared xrt handle and seed initial state into both Pico4 instances,
            # bypassing their connect() to avoid calling xrt.init() a second time.
            self._init_pico4_instance(self._left_pico4, xrt, left_tcp_pose_quat)
            self._init_pico4_instance(self._right_pico4, xrt, right_tcp_pose_quat)

            self._is_connected = True

            # Seed edge-detection state from the actual current A-button state so
            # that a held button at connect time does not fire as a false
            # "just pressed" event on the first poll_buttons() call.
            self.poll_buttons()
            self._was_reset_button_pressed = self._right_controller_pico4._last_a_button
            self._was_sensitivity_button_pressed = self._left_controller_pico4._last_x_button
            self._was_b_button_pressed = self._right_controller_pico4._last_b_button
            self._last_b_press_time = None
            self._was_y_button_pressed = self._left_controller_pico4._last_y_button
            self._last_y_press_time = None
            self._was_left_wrist_flip_button_pressed = self._left_pico4._last_axis_click
            self._was_right_wrist_flip_button_pressed = self._right_pico4._last_axis_click
            self._pending_wrist_flip_requests.clear()

            mapping = (
                "right->left, left->right"
                if self._swap_controllers
                else "left->left, right->right"
            )
            self.logger.info(
                f"BiPico4 connected (mode={self.config.control_mode}, controller mapping={mapping})."
            )

        except DeviceNotConnectedError:
            raise
        except Exception as e:
            self._xrt = None
            raise DeviceNotConnectedError(f"Failed to connect BiPico4: {e}") from e

    def _init_pico4_instance(
        self, pico4: Pico4, xrt, tcp_pose_quat: np.ndarray
    ) -> None:
        """Inject shared xrt handle and seed initial pose into a Pico4 instance.

        This mirrors what Pico4.connect() does after xrt.init(), so we avoid
        initialising the SDK a second time for the second arm.

        Args:
            pico4: The Pico4 instance to initialise.
            xrt: The already-initialised xrt SDK module.
            tcp_pose_quat: Initial TCP pose [x,y,z,qw,qx,qy,qz,gripper_pos].
        """
        pico4._xrt = xrt
        pico4._target_pos = tcp_pose_quat[:3].copy().astype(np.float32)
        pico4._target_quat = normalize_quaternion(tcp_pose_quat[3:7], input_format="wxyz")
        pico4._target_gripper_pos = float(tcp_pose_quat[7])
        pico4._start_pos = tcp_pose_quat[:3].copy().astype(np.float32)
        pico4._ref_pos = None
        pico4._quat_offset = None
        pico4._enabled = False
        pico4._was_enabled = False
        pico4._orientation_control_active = True
        pico4._motion_constraint = "normal"
        pico4._last_controller_pos_world = None
        pico4._last_controller_quat_world = None
        pico4._last_raw_pose = None
        pico4._last_a_button = False
        pico4._last_b_button = False
        pico4._last_x_button = False
        pico4._last_y_button = False
        pico4._last_axis_click = False
        pico4._motion_input_suspended = False
        pico4._resume_requires_grip_release = False
        pico4._is_connected = True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def get_action(self) -> dict[str, Any]:
        """Get action from both controllers.

        Delegates to each Pico4 instance and prefixes the returned keys with
        "left_" / "right_" to match the bimanual world-frame TCP action space.

        Returns:
            Flat dict with keys: left_tcp.{x,y,z,r1-r6}, left_gripper.pos,
            right_tcp.{x,y,z,r1-r6}, right_gripper.pos.
        """
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        left_action = self._left_pico4.get_action()
        right_action = self._right_pico4.get_action()
        self._handle_runtime_buttons()

        return {f"left_{k}": v for k, v in left_action.items()} | {
            f"right_{k}": v for k, v in right_action.items()
        }

    @property
    def position_sensitivity(self) -> float:
        return float(self.config.pos_sensitivity)

    @property
    def motion_sensitivity(self) -> float:
        """Shared runtime translation/orientation sensitivity."""
        return float(self.config.pos_sensitivity)

    @property
    def right_orientation_frozen(self) -> bool:
        return self._right_pico4.orientation_frozen

    @property
    def left_motion_constraint(self) -> str:
        return self._left_pico4.motion_constraint

    @property
    def right_motion_constraint(self) -> str:
        return self._right_pico4.motion_constraint

    def _handle_runtime_buttons(self) -> None:
        """Apply sensitivity, constraint, and per-arm wrist-flip bindings."""
        sensitivity_pressed = self._left_controller_pico4._last_x_button
        sensitivity_just_pressed = (
            sensitivity_pressed and not self._was_sensitivity_button_pressed
        )
        self._was_sensitivity_button_pressed = sensitivity_pressed
        if sensitivity_just_pressed:
            self._cycle_motion_sensitivity()

        (
            self._was_b_button_pressed,
            self._last_b_press_time,
        ) = self._handle_constraint_button(
            pressed=self._right_controller_pico4._last_b_button,
            was_pressed=self._was_b_button_pressed,
            previous_press_time=self._last_b_press_time,
            pico4=self._left_pico4,
            arm_label="Left arm",
            button_label="B",
        )
        (
            self._was_y_button_pressed,
            self._last_y_press_time,
        ) = self._handle_constraint_button(
            pressed=self._left_controller_pico4._last_y_button,
            was_pressed=self._was_y_button_pressed,
            previous_press_time=self._last_y_press_time,
            pico4=self._right_pico4,
            arm_label="Right arm",
            button_label="Y",
        )

        self._was_left_wrist_flip_button_pressed = self._handle_wrist_flip_button(
            side="left",
            pico4=self._left_pico4,
            was_pressed=self._was_left_wrist_flip_button_pressed,
        )
        self._was_right_wrist_flip_button_pressed = self._handle_wrist_flip_button(
            side="right",
            pico4=self._right_pico4,
            was_pressed=self._was_right_wrist_flip_button_pressed,
        )

    def _handle_wrist_flip_button(
        self,
        *,
        side: str,
        pico4: Pico4,
        was_pressed: bool,
    ) -> bool:
        pressed = pico4._last_axis_click
        just_pressed = pressed and not was_pressed
        if not just_pressed:
            return pressed

        if pico4._enabled or pico4._last_grip > pico4.config.grip_disable_threshold:
            self.logger.warn(
                f"{side.capitalize()} wrist flip ignored: release Grip before clicking the joystick."
            )
        elif pico4._motion_input_suspended or pico4._resume_requires_grip_release:
            self.logger.warn(
                f"{side.capitalize()} wrist flip ignored: arm is suspended or waiting for Grip release."
            )
        else:
            self._pending_wrist_flip_requests.add(side)
        return pressed

    def pop_wrist_flip_requests(self) -> tuple[str, ...]:
        """Return and clear joystick-click wrist-flip requests."""
        requests = tuple(
            side for side in ("left", "right") if side in self._pending_wrist_flip_requests
        )
        self._pending_wrist_flip_requests.clear()
        return requests

    def _handle_constraint_button(
        self,
        *,
        pressed: bool,
        was_pressed: bool,
        previous_press_time: float | None,
        pico4: Pico4,
        arm_label: str,
        button_label: str,
    ) -> tuple[bool, float | None]:
        just_pressed = pressed and not was_pressed
        if not just_pressed:
            return pressed, previous_press_time

        now = time.monotonic()
        if (
            previous_press_time is None
            or now - previous_press_time > self.config.button_double_click_interval_s
        ):
            return pressed, now

        next_constraint = {
            "normal": "freeze_translation",
            "freeze_translation": "freeze_orientation",
            "freeze_orientation": "normal",
        }[pico4.motion_constraint]
        pico4.set_motion_constraint(next_constraint)
        state_label = {
            "normal": "NORMAL",
            "freeze_orientation": "ORIENTATION FROZEN (translation only)",
            "freeze_translation": "TRANSLATION FROZEN (rotation only)",
        }[next_constraint]
        self.logger.info(f"{arm_label}: {state_label} ({button_label} double-click).")
        return pressed, None

    def _cycle_motion_sensitivity(self) -> None:
        levels = tuple(float(value) for value in self.config.position_sensitivity_levels)
        if not levels or any(value <= 0.0 for value in levels):
            raise ValueError(
                "position_sensitivity_levels must contain positive values, "
                f"got {self.config.position_sensitivity_levels}."
            )

        current = float(self.config.pos_sensitivity)
        current_index = next(
            (index for index, value in enumerate(levels) if np.isclose(value, current)),
            -1,
        )
        new_sensitivity = levels[(current_index + 1) % len(levels)]
        self.config.pos_sensitivity = new_sensitivity
        self.config.ori_sensitivity = new_sensitivity
        self._left_pico4.set_position_sensitivity(new_sensitivity)
        self._right_pico4.set_position_sensitivity(new_sensitivity)
        self._left_pico4.set_orientation_sensitivity(new_sensitivity)
        self._right_pico4.set_orientation_sensitivity(new_sensitivity)
        self.logger.info(
            f"Motion sensitivity: {new_sensitivity:.2f} "
            f"(translation and rotation, X button)."
        )

    def reset_to_pose(
        self,
        left_pose_7d: np.ndarray,
        right_pose_7d: np.ndarray,
        left_gripper_pos: float = 0.0,
        right_gripper_pos: float = 0.0,
    ) -> None:
        """Reset both arm target poses (e.g. after robot moves to start position).

        Args:
            left_pose_7d: [x, y, z, qw, qx, qy, qz] in world frame.
            right_pose_7d: [x, y, z, qw, qx, qy, qz] in world frame.
            left_gripper_pos: Left gripper position.
            right_gripper_pos: Right gripper position.
        """
        self._left_pico4.reset_to_pose(left_pose_7d, left_gripper_pos)
        self._right_pico4.reset_to_pose(right_pose_7d, right_gripper_pos)

    def suspend_arm_motion(self, side: str) -> None:
        """Suspend one mapped controller while its robot arm executes a trajectory."""
        self._pico4_for_side(side).suspend_motion_input()

    def complete_arm_wrist_flip(
        self,
        side: str,
        pose_7d: np.ndarray,
        gripper_pos: float,
    ) -> None:
        """Rebase one arm after a flip and require Grip release before resuming."""
        pico4 = self._pico4_for_side(side)
        pico4.reset_to_pose(pose_7d, gripper_pos)
        pico4.resume_motion_after_grip_release()

    def _pico4_for_side(self, side: str) -> Pico4:
        if side == "left":
            return self._left_pico4
        if side == "right":
            return self._right_pico4
        raise ValueError(f"Unknown arm side {side!r}; expected 'left' or 'right'.")

    def reset_motion_constraints(self) -> None:
        """Restore both arms to normal mapping and clear partial double-clicks."""
        constraints_changed = (
            self._left_pico4.motion_constraint != "normal"
            or self._right_pico4.motion_constraint != "normal"
        )
        self._left_pico4.set_motion_constraint("normal")
        self._right_pico4.set_motion_constraint("normal")
        self._last_b_press_time = None
        self._last_y_press_time = None
        self._was_b_button_pressed = self._right_controller_pico4._last_b_button
        self._was_y_button_pressed = self._left_controller_pico4._last_y_button
        if constraints_changed:
            self.logger.info("Both arm motion constraints reset to NORMAL (A/Home).")

    def poll_buttons(self) -> None:
        """Refresh cached controller button states without a full pose/action computation.

        Call this at the top of a control loop when button state is needed before get_action().
        get_action() will re-read the same buttons, which is safe (idempotent reads).
        A/B are read from the physical right controller and X/Y from the physical
        left controller, independent of normal/mirror arm mapping.
        """
        if not self._is_connected or self._xrt is None:
            return
        self._right_controller_pico4._last_a_button = bool(self._xrt.get_A_button())
        self._right_controller_pico4._last_b_button = bool(self._xrt.get_B_button())
        self._left_controller_pico4._last_x_button = bool(self._xrt.get_X_button())
        self._left_controller_pico4._last_y_button = bool(self._xrt.get_Y_button())
        self._left_controller_pico4._last_axis_click = bool(self._xrt.get_left_axis_click())
        self._right_controller_pico4._last_axis_click = bool(self._xrt.get_right_axis_click())

    def get_reset_button(self) -> bool:
        """Get rising-edge state of the A button (right controller).

        Returns True only on the first frame the button is pressed.
        Both arms share the same reset button — pressing A resets both simultaneously.

        Note: Call get_action() before this method each frame so that the right
        Pico4 instance has polled the SDK and cached the A button state.
        """
        current = self._right_controller_pico4._last_a_button
        just_pressed = current and not self._was_reset_button_pressed
        self._was_reset_button_pressed = current
        if just_pressed:
            self.reset_motion_constraints()
        return just_pressed

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        raise NotImplementedError("Feedback is not supported by BiPico4.")

    def disconnect(self) -> None:
        if not self._is_connected or self._xrt is None:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.logger.info("Closing XenseVR SDK...")

        # Mark both Pico4 instances as disconnected before closing the shared handle
        self._left_pico4._is_connected = False
        self._left_pico4._xrt = None
        self._right_pico4._is_connected = False
        self._right_pico4._xrt = None

        try:
            self._xrt.close()
        except Exception as e:
            self.logger.warning(f"Error closing XenseVR SDK: {e}")
        finally:
            self._is_connected = False
            self._xrt = None
            self.logger.info("BiPico4 disconnected.")

    def __del__(self):
        if self._is_connected:
            try:
                self.disconnect()
            except Exception:
                pass
            finally:
                self._is_connected = False
                self._xrt = None
