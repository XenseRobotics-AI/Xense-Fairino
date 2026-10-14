#!/usr/bin/env python

# Copyright 2026 XenseRobotics Inc. team. All rights reserved.
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

"""Interactively record read-only bimanual Flexiv poses as JSON snapshots.

This program deliberately does not instantiate ``BiFlexivRizon4RT`` because
that class enables the arms, switches modes, starts Cartesian RT control and
moves home during disconnect. Here the Flexiv interfaces are used only through
``states()``, and the grippers receive status-query packets only.
"""

import argparse
import json
import re
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from glob import glob
from pathlib import Path
from typing import Any

import numpy as np


JOINT_COUNT = 7
POSE_FILE_PATTERN = re.compile(r"pose(\d+)\.json$")


class ReadOnlyGripper:
    """Serial gripper connection that never sends a motion command."""

    def __init__(
        self,
        *,
        sn: str,
        baudrate: int,
        serial_timeout: float,
        device_id: int,
        min_position_mm: float,
        max_position_mm: float,
    ) -> None:
        self.sn = sn
        self.baudrate = baudrate
        self.serial_timeout = serial_timeout
        self.device_id = device_id
        self.min_position_mm = min_position_mm
        self.max_position_mm = max_position_mm
        self.port = ""
        self._device: Any | None = None

    def connect(self) -> None:
        # Import only when a real connection is requested. This keeps --help
        # free of hardware/network SDK initialization side effects.
        from xensegripper import XenseSerialGripper, read_board_sn

        candidates = sorted(glob("/dev/ttyUSB*") + glob("/dev/ttyACM*"))
        for port in candidates:
            try:
                found_sn = read_board_sn(
                    port,
                    baudrate=self.baudrate,
                    device_id=self.device_id,
                )
            except Exception:
                continue
            if found_sn and found_sn.strip() == self.sn.strip():
                self.port = port
                break
        if not self.port:
            raise RuntimeError(
                f"Could not find gripper SN={self.sn!r}; scanned ports: {candidates}."
            )

        self._device = XenseSerialGripper(
            port=self.port,
            device_id=self.device_id,
            baudrate=self.baudrate,
            timeout=self.serial_timeout,
        )

    def read_position(self) -> tuple[float, float]:
        """Return ``(normalized_position, position_mm)`` from status queries."""
        if self._device is None:
            raise RuntimeError(f"Gripper SN={self.sn} is not connected.")

        status = None
        for _ in range(5):
            status = self._device.get_gripper_status(timeout=0.2)
            if status is not None and status.get("position") is not None:
                break
        if status is None or status.get("position") is None:
            raise RuntimeError(f"No status response from gripper SN={self.sn} on {self.port}.")

        position_mm = float(status["position"])
        span = self.max_position_mm - self.min_position_mm
        normalized = (position_mm - self.min_position_mm) / span
        normalized = float(np.clip(normalized, 0.0, 1.0))
        return normalized, position_mm

    def close(self) -> None:
        if self._device is not None:
            # XenseSerialGripper.release() only closes the serial transport. It
            # does not issue the open/stop/move commands used by SerialGripper.disconnect().
            self._device.release()
            self._device = None


def _field_descriptions() -> dict[str, str]:
    fields: dict[str, str] = {}
    for side_name, side_label in (("left", "左臂"), ("right", "右臂")):
        for joint_index in range(JOINT_COUNT):
            fields[f"{side_name}_A{joint_index}"] = (
                f"{side_label}关节 A{joint_index} 的实测连杆侧角度，单位 degree"
            )
        fields.update(
            {
                f"{side_name}_tcp_x": f"{side_label} TCP 在机器人世界坐标系中的 X，单位 m",
                f"{side_name}_tcp_y": f"{side_label} TCP 在机器人世界坐标系中的 Y，单位 m",
                f"{side_name}_tcp_z": f"{side_label} TCP 在机器人世界坐标系中的 Z，单位 m",
                f"{side_name}_tcp_qw": f"{side_label} TCP 姿态四元数实部 qw，无单位",
                f"{side_name}_tcp_qx": f"{side_label} TCP 姿态四元数 qx，无单位",
                f"{side_name}_tcp_qy": f"{side_label} TCP 姿态四元数 qy，无单位",
                f"{side_name}_tcp_qz": f"{side_label} TCP 姿态四元数 qz，无单位",
                f"{side_name}_gripper": (
                    f"{side_label}夹爪归一化实测开度：0=完全闭合，1=完全张开"
                ),
                f"{side_name}_gripper_mm": f"{side_label}夹爪实测开度，单位 mm",
            }
        )
    return fields


def _arm_pose(side: str, robot_state: Any) -> dict[str, float]:
    joint_positions = np.asarray(robot_state.q, dtype=np.float64)
    if joint_positions.size < JOINT_COUNT:
        raise RuntimeError(
            f"{side} arm returned {joint_positions.size} joints; expected at least {JOINT_COUNT}."
        )
    # Flexiv places any external axes before the seven manipulator joints.
    joint_degrees = np.rad2deg(joint_positions[-JOINT_COUNT:])

    tcp_pose = np.asarray(robot_state.tcp_pose, dtype=np.float64)
    if tcp_pose.shape != (7,):
        raise RuntimeError(f"{side} arm returned invalid TCP pose shape {tcp_pose.shape}.")

    values = {
        f"{side}_A{index}": float(value)
        for index, value in enumerate(joint_degrees)
    }
    for name, value in zip(
        ("tcp_x", "tcp_y", "tcp_z", "tcp_qw", "tcp_qx", "tcp_qy", "tcp_qz"),
        tcp_pose,
        strict=True,
    ):
        values[f"{side}_{name}"] = float(value)
    return values


def _next_pose_path(session_dir: Path) -> Path:
    existing_indices = []
    for path in session_dir.glob("pose*.json"):
        match = POSE_FILE_PATTERN.fullmatch(path.name)
        if match:
            existing_indices.append(int(match.group(1)))
    next_index = max(existing_indices, default=0) + 1
    return session_dir / f"pose{next_index:02d}.json"


def _make_session_dir(output_dir: Path, requested_name: str | None) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    if requested_name is not None:
        if Path(requested_name).name != requested_name or requested_name in {".", ".."}:
            raise ValueError("--session-name must be one folder name, without '/' or '..'.")
        session_dir = output_dir / requested_name
        session_dir.mkdir(parents=False, exist_ok=True)
        return session_dir

    base_name = datetime.now().astimezone().strftime("session_%Y%m%d_%H%M%S")
    session_dir = output_dir / base_name
    suffix = 1
    while session_dir.exists():
        session_dir = output_dir / f"{base_name}_{suffix:02d}"
        suffix += 1
    session_dir.mkdir()
    return session_dir


def _connect_arm(sn: str, retries: int, retry_interval_s: float) -> Any:
    # Construction opens the state transport. Do not call Enable, SwitchMode,
    # Stop, ExecutePrimitive or any RT-control method in this program.
    import flexiv_rt as frt

    return frt.Robot(
        sn,
        connect_retries=retries,
        retry_interval_sec=retry_interval_s,
    )


def _capture_snapshot(
    left_robot: Any,
    right_robot: Any,
    left_gripper: ReadOnlyGripper,
    right_gripper: ReadOnlyGripper,
    config: Any,
) -> dict[str, Any]:
    # Read the two controllers and two serial devices concurrently to reduce
    # left/right timestamp skew. All four calls are state reads only.
    with ThreadPoolExecutor(max_workers=4) as executor:
        left_state_future = executor.submit(left_robot.states)
        right_state_future = executor.submit(right_robot.states)
        left_gripper_future = executor.submit(left_gripper.read_position)
        right_gripper_future = executor.submit(right_gripper.read_position)
        left_state = left_state_future.result()
        right_state = right_state_future.result()
        left_gripper_position = left_gripper_future.result()
        right_gripper_position = right_gripper_future.result()

    pose = _arm_pose("left", left_state)
    pose.update(_arm_pose("right", right_state))
    pose["left_gripper"], pose["left_gripper_mm"] = left_gripper_position
    pose["right_gripper"], pose["right_gripper_mm"] = right_gripper_position

    return {
        "metadata": {
            "format_version": 1,
            "recorded_at": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "read_only": True,
            "bi_mount_type": config.bi_mount_type,
            "devices": {
                "left_robot_sn": config.left_robot_sn,
                "right_robot_sn": config.right_robot_sn,
                "left_gripper_sn": config.left_gripper_sn,
                "right_gripper_sn": config.right_gripper_sn,
                "left_gripper_port": left_gripper.port,
                "right_gripper_port": right_gripper.port,
            },
            "field_descriptions": _field_descriptions(),
        },
        "pose": pose,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Read and record bimanual Flexiv joint/TCP/gripper poses without commanding motion."
    )
    parser.add_argument(
        "--bi-mount-type",
        default="forward-05",
        help="BiFlexiv device preset (default: forward-05).",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("recorded_poses"),
        help="Parent folder for the newly created session subfolder (default: ./recorded_poses).",
    )
    parser.add_argument(
        "--session-name",
        default=None,
        help="Optional session subfolder name; otherwise a timestamped name is created.",
    )
    args = parser.parse_args()

    # Delayed so ``--help`` never initializes optional camera/gripper/network SDKs.
    from lerobot.robots.bi_flexiv_rizon4_rt.config_bi_flexiv_rizon4_rt import (
        BiFlexivRizon4RTConfig,
    )

    config = BiFlexivRizon4RTConfig(
        bi_mount_type=args.bi_mount_type,
        enable_tactile_sensors=False,
    )
    if not config.left_gripper or not config.right_gripper:
        raise RuntimeError("The selected preset must configure both grippers.")

    session_dir = _make_session_dir(args.output_dir.expanduser().resolve(), args.session_name)
    print(f"保存目录：{session_dir}")
    print("只读模式：不会 Enable、切换模式、发送运动命令、回 Home 或开合夹爪。")
    print("请先关闭 lerobot-teleoperate 和其他占用夹爪串口的程序。")

    left_robot: Any | None = None
    right_robot: Any | None = None
    left_gripper = ReadOnlyGripper(
        sn=config.left_gripper.sn or "",
        baudrate=config.left_gripper.baudrate,
        serial_timeout=config.left_gripper.serial_timeout,
        device_id=config.left_gripper.device_id,
        min_position_mm=config.left_gripper.gripper_min_pos,
        max_position_mm=config.left_gripper.gripper_max_pos,
    )
    right_gripper = ReadOnlyGripper(
        sn=config.right_gripper.sn or "",
        baudrate=config.right_gripper.baudrate,
        serial_timeout=config.right_gripper.serial_timeout,
        device_id=config.right_gripper.device_id,
        min_position_mm=config.right_gripper.gripper_min_pos,
        max_position_mm=config.right_gripper.gripper_max_pos,
    )

    try:
        print("正在建立两臂只读状态连接……")
        with ThreadPoolExecutor(max_workers=2) as executor:
            left_future = executor.submit(
                _connect_arm,
                config.left_robot_sn,
                config.connect_retries,
                config.retry_interval_sec,
            )
            right_future = executor.submit(
                _connect_arm,
                config.right_robot_sn,
                config.connect_retries,
                config.retry_interval_sec,
            )
            left_robot = left_future.result()
            right_robot = right_future.result()

        print("正在连接两个夹爪（仅查询状态）……")
        left_gripper.connect()
        right_gripper.connect()
        print("连接完成。按 Enter 记录一个点，输入 q 后按 Enter 退出。")

        while True:
            command = input("记录> ").strip().lower()
            if command in {"q", "quit", "exit"}:
                break
            if command:
                print("未知输入；按 Enter 记录，输入 q 退出。")
                continue

            snapshot = _capture_snapshot(
                left_robot,
                right_robot,
                left_gripper,
                right_gripper,
                config,
            )
            output_path = _next_pose_path(session_dir)
            output_path.write_text(
                json.dumps(snapshot, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            pose = snapshot["pose"]
            print(
                f"已保存 {output_path.name} | "
                f"left_gripper={pose['left_gripper']:.4f}, "
                f"right_gripper={pose['right_gripper']:.4f}"
            )
    except (KeyboardInterrupt, EOFError):
        print("\n记录结束。")
    finally:
        left_gripper.close()
        right_gripper.close()
        if left_robot is not None:
            left_robot.close()
        if right_robot is not None:
            right_robot.close()
        print("只读连接已关闭；未向机械臂或夹爪发送运动指令。")


if __name__ == "__main__":
    main()
