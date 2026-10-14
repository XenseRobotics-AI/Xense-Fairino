#!/usr/bin/env python3
"""Relay an upper-body HardwareState error code as a stable Int32 topic."""

from __future__ import annotations

import json
import subprocess
import time

import rclpy
from hardware_state_broadcaster_msgs.msg import HardwareState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32


class UpperEstopRelay(Node):
    """Extract only the safety-relevant code from the upper HardwareState."""

    def __init__(self) -> None:
        super().__init__('upper_estop_relay')
        self.declare_parameter('input_topic', '/hardware_states')
        self.declare_parameter('output_topic', '/upper_estop_error_code')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        if not input_topic or not output_topic:
            raise ValueError('input_topic and output_topic must not be empty')

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(Int32, output_topic, output_qos)
        self._subscription = self.create_subscription(
            HardwareState, input_topic, self._hardware_state_callback, input_qos)
        self._last_error_code: int | None = None
        self.get_logger().info(
            f'Relaying {input_topic}.hexerrorcode_curerrcode to {output_topic}')

    def _hardware_state_callback(self, message: HardwareState) -> None:
        error_code = int(message.hexerrorcode_curerrcode)
        self._publisher.publish(Int32(data=error_code))
        if error_code != self._last_error_code:
            self.get_logger().info(f'Upper error code changed to {error_code}')
            self._last_error_code = error_code


def network_ready() -> bool:
    """Fast DDS must see the robot LAN when its participant is created."""
    try:
        result = subprocess.run(
            ['ip', '-j', '-4', 'address', 'show', 'dev', 'eth0'],
            capture_output=True, text=True, check=True, timeout=2)
        links = json.loads(result.stdout)
        return any(
            'UP' in link.get('flags', []) and 'LOWER_UP' in link.get('flags', [])
            and any(a.get('local') == '192.168.58.1'
                    for a in link.get('addr_info', []))
            for link in links)
    except (OSError, subprocess.SubprocessError, ValueError, TypeError):
        return False


def wait_for_network() -> None:
    # Publish no status while waiting. A missing status continues to block AMR.
    # Wait for a stable address instead of assuming network.target means ready.
    print('Waiting for upper robot LAN eth0 / 192.168.58.1 before DDS init', flush=True)
    stable_since = None
    while True:
        now = time.monotonic()
        if network_ready():
            if stable_since is None:
                stable_since = now
            if now - stable_since >= 1.0:
                print('Robot LAN ready; initializing upper E-stop relay', flush=True)
                return
        else:
            stable_since = None
        time.sleep(0.2)


def main() -> None:
    wait_for_network()
    rclpy.init()
    node = UpperEstopRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
