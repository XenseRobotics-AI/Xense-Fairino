"""Read hardware feedback only. No publishers, actions or write services."""
import json
import math
import os
import signal
import time


def feedback_packet(state, received, now, started, rejected=None):
    age = now - received if received is not None else None
    packet = {'hardware': state, 'age': age}
    if rejected:
        packet['error'] = '/hardware_states 数据无效：' + rejected
    elif age is None:
        waited = now - started
        packet['status'] = 'SSH 已连接，等待 /hardware_states 硬件反馈'
        if waited >= 5:
            packet['error'] = (f'SSH 已连接，但 {waited:.1f} 秒未收到有效的 /hardware_states；'
                               '请检查状态发布器及 ROS 通信配置')
    elif age > .5:
        packet['error'] = f'/hardware_states 已 {age:.1f} 秒未更新'
    return packet


def main():
    import rclpy
    from hardware_state_broadcaster_msgs.msg import HardwareState
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    node = rclpy.create_node('workbench_feedback_' + str(os.getpid()))
    state = {}
    received = None
    rejected = None
    running = True

    def stop(*_):
        nonlocal running
        running = False

    def receive(msg):
        nonlocal state, received, rejected
        names = list(msg.joint_names)
        fields = ('position', 'velocity', 'stateword', 'errorcode')
        values = {field: list(getattr(msg, field)) for field in fields}
        if not names or any(len(v) != len(names) or not all(math.isfinite(x) for x in v) for v in values.values()):
            rejected = '关节数组长度不一致、为空或包含非有限数值'
            return
        state = {'joint_names': names, **values, 'error': int(msg.hexerrorcode_curerrcode)}
        state['enabled'] = [(int(v) & 0x6f) == 0x27 for v in values['stateword']]
        received = time.monotonic()
        rejected = None

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    subscription = node.create_subscription(HardwareState, '/hardware_states', receive, qos_profile_sensor_data)
    last = 0.
    started = time.monotonic()
    try:
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.02)
            now = time.monotonic()
            if now-last >= .1:
                packet = feedback_packet(state, received, now, started, rejected)
                print('WORKBENCH_FEEDBACK ' + json.dumps(packet, allow_nan=False), flush=True)
                last = now
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
