"""Read hardware feedback only. No publishers, actions or write services."""
import json
import math
import os
import signal
import time


def main():
    import rclpy
    from hardware_state_broadcaster_msgs.msg import HardwareState
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    node = rclpy.create_node('workbench_feedback_' + str(os.getpid()))
    state = {}
    received = 0.
    running = True

    def stop(*_):
        nonlocal running
        running = False

    def receive(msg):
        nonlocal state, received
        names = list(msg.joint_names)
        fields = ('position', 'velocity', 'stateword', 'errorcode')
        values = {field: list(getattr(msg, field)) for field in fields}
        if not names or any(len(v) != len(names) or not all(math.isfinite(x) for x in v) for v in values.values()):
            return
        state = {'joint_names': names, **values, 'error': int(msg.hexerrorcode_curerrcode)}
        state['enabled'] = [(int(v) & 0x6f) == 0x27 for v in values['stateword']]
        received = time.monotonic()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    subscription = node.create_subscription(HardwareState, '/hardware_states', receive, qos_profile_sensor_data)
    last = 0.
    try:
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.02)
            now = time.monotonic()
            if now-last >= .1:
                print('WORKBENCH_FEEDBACK ' + json.dumps({'hardware': state, 'age': now-received if received else None}, allow_nan=False), flush=True)
                last = now
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
