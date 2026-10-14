
import json, math, time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from hardware_state_broadcaster_msgs.msg import HardwareState
from fairino_hardware_command_msgs.action import StringCommand
from controller_manager_msgs.srv import ListControllers

rclpy.init()
n = rclpy.create_node('codex_upper_reset_error_once')
latest = None
stamp = 0.
serial = 0
report = {'command': 'reseterror()', 'sent': False}

def on_state(m):
    global latest, stamp, serial
    latest = m
    stamp = time.monotonic()
    serial += 1

sub = n.create_subscription(HardwareState, '/hardware_states', on_state, 10)

def wait_future(future, seconds):
    end = time.monotonic() + seconds
    while not future.done() and time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=.05)
    if not future.done():
        raise RuntimeError('ROS request timed out; do not repeat automatically')
    return future.result()

def read_fresh(seconds=5.):
    previous = serial
    end = time.monotonic() + seconds
    while serial <= previous and time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=.05)
    if latest is None or serial <= previous or time.monotonic() - stamp > .4:
        raise RuntimeError('No fresh hardware feedback')
    return latest

def summary(m):
    names = list(m.joint_names)
    return {'joint_names': names,
            'enabled_count': sum(bool(int(s) & 4) for s in m.stateword),
            'error_hex': f'0x{int(m.hexerrorcode_curerrcode):08X}',
            'joint_errors': {name: int(code) for name, code in zip(names, m.errorcode) if code},
            'stateword': [int(s) for s in m.stateword],
            'max_abs_velocity_rad_s': max(abs(float(v)) for v in m.velocity),
            'j14_position_deg': math.degrees(float(m.position[names.index('j14')]))}

def require_stopped(m):
    names = list(m.joint_names)
    if len(names) != 20 or set(names) != {f'j{i}' for i in range(1, 21)}:
        raise RuntimeError('Unexpected hardware joint mapping')
    if len(m.stateword) != 20 or len(m.velocity) != 20:
        raise RuntimeError('Incomplete hardware feedback')
    if any(int(s) & 4 for s in m.stateword):
        raise RuntimeError('Some joints are enabled; reset not sent')
    if any(not math.isfinite(float(v)) or abs(float(v)) > .005 for v in m.velocity):
        raise RuntimeError('Joints are not stationary; reset not sent')

try:
    state = read_fresh()
    require_stopped(state)
    report['before'] = summary(state)
    gpio = {str(i): Path(f'/sys/class/gpio/gpio{i}/value').read_text().strip() for i in (56, 57)}
    report['gpio'] = gpio
    if any(v != '0' for v in gpio.values()):
        raise RuntimeError('Emergency-stop input is active; reset not sent')
    client = n.create_client(ListControllers, '/controller_manager/list_controllers')
    if not client.wait_for_service(timeout_sec=5.):
        raise RuntimeError('Controller list unavailable; reset not sent')
    controllers = wait_future(client.call_async(ListControllers.Request()), 5.)
    active = [c.name for c in controllers.controller if c.state == 'active']
    report['active_controllers'] = active
    allowed = {'fairino_hardware_state_broadcaster', 'fairino_hardware_command_controller', 'joint_state_broadcaster'}
    if set(active) - allowed:
        raise RuntimeError('An upper-body motion controller is active; reset not sent')
    action = ActionClient(n, StringCommand, '/fairino_hardware_command_controller/command')
    if not action.wait_for_server(timeout_sec=5.):
        raise RuntimeError('Command action unavailable; reset not sent')
    require_stopped(read_fresh())
    goal = StringCommand.Goal()
    goal.command = 'reseterror()'
    report['sent'] = True
    accepted = wait_future(action.send_goal_async(goal), 5.)
    report['accepted'] = accepted.accepted
    if not accepted.accepted:
        raise RuntimeError('Reset goal was rejected')
    result = wait_future(accepted.get_result_async(), 12.)
    report['action_status'] = result.status
    report['command_result'] = result.result.error_code
    end = time.monotonic() + 5.
    consecutive = 0
    while time.monotonic() < end:
        state = read_fresh(seconds=1.)
        report['after'] = summary(state)
        clean = int(state.hexerrorcode_curerrcode) == 0 and not any(state.errorcode)
        off = not any(int(s) & 4 for s in state.stateword)
        consecutive = consecutive + 1 if clean and off else 0
        if consecutive >= 5:
            break
    report['cleared_and_disabled'] = consecutive >= 5
except Exception as exc:
    report['failure'] = str(exc)
finally:
    print('RESET_RESULT ' + json.dumps(report, ensure_ascii=False), flush=True)
    n.destroy_node()
    rclpy.shutdown()
