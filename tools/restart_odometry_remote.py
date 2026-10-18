#!/usr/bin/env python3
"""Apply installed odometry code only, while base control is idle and disabled.

Run on the robot in the chassis ROS environment. Default is a read-only check.
The temporary systemd unit is tied to amr_startup and is not enabled at boot;
the normal start.sh owns the odometry guard again after the next robot reboot.
"""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

RUNTIME = Path.home()/'.local/share/fairino-odometry'
UNIT = 'fairino-odometry-live.service'


def command(*args):
    return subprocess.run(args, capture_output=True, text=True, check=True).stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '1':
        raise RuntimeError('必须在底盘 ROS Domain 1 中检查真实反馈')
    lease = (Path.home()/'.fairino_keyboard.lock').open('a')
    fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
    if command('systemctl', 'is-active', 'amr_startup.service').strip() != 'active':
        raise RuntimeError('底盘启动服务未运行')
    active = subprocess.run(['systemctl', 'is-active', 'fairino-manual-mapping.service'], capture_output=True, text=True).stdout.strip()
    if active not in ('inactive', 'failed'):
        raise RuntimeError('请先结束建图会话，再切换里程计')
    mode = json.loads((RUNTIME/'mode.json').read_text())['mode']
    if mode not in ('raw', 'ekf'):
        raise RuntimeError('未知的已安装里程计模式')
    matches = []
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            argv = [os.fsdecode(x) for x in (proc/'cmdline').read_bytes().split(b'\0') if x]
            if len(argv) > 1 and argv[1] == str(RUNTIME/'odom_guard.py'):
                matches.append((int(proc.name), argv))
        except OSError:
            continue
    if len(matches) != 1:
        raise RuntimeError('需要且仅允许一个已知的里程计守护进程')
    pid, argv = matches[0]
    try:
        environ = (Path('/proc')/str(pid)/'environ').read_bytes()
    except PermissionError:
        # Some vendor services restrict ptrace even for the same Unix user.
        environ = subprocess.check_output(['sudo','-n','cat',f'/proc/{pid}/environ'])
    import rclpy
    from hardware_state_broadcaster_msgs.msg import HardwareState
    from controller_manager_msgs.srv import ListControllers
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init(args=[])
    node = rclpy.create_node('fairino_odometry_restart_check')
    latest = []
    def on_hardware(msg):
        latest[:] = [time.monotonic(), msg]
    subscription = node.create_subscription(HardwareState, '/hardware_states', on_hardware, qos_profile_sensor_data)
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    try:
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError('无法查询控制器状态')
        future = client.call_async(ListControllers.Request())
        end = time.monotonic()+10
        good_since = None
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.03)
            now = time.monotonic()
            if not latest or now-latest[0] > .4:
                good_since = None
                continue
            hw = latest[1]
            values = list(hw.velocity)+list(hw.command_velocity)
            if len(hw.stateword) != 8 or len(values) != 16 or any(int(s)&4 for s in hw.stateword):
                raise RuntimeError('底盘未全部下使能或不静止；未重启里程计：'+json.dumps({
                    'stateword':list(hw.stateword), 'velocity':list(hw.velocity),
                    'command_velocity':list(hw.command_velocity)},ensure_ascii=False))
            if any(not math.isfinite(v) or abs(v) > .005 for v in values):
                good_since = None
                continue
            good_since = now if good_since is None else good_since
            if now-good_since >= 1 and future.done():
                break
        else:
            raise RuntimeError('没有连续一秒的新鲜静止/下使能反馈')
        controllers = {c.name:c.state for c in future.result().controller}
        allowed = {'fairino_hardware_state_broadcaster', 'fairino_hardware_command_controller'}
        if any(state == 'active' and name not in allowed for name,state in controllers.items()):
            raise RuntimeError('仍有运动控制器 active；未重启里程计')
        names = node.get_node_names_and_namespaces()
        publishers = node.get_publishers_info_by_topic('/odom')
        if (any(name in ('slam_toolbox', 'ekf_filter_node') for name,_ in names)
                or any(p.node_name != 'fairino_odometry_guard' for p in publishers)):
            raise RuntimeError('仍有 SLAM、EKF 或其他里程计发布者；未重启')
        report = {'execute':args.execute, 'mode':mode, 'old_pid':pid, 'controllers':controllers,
                  'hardware_restarted':False, 'enabled_count':0}
        if args.execute:
            restarted_after = time.time()
            old_env = dict(os.fsdecode(x).split('=',1) for x in environ.split(b'\0') if b'=' in x)
            # Stop only the guard, not its parent chassis service or process group.
            live = subprocess.run(['systemctl','is-active',UNIT],capture_output=True,text=True).stdout.strip()
            if live == 'active':
                command('sudo','-n','systemctl','stop',UNIT)
            else:
                os.kill(pid, signal.SIGINT)
            deadline = time.monotonic()+8
            while (Path('/proc')/str(pid)/'cmdline').exists() and time.monotonic()<deadline:
                if not (Path('/proc')/str(pid)/'cmdline').read_bytes():
                    break
                time.sleep(.05)
            else:
                if (Path('/proc')/str(pid)/'cmdline').exists():
                    raise RuntimeError('原守护进程未退出，拒绝并行启动')
            start = ['sudo','-n','systemd-run','--unit='+UNIT,'--collect','--uid=fairino','--gid=fairino',
                     '--property=PartOf=amr_startup.service','--property=After=amr_startup.service',
                     '--property=KillSignal=SIGINT','--property=TimeoutStopSec=10']
            for key in ('PATH','PYTHONPATH','LD_LIBRARY_PATH','AMENT_PREFIX_PATH','ROS_DOMAIN_ID',
                        'ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION','FASTRTPS_DEFAULT_PROFILES_FILE'):
                if key in old_env:
                    start.append('--setenv='+key+'='+old_env[key])
            command(*start, *argv)
            deadline=time.monotonic()+8
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.05)
                try:
                    status=json.loads((RUNTIME/'status.json').read_text())
                    if (status.get('mode') == mode and restarted_after < status['time'] <= time.time()
                            and time.time()-status['time']<1 and status.get('armed')
                            and subprocess.run(['systemctl','is-active','--quiet',UNIT]).returncode == 0):
                        report['status']=status
                        report['unit']=UNIT
                        break
                except (OSError,ValueError,KeyError):
                    pass
            else:
                raise RuntimeError('新里程计守护未就绪；硬件没有重启，请查看 '+UNIT+' 日志')
        print('ODOMETRY_RESTART '+json.dumps(report,ensure_ascii=False),flush=True)
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__ == '__main__':
    main()
