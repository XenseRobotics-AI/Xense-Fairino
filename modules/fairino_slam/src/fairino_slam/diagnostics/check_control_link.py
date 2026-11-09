#!/usr/bin/env python3
"""Read-only SSH/ROS heartbeat probe using the real desktop transport; no enable."""
import argparse
import base64
import json
from pathlib import Path
import shlex
import sys
import time
from types import SimpleNamespace
from unittest.mock import patch
import zlib

from fairino_slam.paths import PACKAGE_ROOT, PROJECT_ROOT as ROOT, RESOURCE_ROOT, LOG_ROOT
from fairino_slam.chassis import transport
from fairino_slam.chassis.control_core import configuration_snapshot, DEFAULT_LINEAR, DEFAULT_ANGULAR
from fairino_slam.chassis.source import render_control_core
import pexpect

PROBE = '''
def link_probe(self):
    samples, gaps = [], []
    failures = {}
    start = previous = time.monotonic()
    while not self.stop.is_set() and time.monotonic()-start < PROBE_SECONDS:
        self.pump(.02, guarded=False)
        self.navigation_command()  # graph query / subscription only; never publish
        now = time.monotonic()
        gap, previous = now-previous, now
        with self.input_lock:
            packet, received = self.packet, self.received
        if now-start > 3 and packet:
            link = self.link_status(now)
            samples.append(link)
            gaps.append(gap)
            try:
                check_input(packet, received, now)
            except RuntimeError as exc:
                key = str(exc).split('（')[0]
                failures[key] = failures.get(key, 0)+1
        self.phase = '只读心跳测量：未使能、未发布运动指令'
    def stats(values):
        values = sorted(v for v in values if v is not None)
        return {'max': max(values, default=None),
                'p99': values[min(len(values)-1, int(len(values)*.99))] if values else None}
    self.status(True, finished=True, probe={
        'samples': len(samples), 'nice': os.getpriority(os.PRIO_PROCESS, 0),
        'ack_age': stats([s['ack_age'] for s in samples]),
        'input_age': stats([s['input_age'] for s in samples]),
        'client_send_gap': stats([s['client_send_gap'] for s in samples]),
        'loop_gap': stats(gaps), 'failures': failures, 'motors_operated': False})
'''


def command(args):
    folder = PACKAGE_ROOT/'chassis'
    worker = (folder/'robot_worker.py').read_text()
    end = '    sys.exit(main())'
    if worker.count(end) != 1:
        raise RuntimeError('Worker entry point changed; refusing to run diagnostic')
    # Override run before main constructs the worker. Startup is never called;
    # owned remains false, so cleanup never commands the chassis either.
    worker = worker.replace(end, '    Worker.run = link_probe\n'+end)
    worker = PROBE.replace('PROBE_SECONDS', repr(args.seconds))+'\n'+worker
    bundle = {'control_core.py': render_control_core(configuration_snapshot())}
    bundle['robot_worker.py'] = worker
    encoded = base64.b64encode(zlib.compress(json.dumps(bundle).encode())).decode()
    loader = ("import base64,zlib,json,sys,types;"
              f"bundle=json.loads(zlib.decompress(base64.b64decode({encoded!r})));"
              "m=types.ModuleType('control_core');sys.modules['control_core']=m;"
              "exec(bundle['control_core.py'],m.__dict__);"
              "sys.argv=['robot_worker.py','--navigation'];"
              "exec(compile(bundle['robot_worker.py'],'link_probe','exec'),{'__name__':'__main__'})")
    root = '/opt/fairino_AMR_project_B/reference_chassis_runtime/install'
    env = ['set -e', 'source /opt/ros/humble/setup.bash']
    for pkg in ('hardware_state_broadcaster_msgs', 'fairino_hardware_command_msgs'):
        env.append(f'source {root}/{pkg}/share/{pkg}/local_setup.bash')
    env += ['export ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
            'export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fairino_AMR_project_B/config/fastrtps_udp.xml',
            'echo KEYCTRL_CONNECTED',
            f'exec nice -n {args.nice} python3 -u -c '+shlex.quote(loader)]
    return 'sudo -n runuser -l fairino -c '+shlex.quote('; '.join(env))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--nice', type=int, choices=(0, 19), default=0)
    parser.add_argument('--seconds', type=float, default=60)
    args = parser.parse_args()
    if not 5 <= args.seconds <= 120:
        parser.error('seconds must be between 5 and 120')
    original = pexpect.spawn
    def shared_spawn(cmd, argv, **kw):
        return original(cmd, ['-o', 'BatchMode=yes', '-o', 'ControlMaster=auto',
                              '-o', 'ControlPath=~/.ssh/revo3-%C', *argv], **kw)
    options = SimpleNamespace(host='192.168.58.3', user='fairino', linear=DEFAULT_LINEAR, angular=DEFAULT_ANGULAR,
                              log_directory=str(LOG_ROOT/'diagnostics/control-link'))
    with patch.object(transport, 'remote_command', return_value=command(args)), \
         patch.object(pexpect, 'spawn', side_effect=shared_spawn):
        connection = transport.Connection(options)
    try:
        deadline = time.monotonic()+args.seconds+20
        while not connection.done.is_set() and time.monotonic() < deadline:
            connection.send(keys=(), focused=False)
            time.sleep(.05)
    finally:
        connection.finish()
    if 'probe' not in connection.latest:
        raise RuntimeError('Probe did not finish: '+connection.last_error)
    print(json.dumps({'log':str(connection.log_path), **connection.latest['probe']}, ensure_ascii=False))


if __name__ == '__main__':
    main()
