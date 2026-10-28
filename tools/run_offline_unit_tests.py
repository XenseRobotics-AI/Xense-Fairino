#!/usr/bin/env python3
"""Run repository unit suites with IPv4/IPv6/packet sockets disabled.

This does not start ROS, the console, a device SDK service, or robot control.
An inherited Linux seccomp filter denies IP sockets, including native SDK and
child processes. Python audit hooks additionally reject external DNS and common
network commands. Unix sockets remain available; this is not a general sandbox.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile
import time

from offline_socket_guard import install_socket_filter

ROOT = Path(__file__).resolve().parents[1]
APPS = (
    'fairino_keyboard', 'fairino_upper_probe',
    'fairino_upper_probe/body_control', 'fairino_tcp_probe',
    'map_viewer', 'navigation', 'workbench', 'gripper_control',
)
GUARD = r'''
import ipaddress
import json
import os
from pathlib import Path
import sys

def local_host(host):
    if host in (None, '', 'localhost', b'localhost'):
        return True
    try:
        if isinstance(host, bytes):
            host = host.decode('ascii')
        return ipaddress.ip_address(str(host).split('%')[0]).is_loopback
    except ValueError:
        return False

def deny(event, target):
    path = os.environ.get('FAIRINO_OFFLINE_DENIAL_LOG')
    if path:
        with open(path, 'a', encoding='utf-8') as out:
            out.write(json.dumps({'event': event, 'target': str(target)}) + '\n')
    raise PermissionError('Offline unit suite blocked external I/O: ' + event)

def audit(event, args):
    if event in ('socket.connect', 'socket.bind'):
        address = args[1]
        if isinstance(address, tuple) and not local_host(address[0]):
            deny(event, address[0])
    elif event == 'socket.sendto':
        address = args[-1]
        if isinstance(address, tuple) and not local_host(address[0]):
            deny(event, address[0])
    elif event == 'socket.getaddrinfo' and not local_host(args[0]):
        deny(event, args[0])
    elif event == 'subprocess.Popen':
        executable, argv = args[:2]
        forbidden = {'ssh', 'sshpass', 'scp', 'sftp', 'rsync', 'curl', 'wget',
                     'ping', 'nc', 'ncat', 'netcat', 'socat', 'ros2', 'adb'}
        name = os.path.basename(os.fsdecode(executable))
        if name in forbidden:
            deny(event, name)
        if name in {'sh', 'bash', 'dash', 'zsh'} and isinstance(argv, (list, tuple)):
            if any(x in ('-c', '-lc', '-ic') for x in argv):
                deny(event, name + ' inline shell')
sys.addaudithook(audit)
OFFLINE_AUDIT_INSTALLED = True
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path,
                        default=ROOT/'logs/20261026-lift-offline-verification/unit-tests')
    parser.add_argument('--timeout', type=float, default=300.)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    base_python = ROOT/'.venv/bin/python'
    pico_python = ROOT/'.local/pico4/env/bin/python'
    suites = [(name.replace('/', '-'), ROOT/'apps'/name, base_python,
               ['discover', '-p', 'test_*.py']) for name in APPS]
    suites += [('pico-fairino', ROOT/'scripts/pico4/fairino', pico_python,
                ['discover', '-p', 'test_*.py']),
               ('pico-receiver', ROOT/'scripts/pico4', pico_python, ['test_pico'])]
    report = {'started_at': dt.datetime.now().astimezone().isoformat(),
              'scope': 'Offline repository unit tests; no robot connection or motion',
              'guard_scope': 'Linux seccomp denies IPv4/IPv6/packet socket creation in suites and descendants, including loopback. Python audit hooks also deny external DNS and common robot/network executables. Unix sockets remain available; not a general sandbox.',
              'suites': []}
    with tempfile.TemporaryDirectory(prefix='fairino-offline-tests-') as temporary:
        guard_dir = Path(temporary)
        (guard_dir/'sitecustomize.py').write_text(GUARD, encoding='utf-8')
        env = {**os.environ, 'PYTHONDONTWRITEBYTECODE': '1',
               'FAIRINO_TCP_DIR': str(ROOT/'apps/fairino_tcp_probe'),
               'XENSE_LOG_DIR': str(output/'sdk-logs'),
               'PYTHONPATH': os.pathsep.join([str(guard_dir), str(ROOT/'src'),
                                            os.environ.get('PYTHONPATH', '')])}
        check = '''import socket, subprocess, sys
assert getattr(sys.modules.get('sitecustomize'), 'OFFLINE_AUDIT_INSTALLED', False), 'Audit hook not installed'
blocked = 0
for domain in (socket.AF_INET, socket.AF_INET6, socket.AF_PACKET):
    try: socket.socket(domain, socket.SOCK_DGRAM)
    except PermissionError: blocked += 1
assert blocked == 3, 'Kernel socket filter not active'
with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM):
    pass
for f in (lambda: socket.create_connection(('192.0.2.1', 22), timeout=.01),
          lambda: subprocess.run(['ssh', '-V'], check=True)):
    try: f()
    except PermissionError: blocked += 1
assert blocked == 5, blocked
print('Kernel IP socket filter and Python external I/O guards verified')
'''
        for python in dict.fromkeys((base_python, pico_python)):
            result = subprocess.run([str(python), '-c', check], env=env,
                                    text=True, capture_output=True, timeout=20,
                                    preexec_fn=install_socket_filter, close_fds=True)
            if result.returncode:
                raise RuntimeError('Offline guard self-check failed: ' + result.stderr)
        report['guard_self_check'] = 'PASS for both interpreters'
        for name, cwd, python, selection in suites:
            path = output/(name+'.log')
            denial = output/(name+'-denied.jsonl')
            if denial.exists():
                denial.unlink()
            command = [str(python), '-m', 'unittest', *selection]
            started = time.monotonic()
            with path.open('w', encoding='utf-8') as log:
                try:
                    result = subprocess.run(command, cwd=cwd,
                        env={**env, 'FAIRINO_OFFLINE_DENIAL_LOG': str(denial)},
                        stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout,
                        preexec_fn=install_socket_filter, close_fds=True)
                    code = result.returncode
                except subprocess.TimeoutExpired:
                    code = 124
                    log.write('\nOFFLINE RUNNER: suite timed out\n')
            text = path.read_text(encoding='utf-8', errors='replace')
            count = re.search(r'Ran (\d+) tests? in', text)
            skipped = re.search(r'\bskipped=(\d+)', text)
            row = {'suite': name, 'command': command, 'cwd': str(cwd),
                   'returncode': code, 'tests': int(count.group(1)) if count else 0,
                   'skipped': int(skipped.group(1)) if skipped else 0,
                   'elapsed_s': round(time.monotonic()-started, 3),
                   'blocked_io': len(denial.read_text().splitlines()) if denial.exists() else 0,
                   'log': str(path)}
            report['suites'].append(row)
            print(json.dumps(row, ensure_ascii=False), flush=True)
    report['tests'] = sum(s['tests'] for s in report['suites'])
    report['skipped'] = sum(s['skipped'] for s in report['suites'])
    report['passed'] = all(s['returncode'] == 0 and s['tests'] and not s['blocked_io']
                           for s in report['suites'])
    report['finished_at'] = dt.datetime.now().astimezone().isoformat()
    (output/'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n',
                                       encoding='utf-8')
    print(json.dumps({k: report[k] for k in ('passed', 'tests', 'skipped')}, ensure_ascii=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
