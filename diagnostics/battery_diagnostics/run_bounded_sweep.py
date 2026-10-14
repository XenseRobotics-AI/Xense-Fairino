#!/usr/bin/env python3
"""Stream a finite remote read-only diagnostic, preserving every result locally."""
import argparse
import json
from pathlib import Path
import shlex
import pexpect

p = argparse.ArgumentParser()
p.add_argument('--host', required=True, choices=['192.168.58.1', '192.168.58.3'])
p.add_argument('--mode', required=True, choices=['common', 'addresses'])
p.add_argument('--devices', nargs='+', default=['/dev/ttyS7'])
args = p.parse_args()
root = Path(__file__).resolve().parent
script = (root / 'bounded_bms_sweep.py').read_text()
command = 'sudo -n python3 -u -c ' + shlex.quote(script) + ' --mode ' + args.mode
command += ' --devices ' + ' '.join(shlex.quote(d) for d in args.devices)
ssh = pexpect.spawn('ssh', ['-o', 'ConnectTimeout=5', '-o', 'StrictHostKeyChecking=accept-new',
                          '-o', 'UserKnownHostsFile=/tmp/fairino-known-hosts',
                          'fairino@' + args.host, command], encoding='utf-8', timeout=330,
                    maxread=65536)
choice = ssh.expect(['(?i)password:', pexpect.EOF])
if choice == 0:
    ssh.sendline('1')
else:
    raise RuntimeError('SSH ended before login: ' + ssh.before)
log = root / ('sweep-' + args.host + '-' + args.mode + '-20261011.jsonl')
port_counts = {}
with log.open('a') as out:
    while True:
        try:
            line = ssh.readline()
        except pexpect.EOF:
            break
        if not line:
            break
        line = line.strip()
        if not line:
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            # Do not print SSH password echo or terminal noise.
            if line == '1':
                continue
            print(json.dumps({'host': args.host, 'remote_message': line}), flush=True)
            continue
        record['host'] = args.host
        out.write(json.dumps(record, ensure_ascii=False) + '\n')
        out.flush()
        compact = dict(record)
        if 'attempted_addresses' in compact:
            compact['query_count'] = len(compact.pop('attempted_addresses'))
            device = compact.get('device')
            port_counts[device] = port_counts.get(device, 0) + compact['query_count']
        if compact.get('event') == 'config_complete' and not compact.get('rx_bytes'):
            continue
        if compact.get('event') == 'port_restored':
            compact['total_queries'] = port_counts.get(compact.get('device'), 0)
        print(json.dumps(compact, ensure_ascii=False), flush=True)
ssh.close()
print(json.dumps({'host': args.host, 'ssh_exit': ssh.exitstatus, 'ssh_signal': ssh.signalstatus,
                  'log': str(log)}), flush=True)
