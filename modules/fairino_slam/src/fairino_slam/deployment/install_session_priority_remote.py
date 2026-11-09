#!/usr/bin/env python3
"""Install a per-user PAM priority correction; leave rtprio/memlock unchanged."""
import argparse
import datetime
import json
from pathlib import Path
import shutil
import subprocess

TARGET = Path('/etc/security/limits.d/90-fairino-session-priority.conf')
CONTENT = """# Managed by fairino-robot-toolkit: SSH control uses normal scheduling.
# priority is the ordinary nice value, NOT the rtprio permission.
# Override the vendor @realtime priority=60 for this user only.
fairino - priority 0
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    report = {'target': str(TARGET), 'content': CONTENT, 'written': False}
    if args.execute:
        backup = Path('/home/fairino/.local/share/fairino-navigation/backups')/(
            'session-priority-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
        backup.mkdir(parents=True)
        shutil.copy2('/etc/security/limits.conf', backup/'limits.conf')
        if TARGET.exists():
            shutil.copy2(TARGET, backup/TARGET.name)
        temp = TARGET.with_suffix('.tmp')
        temp.write_text(CONTENT)
        temp.chmod(0o644)
        temp.replace(TARGET)
        result = subprocess.run(
            ['runuser', '-l', 'fairino', '-c',
             "python3 -c 'import os; print(os.getpriority(os.PRIO_PROCESS, 0))'"],
            text=True, capture_output=True, timeout=10)
        if result.returncode or result.stdout.strip() != '0':
            if (backup/TARGET.name).exists():
                shutil.copy2(backup/TARGET.name, TARGET)
            else:
                TARGET.unlink()
            raise RuntimeError('PAM verification failed, rolled back: '+result.stdout+result.stderr)
        report.update(written=True, backup=str(backup), fresh_login_nice=0,
                      existing_sessions_changed=False, motors_operated=False)
    print(json.dumps(report, ensure_ascii=False))


if __name__ == '__main__':
    main()
