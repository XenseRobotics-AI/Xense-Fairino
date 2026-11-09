"""SSH wrapper for the existing robot-side PAM priority repair."""
import argparse
import getpass
import os
import shlex

from fairino_slam.connection.ssh import run
from fairino_slam.paths import PACKAGE_ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.58.3')
    parser.add_argument('--user', default='fairino')
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if not args.execute:
        from .install_session_priority_remote import CONTENT, TARGET
        print(str(TARGET) + '\n' + CONTENT)
        print('加 --execute 才连接机器人并安装；需要远端 sudo -n 权限。')
        return 0
    source = (PACKAGE_ROOT/'deployment/install_session_priority_remote.py').read_text()
    password = os.environ.pop('FAIRINO_SSH_PASSWORD', None) or getpass.getpass(
        f'SSH 密码（{args.user}@{args.host}）：')
    print(run(args.host, 'sudo -n python3 -u -c ' + shlex.quote(source) + ' --execute',
              user=args.user, password=password, timeout=45))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
