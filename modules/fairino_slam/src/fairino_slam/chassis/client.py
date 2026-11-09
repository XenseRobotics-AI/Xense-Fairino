"""Inspect chassis feedback or validate controller handoff without motor enable."""
import argparse
import os
import time

from fairino_slam.paths import LOG_ROOT
from .control_core import DEFAULT_LINEAR, DEFAULT_ANGULAR
from .transport import Connection


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true', help='只读检查反馈与控制器（默认）')
    mode.add_argument('--prepare-check', action='store_true',
                      help='检查控制器接口后停用；会暂停里程计，不回正、不使能')
    parser.add_argument('--host', default='192.168.58.3')
    parser.add_argument('--user', default='fairino')
    args = parser.parse_args(argv)
    args.check = not args.prepare_check
    args.navigation = True
    args.linear, args.angular = DEFAULT_LINEAR, DEFAULT_ANGULAR
    args.log_directory = str(LOG_ROOT/'chassis'/'checks')
    password = os.environ.pop('FAIRINO_SSH_PASSWORD', None)
    connection = None
    try:
        connection = Connection(args, password=password)
        phase = None
        while not connection.done.is_set():
            connection.send(keys=(), focused=True)
            current = connection.latest.get('phase')
            if current != phase:
                phase = current
                print(current or '等待底盘反馈', flush=True)
            time.sleep(.05)
        # Connection retains all status messages, even when the final cleanup
        # packet has replaced the checked packet in latest.
        checked = False
        while not connection.events.empty():
            checked |= bool(connection.events.get_nowait().get('checked'))
        if connection.last_error:
            raise RuntimeError(connection.last_error)
        if not checked:
            raise RuntimeError('未收到控制器检查完成确认')
        print('检查通过；未回正、未使能。', flush=True)
        return 0
    except KeyboardInterrupt:
        print('检查已取消。')
        return 130
    except Exception as exc:
        print('检查未完成：'+str(exc))
        return 1
    finally:
        if connection is not None:
            connection.finish()


if __name__ == '__main__':
    raise SystemExit(main())
