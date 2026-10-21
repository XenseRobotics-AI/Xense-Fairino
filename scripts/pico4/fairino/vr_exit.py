"""Local exit evidence, independent from the SSH reader and its cleanup."""
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import time


def exit_notice(message, *, error=False):
    """Console output is optional once shutdown/cleanup has started."""
    try:
        print(message, file=sys.stderr if error else sys.stdout, flush=True)
    except Exception:
        pass


class ExitLog:
    def __init__(self, directory):
        directory = Path(directory)
        self.path = directory/(datetime.now().strftime('%Y%m%d-%H%M%S-%f')+'.exit.jsonl')

    def write(self, event, **values):
        # A logging error must never prevent the robot cleanup path.
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open('a') as log:
                log.write(json.dumps(dict(local_time=time.time(), client_monotonic=time.monotonic(),
                                          event=event, **values), ensure_ascii=False, allow_nan=False)+'\n')
        except (OSError, ValueError, TypeError) as exc:
            exit_notice('退出日志写入失败：'+str(exc), error=True)


def connection_state(connection):
    """Best-effort evidence only: malformed/partial feedback cannot block cleanup."""
    if connection is None:
        return None
    try:
        latest = connection.latest
        if not isinstance(latest, dict):
            latest = {}
        hardware = latest.get('hardware')
        enabled = hardware.get('enabled') if isinstance(hardware, dict) else None
        enabled_count = (sum(enabled) if isinstance(enabled, (tuple, list))
                         and all(isinstance(v, bool) or v in (0, 1) for v in enabled) else None)
        received = connection.last_status
        numeric = lambda value: isinstance(value, (int, float)) and math.isfinite(value)
        age = max(0., time.monotonic()-received) if numeric(received) and received > 0 else None
        stamp = latest.get('stamp')
        return dict(log_path=str(connection.log_path),
                    phase=latest.get('phase') if isinstance(latest.get('phase'), str) else None,
                    disabled_confirmed=(latest.get('disabled_confirmed')
                                        if isinstance(latest.get('disabled_confirmed'), bool) else None),
                    feedback_age_s=age, remote_stamp=stamp if numeric(stamp) else None,
                    enabled_count=enabled_count)
    except Exception as exc:
        return dict(snapshot_error=type(exc).__name__)
