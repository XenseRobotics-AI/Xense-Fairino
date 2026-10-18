import datetime
import json
from pathlib import Path

from scripts.pico4.fairino.clear_errors import report_lines, run_part

ROOT = Path(__file__).resolve().parents[2]


def reset(part, host, user, password, execute=False):
    report = run_part(part, execute, user, host=host, password=password)
    folder = ROOT / 'logs' / (datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f') + '-console-faults')
    folder.mkdir(parents=True)
    destination = folder / (part + '.json')
    destination.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    lines = report_lines(report, execute)
    lines.append('记录：' + str(destination))
    return '\n'.join(lines)
