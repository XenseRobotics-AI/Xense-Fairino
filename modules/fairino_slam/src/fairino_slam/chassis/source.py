"""Freeze validated local parameters into the standalone robot-side core."""
from pathlib import Path

from fairino_slam.configuration import load_chassis, validate_chassis


def render_control_core(snapshot=None):
    """Return self-contained source for file or in-memory robot execution.

    The robot does not need the desktop package, configuration files, or a
    ``__file__`` global. Connection callers supply their process snapshot so
    local and remote limits agree. Deployment callers read current files by
    omitting ``snapshot``; a running motor session is never hot-reloaded.
    """
    configuration = load_chassis() if snapshot is None else validate_chassis(snapshot)
    source = Path(__file__).with_name('control_core.py').read_text()
    return '_CHASSIS_CONFIG = ' + repr(configuration) + '\n' + source
