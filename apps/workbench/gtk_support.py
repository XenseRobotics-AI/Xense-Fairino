"""Use the project-local Ubuntu GI/cairo bridge when the system lacks it."""
from pathlib import Path
import sys
import gi

# A worktree can share another checkout's .venv through a symlink. The
# project-local GI extension lives beside that environment, outside site-packages.
# Prefer this checkout's bridge, then the shared environment's matching bridge.
for root in (Path(__file__).resolve().parents[2], Path(sys.prefix).resolve().parent):
    bridge = root / '.local/gtk/root/usr/lib/python3/dist-packages/gi'
    if bridge.is_dir() and str(bridge) not in gi.__path__:
        gi.__path__.append(str(bridge))

gi.require_version('Gtk', '3.0')
