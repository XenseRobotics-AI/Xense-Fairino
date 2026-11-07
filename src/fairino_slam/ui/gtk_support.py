"""Use the project-local Ubuntu GI/cairo bridge when the system lacks it."""
from fairino_slam.paths import PROJECT_ROOT
import gi

bridge = PROJECT_ROOT / '.local/gtk/root/usr/lib/python3/dist-packages/gi'
if bridge.is_dir() and str(bridge) not in gi.__path__:
    gi.__path__.append(str(bridge))

gi.require_version('Gtk', '3.0')
