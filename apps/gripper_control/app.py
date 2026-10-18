#!/usr/bin/env python3
"""Standalone gripper debugging, using exactly the workbench panel/backend."""
import argparse
from pathlib import Path
import signal
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from apps.workbench import gtk_support
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GLib
from apps.gripper_control.panel import GripperPanel


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.58.1')
    parser.add_argument('--user', default='fairino')
    parser.add_argument('--demo', action='store_true')
    args = parser.parse_args()
    settings = Gtk.Settings.get_default()
    if settings:
        settings.set_property('gtk-font-name', 'Noto Sans CJK SC 10')
    window = Gtk.Window(title='TacCap 夹爪调试')
    window.set_default_size(1000, 600)
    outer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=12)
    outer.set_border_width(15)
    window.add(outer)
    row = Gtk.Box(spacing=8)
    entries = []
    for label, initial in [('上半身 IP', args.host), ('用户', args.user), ('SSH 密码', '')]:
        row.pack_start(Gtk.Label(label=label), False, False, 0)
        entry = Gtk.Entry(text=initial)
        if label == 'SSH 密码':
            entry.set_visibility(False)
        entries.append(entry)
        row.pack_start(entry, False, False, 0)
    outer.pack_start(row, False, False, 0)
    def credentials(_part):
        values = tuple(e.get_text().strip() if i < 2 else e.get_text() for i, e in enumerate(entries))
        if not all(values):
            panel.state.set_text('请填写上半身 IP、用户名和 SSH 密码')
            return None
        return values
    panel = GripperPanel(credentials, lambda *_: True, lambda *_: None, args.demo)
    outer.pack_start(panel, True, True, 0)
    def close(*_):
        panel.close()
        return True
    def tick():
        running = panel.session and not panel.session.finished()
        for entry in entries:
            entry.set_sensitive(not running and not panel.closing)
        if panel.closing and panel.finished():
            Gtk.main_quit()
            return False
        return True
    window.connect('delete-event', close)
    signal.signal(signal.SIGTERM, close)
    signal.signal(signal.SIGINT, close)
    GLib.timeout_add(100, tick)
    window.show_all()
    Gtk.main()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
