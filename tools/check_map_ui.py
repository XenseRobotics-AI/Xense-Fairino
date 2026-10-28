#!/usr/bin/env python3
"""Render the real GTK workbench offline and verify usable map/stop controls.

Uses Gtk.OffscreenWindow, a running display (or xvfb-run), and DemoStream.
No robot credentials, ROS graph or network access are required.
"""
import argparse
from contextlib import ExitStack
import json
from pathlib import Path
import statistics
import sys
import time
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'logs/map-ui-check')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    from apps.workbench.app import Workbench, Gtk, GLib
    if not Gtk.init_check()[0]:
        raise SystemExit('需要 GTK 显示环境；可在桌面终端或 xvfb-run 下运行。')

    def settle(seconds=.15):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            while Gtk.events_pending():
                Gtk.main_iteration_do(False)
            time.sleep(.005)

    class HiddenWindow(Gtk.OffscreenWindow):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            self.set_decorated(False)

        def present(self):
            pass

    blocked = AssertionError('Offline UI check attempted a robot/network operation')
    report = {'data': 'DemoStream simulated data', 'hardware_connected': False, 'screenshots': []}
    with ExitStack() as stack:
        stack.enter_context(patch.object(Gtk, 'Window', HiddenWindow))
        stack.enter_context(patch.object(GLib, 'timeout_add', return_value=0))
        stack.enter_context(patch.object(GLib, 'timeout_add_seconds', return_value=0))
        for entry in ('apps.workbench.app.Feedback', 'apps.workbench.app.route_info',
                      'apps.map_viewer.panel.map_action', 'apps.navigation.panel.execute',
                      'apps.navigation.panel.DriveSession', 'threading.Thread.start'):
            stack.enter_context(patch(entry, side_effect=blocked))
        app = Workbench(SimpleNamespace(demo=True, base_host='192.168.58.3',
            upper_host='192.168.58.1', user='fairino', page='map', live_arms=True))
        try:
            panel = app.map_panel
            assert panel is not None and not app.model_panel.get_visible()
            assert not app.observers and not app.controls and not app.leases
            stream = panel.stream

            def size(width, height):
                app.window.set_default_size(width, height)
                app.window.set_size_request(width, height)
                app.window.resize(width, height)
                app.window.check_resize()
                settle()

            def capture(name, width=1500, height=940, tab=None, scenario='tracking', manual=False, follow=False, drag=False, scroll_end=False):
                tab = panel.workflow_page if tab is None else tab
                panel.cancel_pick()
                panel.base_expander.set_expanded(manual)
                panel.scenario_choice.set_active_id(scenario)
                panel.side_notebook.set_current_page(tab)
                panel.follow.set_active(follow)
                panel.navigation.tick()
                panel.tick()
                size(width, height)
                panel.tick()
                panel.canvas.draw()
                adjustment = panel.workflow_scroll.get_vadjustment()
                adjustment.set_value(adjustment.get_upper() if scroll_end else 0)
                if drag:
                    panel.begin_initial_pose()
                    x, y, _ = panel.displayed['frame']['pose']
                    event = lambda px, py: SimpleNamespace(inaxes=panel.plot.ax, button=1, xdata=px, ydata=py)
                    panel.pose_press(event(x-.7, y-.5))
                    panel.pointer_move(event(x+.9, y+.3))
                    assert panel.plot.pose_preview.arrow.get_visible()
                    assert panel.plot.pose_preview.body.get_visible()
                    panel.canvas.draw()
                app.window.queue_draw()
                settle()
                pix = app.window.get_pixbuf()
                path = args.output/(name+'.png')
                pix.savev(str(path), 'png', [], [])
                assert (pix.get_width(), pix.get_height()) == (width, height), (name, pix.get_width(), pix.get_height(), panel.get_preferred_height(), panel.body_pane.get_preferred_height(), panel.base_scroll.get_preferred_height())
                assert panel.canvas.get_allocated_width() >= width-420
                assert panel.canvas.get_allocated_height() >= 200
                if tab == panel.workflow_page:
                    for action in ('cancel', 'end'):
                        button = panel.navigation.buttons[action]
                        x, y = button.translate_coordinates(app.window, 0, 0)
                        assert y+button.get_allocated_height() < height, (name, action, y)
                        assert x+button.get_allocated_width() < width
                    if not manual and not scroll_end and not drag:
                        # Core workflow stays visible together at normal desktop sizes.
                        viewport = panel.workflow_scroll.get_child()
                        for button in (panel.prepare_drive, panel.buttons['initial-pose'],
                                       panel.navigation.buttons['start'], panel.navigation.buttons['pick']):
                            x, y = button.translate_coordinates(viewport, 0, 0)
                            assert y+button.get_allocated_height() <= panel.workflow_scroll.get_allocated_height(), (name, button.get_label(), y, panel.workflow_scroll.get_allocated_height())
                report['screenshots'].append({'file': str(path), 'size': [width,height],
                    'map_canvas': [panel.canvas.get_allocated_width(),panel.canvas.get_allocated_height()],
                    'scenario': scenario, 'manual_expanded': manual})

            size(1500, 940)
            for step in range(200):
                stream.elapsed = step*.4
                panel.tick()
            panel.navigation.set_picked_target(6.5,4.3,0.)
            panel.navigation.control('plan')
            capture('console-navigation')
            capture('console-follow', follow=True)
            capture('console-workflow-1366', 1366, 900)
            capture('console-manual-1366', 1366, 900, manual=True, follow=True)
            capture('console-workflow-scroll-end', 1366, 900, manual=True, scroll_end=True)
            capture('console-localization', tab=1, scenario='candidate', follow=True)
            capture('console-initial-pose-drag', tab=1, scenario='candidate', follow=True, drag=True)
            panel.cancel_pick()
            capture('console-stale', tab=2, scenario='stale', follow=True)
            capture('console-empty', tab=0, scenario='empty')

            stream.scenario = 'tracking'
            panel.base_expander.set_expanded(False)
            panel.follow.set_active(True)
            size(1500, 940)
            durations = []
            for step in range(30):
                stream.elapsed = 80+step*.25
                started = time.perf_counter()
                panel.tick()
                panel.canvas.draw()
                durations.append((time.perf_counter()-started)*1000)
            report['render_ms'] = {'frames':len(durations), 'median':round(statistics.median(durations),2),
                                   'max':round(max(durations),2)}
            assert not app.observers and not app.controls and not app.leases
        finally:
            app.disconnect_observers()
            app.window.destroy()
            settle(.03)
    (args.output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
