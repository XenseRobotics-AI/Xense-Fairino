#!/usr/bin/env python3
"""Height and posture window, with offline demo and a ROS-over-SSH backend."""
import argparse
import base64
import json
import math
import queue
import shlex
import signal
import sys
import threading
import time
import zlib
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from upper_keyboard import Connection
from body_core import load_config, plan, sample_interval


def remote_command(args, model):
    bundle = {name: (HERE.parent/name).read_text() for name in ('probe_core.py', 'robot_probe.py')}
    bundle.update({name: (HERE/name).read_text() for name in ('body_core.py', 'body_robot.py')})
    encoded = base64.b64encode(zlib.compress(json.dumps(bundle).encode())).decode()
    argv = ['body_robot.py']
    if args.check or getattr(args, 'calibrate_upright', False):
        argv.append('--check')
    if args.prepare_check:
        argv.append('--prepare-check')
    loader = '\n'.join([
        'import base64,zlib,json,sys,types',
        f'bundle=json.loads(zlib.decompress(base64.b64decode({encoded!r})))',
        "for name in ('probe_core','robot_probe','body_core'):",
        " m=types.ModuleType(name);m.__file__=name+'.py';sys.modules[name]=m",
        " exec(compile(bundle[name+'.py'],name+'.py','exec'),m.__dict__)",
        f'sys.argv={argv!r}',
        "exec(compile(bundle['body_robot.py'],'body_robot.py','exec'),"
        f"{{'__name__':'__main__','CONFIG':{model.config!r}}})"])
    setup = '\n'.join([
        'set -e', 'source /opt/ros/humble/setup.bash',
        'source /opt/fairino_art_plugin/plugin/install/setup.bash',
        'export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
        'export FASTRTPS_DEFAULT_PROFILES_FILE=/home/fairino/fastrtps_udp.xml',
        'echo UPPERPROBE_CONNECTED', 'exec python3 -u -c '+shlex.quote(loader)])
    return 'bash -c '+shlex.quote(setup)


class Window:
    def __init__(self, args, model, connection=None):
        import gi
        gi.require_version('Gtk', '3.0')
        from gi.repository import Gtk, Gdk, GLib
        self.Gtk, self.Gdk, self.GLib = Gtk, Gdk, GLib
        self.args, self.model, self.connection = args, model, connection
        self.sequence = self.cancel_count = 0
        self.request = None
        self.ready = bool(args.demo)
        self.pending = self.stopped = self.closing = False
        self.last_hw = None
        self.demo_plan = None
        self.demo_start = 0.
        self.initialized = bool(args.demo)
        self.updating = False
        self.window = Gtk.Window(title='身体升降与姿态调节')
        self.window.set_default_size(900, 690)
        self.window.set_border_width(18)
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=12)
        self.window.add(box)
        title = Gtk.Label()
        title.set_markup('<span size="18000" weight="bold">身体高度与姿态</span>')
        box.pack_start(title, False, False, 0)
        text = '选择目标后点击执行；空格中止并保持，Esc 退出。切换窗口不会下使能。'
        box.pack_start(Gtk.Label(label=text), False, False, 0)
        if args.demo:
            label = '离线演示：未连接机器人'
            if not model.config.get('upright_confirmed'):
                label += ' · 竖直姿态基准待确认'
        else:
            label = '连接上半身 .1；启动后使能并保持当前位置，点击执行才运动'
        self.mode = Gtk.Label(label=label)
        box.pack_start(self.mode, False, False, 0)
        self.phase = Gtk.Label(label='就绪' if args.demo else '等待机器人状态')
        self.phase.set_line_wrap(True)
        box.pack_start(self.phase, False, False, 0)
        self.actual = Gtk.Label(label='等待反馈')
        self.actual.set_line_wrap(True)
        box.pack_start(self.actual, False, False, 0)
        self.angles = Gtk.Label()
        box.pack_start(self.angles, False, False, 0)

        self.controls = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=12)
        box.pack_start(self.controls, True, True, 0)
        row = Gtk.Box(spacing=12)
        row.pack_start(Gtk.Label(label='升降目标'), False, False, 0)
        self.height = Gtk.Scale.new_with_range(Gtk.Orientation.HORIZONTAL, 0, 100, 1)
        self.height.set_digits(1)
        self.height.set_hexpand(True)
        self.height.add_mark(0, Gtk.PositionType.BOTTOM, '低位')
        self.height.add_mark(50, Gtk.PositionType.BOTTOM, '中位')
        self.height.add_mark(100, Gtk.PositionType.BOTTOM, '高位')
        row.pack_start(self.height, True, True, 0)
        self.controls.pack_start(row, False, False, 0)
        row = Gtk.Box(spacing=10)
        self.presets = {}
        for name, h in [('低位', 0), ('中位', 50), ('高位', 100)]:
            button = Gtk.Button(label=name)
            button.connect('clicked', lambda _, value=h: self.preset(value))
            row.pack_start(button, True, True, 0)
            self.presets[name] = button
        self.controls.pack_start(row, False, False, 0)
        grid = Gtk.Grid(column_spacing=16, row_spacing=12)
        self.controls.pack_start(grid, False, False, 0)
        self.inputs = {}
        for i, (key, label, lo, hi) in enumerate([
                ('forward', '前倾（°，0 为已标定竖直）', 0, model.config['max_forward_deg']),
                ('roll', '侧倾（°，左负 / 右正）', -model.config['max_roll_deg'], model.config['max_roll_deg']),
                ('yaw', '腰旋转（°，右负 / 左正）', -model.config['max_yaw_deg'], model.config['max_yaw_deg'])]):
            spin = Gtk.SpinButton.new_with_range(lo, hi, .5)
            spin.set_digits(1)
            spin.set_width_chars(9)
            spin.set_value(0)
            self.inputs[key] = spin
            grid.attach(Gtk.Label(label=label, xalign=0), 0, i, 1, 1)
            grid.attach(spin, 1, i, 1, 1)
        grid.attach(Gtk.Label(label='各关节最高运动速度（°/s）', xalign=0), 0, 3, 1, 1)
        self.speed = Gtk.Entry()
        self.speed.set_text(f"{model.config['default_speed_deg_s']:g}")
        self.speed.set_width_chars(9)
        self.speed.set_tooltip_text('默认 5°/s；输入任意有限正数，不设软件速度上限')
        grid.attach(self.speed, 1, 3, 1, 1)
        self.roll_note = Gtk.Label()
        self.roll_note.set_line_wrap(True)
        self.controls.pack_start(self.roll_note, False, False, 0)
        self.preview = Gtk.Label()
        self.preview.set_line_wrap(True)
        self.controls.pack_start(self.preview, False, False, 0)
        buttons = Gtk.Box(spacing=10)
        self.copy_button = Gtk.Button(label='填入当前姿态')
        self.copy_button.connect('clicked', lambda *_: self.copy_current())
        self.zero_button = Gtk.Button(label='姿态回正（保留高度）')
        self.zero_button.connect('clicked', lambda *_: self.zero_attitude())
        self.execute_button = Gtk.Button(label='执行到目标')
        self.execute_button.connect('clicked', lambda *_: self.execute())
        for button in (self.copy_button, self.zero_button, self.execute_button):
            buttons.pack_start(button, True, True, 0)
        self.controls.pack_start(buttons, False, False, 0)
        stop = Gtk.Button(label='中止运动并保持使能 · 空格')
        stop.connect('clicked', lambda *_: self.cancel())
        box.pack_start(stop, False, False, 0)
        self.error = Gtk.Label()
        self.error.set_line_wrap(True)
        box.pack_start(self.error, False, False, 0)
        foot = Gtk.Label(label=(
            '高度为模型估算的相对升降量，不是离地高度；头部和双臂保持原关节角度。\n'
            '前倾 0～45°，腰旋转 ±90°；侧倾 ±15°仅在高度≥90%且前倾≤5°时开放。\n'
            '退出、断联、急停或硬件故障时全部下使能。'))
        foot.set_line_wrap(True)
        box.pack_start(foot, False, False, 0)
        self.height.connect('value-changed', lambda *_: self.changed())
        for spin in self.inputs.values():
            spin.connect('value-changed', lambda *_: self.changed())
        self.speed.connect('changed', lambda *_: self.changed())
        self.window.connect('delete-event', self.close)
        self.window.connect('key-press-event', self.key_press)
        if args.demo:
            q = model.pose(dict(height=0, forward=0, roll=0, yaw=0), [0.]*20)
            self.last_hw = dict(position=q, command_position=q[:], enabled=[True]*20)
        self.changed()
        self.update_feedback()
        self.window.show_all()
        self.timer = GLib.timeout_add(50, self.tick)

    def target(self):
        for spin in self.inputs.values():
            spin.update()
        return dict(height=self.height.get_value(), **{k: w.get_value() for k, w in self.inputs.items()})

    def changed(self):
        if self.updating:
            return
        self.updating = True
        try:
            state = dict(height=self.height.get_value(), **{k: w.get_value() for k, w in self.inputs.items()})
            allowed = self.model.roll_allowed(state)
            if not allowed and abs(state['roll']) > 1e-7:
                self.inputs['roll'].set_value(0)
                state['roll'] = 0.
            self.inputs['roll'].set_sensitive(allowed)
            self.roll_note.set_text('侧倾可调 ±15°' if allowed else '侧倾锁定为 0°；离开侧倾区域时，执行程序会先将身体侧倾回正。')
            q = self.model.pose(state, [0.]*20)
            self.preview.set_text(
                f"目标升降 {state['height']:.1f}% · 相对低位约 {state['height']/100*self.model.travel:.1f} mm\n"
                f"折叠三轴目标：J18 {math.degrees(q[17]):+.2f}°　J19 {math.degrees(q[18]):+.2f}°　J20 {math.degrees(q[19]):+.2f}°")
        except ValueError as exc:
            self.error.set_text(str(exc))
        finally:
            self.updating = False

    def preset(self, height):
        self.updating = True
        self.height.set_value(height)
        for spin in self.inputs.values():
            spin.set_value(0)
        self.updating = False
        self.changed()
        self.error.set_text('已填入参考姿态；点击“执行到目标”才运动')

    def zero_attitude(self):
        for spin in self.inputs.values():
            spin.set_value(0)
        self.changed()
        self.error.set_text('已填入回正目标；点击“执行到目标”才运动')

    def copy_current(self):
        if not self.last_hw:
            return
        s = self.model.state(self.last_hw['position'])
        self.updating = True
        self.height.set_value(s['height'])
        for k, w in self.inputs.items():
            w.set_value(s[k])
        self.updating = False
        self.changed()
        self.error.set_text('已填入当前姿态；超出新软范围的输入已收回到边界，点击执行才运动')

    def execute(self):
        if not self.ready or self.pending or self.stopped or not self.last_hw:
            return
        try:
            if self.connection and time.monotonic()-self.connection.last_status > .4:
                raise ValueError('反馈已过期，未发送目标')
            target = self.model.validate(self.target())
            speed = float(self.speed.get_text())
            hold = self.last_hw.get('command_position', self.last_hw['position'])
            planned = plan(self.model, self.last_hw['position'], hold, target, speed)
        except (ValueError, OverflowError) as exc:
            self.error.set_text(str(exc))
            return
        self.sequence += 1
        self.pending = True
        self.ready = False
        self.error.set_text('')
        self.phase.set_text(' → '.join(planned['stages'])+f"；预计 {planned['duration']:.1f} 秒")
        if self.args.demo:
            self.demo_plan = planned
            self.demo_start = time.monotonic()
        else:
            self.request = dict(kind='body_target', id=self.sequence,
                                created=self.connection.latest.get('stamp', 0),
                                target=target, speed_deg_s=speed)
            self.send()
        self.controls.set_sensitive(False)

    def send(self):
        if self.connection:
            self.connection.send(self.request, self.cancel_count, focused=True, exit=self.stopped)

    def cancel(self):
        self.cancel_count += 1
        self.request = None
        if self.args.demo:
            self.demo_plan = None
            self.pending = False
            self.ready = True
            self.controls.set_sensitive(True)
            self.changed()
        self.phase.set_text('正在中止运动并保持使能')
        self.send()

    def key_press(self, _, event):
        if event.keyval == self.Gdk.KEY_Escape:
            self.close(); return True
        if event.keyval == self.Gdk.KEY_space:
            self.cancel(); return True
        return False

    def update_feedback(self):
        if not self.last_hw:
            return
        state = self.model.state(self.last_hw['position'])
        self.actual.set_text(
            f"实测换算：升降 {state['height']:.1f}% / {state['height_mm']:.1f} mm　"
            f"前倾 {state['forward']:+.2f}°　侧倾 {state['roll']:+.2f}°　腰旋转 {state['yaw']:+.2f}°\n"
            f"关节使能 {sum(self.last_hw['enabled'])}/20")
        self.angles.set_text('　'.join(f"J{j} {math.degrees(self.last_hw['position'][j-1]):+.2f}°" for j in range(16, 21)))

    def tick(self):
        if self.closing:
            return False
        if self.demo_plan:
            elapsed = time.monotonic()-self.demo_start
            points = self.demo_plan['points']
            if elapsed >= points[-1]['time']:
                q = self.demo_plan['final'][:]
                self.demo_plan = None
                self.pending = False
                self.ready = True
                self.phase.set_text('演示完成，保持目标姿态')
            else:
                q = points[0]['positions'][:]
                for a, b in zip(points, points[1:]):
                    if a['time'] <= elapsed <= b['time']:
                        q, _ = sample_interval(a, b, elapsed)
                        break
            self.last_hw['position'] = q
            self.last_hw['command_position'] = q[:]
        if self.connection:
            while True:
                try:
                    data = self.connection.events.get_nowait()
                except queue.Empty:
                    break
                if 'error' in data:
                    self.error.set_text(data['error'])
                    self.stopped = True
                    self.ready = False
                if 'phase' in data:
                    self.phase.set_text(data['phase'])
                    self.ready = data.get('ready', False) and not self.stopped
                    if self.ready and data.get('last_request', 0) >= self.sequence:
                        self.pending = False
                        self.request = None
                    if data.get('hardware'):
                        self.last_hw = data['hardware']
                        if not self.initialized:
                            self.copy_current()
                            self.initialized = True
                if data.get('connection_closed'):
                    self.stopped = True
                    self.ready = False
                    if not self.connection.latest.get('disabled_confirmed'):
                        self.error.set_text('连接关闭，未收到最终下使能确认，请现场检查')
            self.send()
        self.controls.set_sensitive(self.ready and not self.pending and not self.stopped)
        self.update_feedback()
        return True

    def close(self, *_):
        if self.closing:
            return True
        self.closing = self.stopped = True
        self.ready = False
        self.phase.set_text('正在退出，等待全部关节下使能确认')
        self.send()
        if self.connection:
            def finish():
                self.connection.finish()
                self.GLib.idle_add(self.Gtk.main_quit)
            threading.Thread(target=finish, daemon=True).start()
        else:
            self.Gtk.main_quit()
        return True

    def run(self):
        self.Gtk.main()


def main():
    parser = argparse.ArgumentParser(description='身体协同升降与姿态调节，默认各轴最高 5°/s')
    parser.add_argument('--host', default='192.168.58.1')
    parser.add_argument('--user', default='fairino')
    parser.add_argument('--config', type=Path, default=HERE/'body_config.json')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--demo', action='store_true')
    mode.add_argument('--check', action='store_true')
    mode.add_argument('--prepare-check', action='store_true')
    mode.add_argument('--calibrate-upright', action='store_true',
                      help='由用户先将胸部摆到竖直；只读当前角度并保存为倾角零点，不使能、不运动')
    args = parser.parse_args()
    if any(s.startswith('-') or '@' in s for s in (args.host, args.user)):
        parser.error('无效的 SSH 主机或用户名')
    read_only = args.check or args.prepare_check or args.calibrate_upright
    model = load_config(args.config, live=not (args.demo or read_only))
    if args.demo:
        Window(args, model).run()
        return 0
    print('连接上半身 .1；检查模式不使能。' if read_only else
          '窗口启动后使能全部 J1–J20 并保持；选择高/中/低只填入目标，点击执行才运动。')
    if args.calibrate_upright:
        print('将当前胸部姿态登记为竖直参考；只读取关节，不写机器人零位或驱动器参数。')
    if not read_only:
        print('速度默认 5°/s、可调；前倾≤45°，腰旋转±90°；侧倾±15°仅在高位区域且前倾≤5°时允许。')
        print('窗口失焦不下使能；空格中止并保持，Esc/退出时全部下使能。')
    connection = Connection(args, worker_command=remote_command(args, model), log_directory=HERE/'logs')
    print('日志：', connection.log_path)
    if read_only:
        passed = failed = False
        calibration = None
        end = time.monotonic()+45
        while not connection.done.is_set() and time.monotonic() < end:
            connection.send()
            time.sleep(.05)
        while not connection.events.empty():
            data = connection.events.get_nowait()
            if 'error' in data:
                failed = True
                print('失败：', data['error'])
            if data.get('checked'):
                passed = True
                if args.calibrate_upright:
                    calibration = data.get('hardware')
                else:
                    print(json.dumps(data, ensure_ascii=False, indent=2))
        connection.finish()
        if args.calibrate_upright and passed and not failed and calibration:
            config = dict(model.config)
            config['upright_joint_sum_deg'] = sum(math.degrees(v) for v in calibration['position'][17:20])
            config['upright_confirmed'] = True
            config['upright_reference_joint_deg'] = [math.degrees(v) for v in calibration['position'][15:20]]
            config['upright_source'] = '用户运行 --calibrate-upright，将当前胸部姿态声明为竖直'
            config['upright_saved_at'] = time.strftime('%Y-%m-%d %H:%M:%S')
            backup = args.config.with_name(args.config.stem+'-before-upright-'+time.strftime('%Y%m%d-%H%M%S')+'.json')
            backup.write_text(args.config.read_text())
            temporary = args.config.with_suffix('.json.tmp')
            temporary.write_text(json.dumps(config, ensure_ascii=False, indent=2)+'\n')
            temporary.replace(args.config)
            print(f"竖直参考已保存：J18+J19+J20={config['upright_joint_sum_deg']:+.4f}°")
            print('只修改本机高度程序的计算参考；没有修改机器人编码器零位。')
        return 0 if passed and not failed else 1
    try:
        window = Window(args, model, connection)
        signal.signal(signal.SIGINT, lambda *_: window.close())
        signal.signal(signal.SIGTERM, lambda *_: window.close())
        window.run()
    finally:
        if not connection.done.is_set():
            connection.finish()
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print('启动失败：', exc, file=sys.stderr)
        raise SystemExit(1)
