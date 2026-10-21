#!/usr/bin/env python3
"""Six-axis TCP nudges: vendor FK/IK locally, guarded ROS execution remotely."""
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

HERE=Path(__file__).resolve().parent
SHARED=HERE.parent/'fairino_upper_probe'
sys.path.insert(0,str(SHARED))
from upper_keyboard import Connection
from tcp_core import MODEL_ID,ARMS,FLANGE_OFFSET_MM,matrix_pose
from tcp_frames import CHEST_FRAME_ID,tcp_in_chest
from tcp_jog import (KEYS,KEY_PAIRS,MAX_SPEED,DEFAULT_SPEED,MAX_LINEAR,MAX_ANGULAR,
                     make_request,validate_velocity,trajectory,sample)
from vendor_kinematics import VendorKinematics
from tool_frames import for_arm, tool_config

AXES=('X','Y','Z','Rx','Ry','Rz')


def remote_command(args):
    bundle={n:(SHARED/n).read_text() for n in ('wrist_guard.py','probe_core.py','robot_probe.py')}
    bundle.update({n:(HERE/n).read_text() for n in ('tcp_core.py','tcp_frames.py','tcp_jog.py','tcp_robot.py')})
    encoded=base64.b64encode(zlib.compress(json.dumps(bundle).encode())).decode()
    argv=['tcp_robot.py']
    if args.check:argv.append('--check')
    if args.prepare_check:argv.append('--prepare-check')
    loader=('import base64,zlib,json,sys,types;'
            f'bundle=json.loads(zlib.decompress(base64.b64decode({encoded!r})));'
            "\nfor name in ('wrist_guard','probe_core','robot_probe','tcp_core','tcp_frames','tcp_jog'):\n"
            " m=types.ModuleType(name);sys.modules[name]=m;exec(compile(bundle[name+'.py'],name+'.py','exec'),m.__dict__)\n"
            f'sys.argv={argv!r}\n'
            "exec(compile(bundle['tcp_robot.py'],'tcp_robot.py','exec'),{'__name__':'__main__'})")
    setup='\n'.join(['set -e','source /opt/ros/humble/setup.bash',
        'source /opt/fairino_art_plugin/plugin/install/setup.bash',
        'export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
        'export FASTRTPS_DEFAULT_PROFILES_FILE=/home/fairino/fastrtps_udp.xml',
        'echo UPPERPROBE_CONNECTED','exec python3 -u -c '+shlex.quote(loader)])
    return 'bash -c '+shlex.quote(setup)


def verify_both(kin,hw):
    details={}
    for arm,joints in ARMS.items():
        q=[hw['position'][j-1] for j in joints]
        result=kin.verify_tcp(q,arm)
        tools = getattr(kin, 'tools', tool_config())
        offset = for_arm(tools, arm)
        definition = 'flange_face_center' if tools['mode'] == 'flange' else 'fixed_gripper_tip_center'
        details[arm]={'tcp_definition':definition,
                      'offset_mm':[FLANGE_OFFSET_MM[i]+offset[i][3] for i in range(3)],
                      'coordinate_frame':CHEST_FRAME_ID,
                      'pose_mm_deg':matrix_pose(tcp_in_chest(arm,result['matrix'])),
                      'base_pose_mm_deg':matrix_pose(result['matrix']),
                      'arm_angle_deg':result['arm_angle'],'roundtrip_deg':result['roundtrip_deg']}
    return {'model':MODEL_ID,'reference':hw['position'][:14],
            'tool_id':getattr(kin, 'tools', tool_config())['id']},details


class Window:
    def __init__(self,args,kin,connection=None,window_factory=None,on_closed=None,start_timer=True):
        import gi
        gi.require_version('Gtk','3.0')
        from gi.repository import Gtk,Gdk,GLib
        self.Gtk,self.Gdk,self.GLib=Gtk,Gdk,GLib
        self.args,self.kin,self.connection=args,kin,connection
        self.arm=args.arm;self.sequence=0;self.request=None;self.cancel_count=0
        self.ready=False;self.stopped=False;self.closing=False;self.had_focus=False
        self.input_suspended=False
        self.held=set();self.releases={};self.active_key=None;self.input_blocked=False
        self.awaiting_stop=False;self.stop_stamp=0.;self.jogging=False
        self.hw=None;self.proof=None;self.last_status=0.;self.verified=None;self.last_plan=0.
        self.demo_path=None;self.demo_time=0.
        self.rotation_anchor=None;self.previous_rotation_request=None
        self.ui_closed=on_closed or Gtk.main_quit
        self.window=(window_factory or Gtk.Window)(title='法奥双臂 · 胸部坐标 · 法兰中心 TCP 十二键连续控制')
        self.window.set_default_size(800,620);self.window.set_border_width(16)
        box=Gtk.Box(orientation=Gtk.Orientation.VERTICAL,spacing=10);self.window.add(box)
        from wrist_guard_ui import GuardControls
        self.wrist_controls = GuardControls(self.switch_wrist)
        self.wrist_controls.toggle.set_sensitive(connection is not None)
        box.pack_start(self.wrist_controls,False,False,0)
        title=Gtk.Label();title.set_markup('<span size="18000" weight="bold">胸部坐标 TCP · 十二键连续控制</span>')
        box.pack_start(title,False,False,0)
        box.pack_start(Gtk.Label(label='W/S 前/后 · A/D 左/右 · R/F 上/下（相对胸部，左右臂相同）'),False,False,0)
        box.pack_start(Gtk.Label(label='长按一个方向键持续运动，松键减速停止；空格停止并保持，Esc 退出。'),False,False,0)
        # A ComboBox popup can deactivate the toplevel on X11 and trigger the
        # real focus-loss stop. Inline radio buttons never create that popup.
        arm_row=Gtk.Box(spacing=16);arm_row.pack_start(Gtk.Label(label='控制手臂'),False,False,0)
        self.arm_buttons={};group=None
        for arm,label in (('left','左臂 J1–J7'),('right','右臂 J8–J14')):
            button=Gtk.RadioButton.new_with_label_from_widget(group,label)
            if group is None:group=button
            button.set_can_focus(False);button.set_active(arm==self.arm)
            self.arm_buttons[arm]=button;arm_row.pack_start(button,False,False,0)
        for arm,button in self.arm_buttons.items():button.connect('toggled',self.change_arm,arm)
        box.pack_start(arm_row,False,False,0)
        row=Gtk.Box(spacing=10)
        self.translation=self.spin(row,'平移 mm/s',.1,MAX_LINEAR,MAX_LINEAR)
        self.rotation=self.spin(row,'旋转 °/s',.1,MAX_ANGULAR,MAX_ANGULAR)
        self.speed=self.spin(row,'关节上限 °/s',.1,MAX_SPEED,DEFAULT_SPEED)
        box.pack_start(row,False,False,0)
        self.phase=Gtk.Label(label='读取反馈并校验厂家正逆解；尚未使能')
        self.phase.set_line_wrap(True);box.pack_start(self.phase,False,False,0)
        self.enabled=Gtk.Label(label='等待反馈');box.pack_start(self.enabled,False,False,0)
        grid=Gtk.Grid(column_spacing=20,row_spacing=8);box.pack_start(grid,True,True,0)
        for col,label in enumerate(('方向','胸部 TCP 值','负向 · 长按','正向 · 长按')):
            grid.attach(Gtk.Label(label=label),col,0,1,1)
        self.values={};self.motion_buttons=[]
        for i,name in enumerate(AXES):
            grid.attach(Gtk.Label(label=name),0,i+1,1,1)
            value=Gtk.Label(label='—');grid.attach(value,1,i+1,1,1);self.values[i]=value
            for col,key in enumerate(KEY_PAIRS[i],2):
                b=Gtk.Button(label=f'{key.upper()}   {name} '+('−' if col==2 else '＋'))
                b.set_can_focus(False)
                b.connect('pressed',lambda _,k=key:self.press(k))
                b.connect('released',lambda _,k=key:self.release(k))
                grid.attach(b,col,i+1,1,1);self.motion_buttons.append(b)
        controls=Gtk.Box(spacing=10)
        self.control_area=Gtk.Button(label='返回方向键控制')
        self.control_area.connect('clicked',lambda *_:self.control_area.grab_focus())
        controls.pack_start(self.control_area,True,True,0)
        cancel=Gtk.Button(label='停止并保持 · 空格');cancel.set_can_focus(False)
        cancel.connect('clicked',lambda *_:self.cancel());controls.pack_start(cancel,True,True,0)
        box.pack_start(controls,False,False,0)
        self.error=Gtk.Label();self.error.set_line_wrap(True);box.pack_start(self.error,False,False,0)
        footer=Gtk.Label(label='TCP：法兰安装面中心，已补偿腕心偏移 [-106, 0, 0] mm；不含夹爪长度。\n'
            '坐标：胸部 X 前 / Y 左 / Z 上，随胸部倾斜；旋转绕胸部轴，保持法兰中心。\n'
            '另一臂和躯干保持。失焦只停止运动并保持使能；Esc、关闭窗口或故障时下使能。')
        footer.set_line_wrap(True);box.pack_start(footer,False,False,0)
        self.window.connect('key-press-event',self.key_press)
        self.window.connect('key-release-event',self.key_release)
        self.window.connect('focus-in-event',self.focus_in)
        self.window.connect('focus-out-event',self.focus_out)
        self.window.connect('delete-event',self.close)
        self.refresh_controls();self.window.show_all();self.control_area.grab_focus();self.window.present()
        self.timer=GLib.timeout_add(50,self.tick) if start_timer else None

    def spin(self,row,label,lo,hi,value):
        row.pack_start(self.Gtk.Label(label=label),False,False,0)
        spin=self.Gtk.SpinButton.new_with_range(lo,hi,.1);spin.set_digits(1)
        spin.set_numeric(True);spin.set_value(value)
        row.pack_start(spin,False,False,0);return spin

    def change_arm(self,button,arm):
        if not button.get_active():return
        if self.active_key or self.awaiting_stop or self.jogging or self.stopped:
            self.arm_buttons[self.arm].set_active(True);return
        self.arm=arm
        self.control_area.grab_focus()
        if self.hw:self.update_hardware(self.hw)

    def refresh_controls(self):
        busy=bool(self.active_key or self.awaiting_stop or self.jogging)
        for w in (*self.arm_buttons.values(),self.translation,self.rotation,self.speed):
            w.set_sensitive(not busy and not self.stopped)
        # Do not disable a pressed mouse button: its release must be delivered.
        for b in self.motion_buttons:b.set_sensitive(not self.stopped)

    def press(self,key):
        if key in self.held:return
        self.held.add(key)
        if len(self.held)>1:
            self.input_blocked=True;self.stop_hold('同时按下多个方向键，已停止；请全部松开');return
        if (self.input_blocked or self.input_suspended or not self.ready or self.stopped or self.awaiting_stop
                or self.proof is None or not self.window.is_active()):
            self.input_blocked=True;return
        for spin in (self.translation,self.rotation,self.speed):spin.update()
        self.active_key=key;self.last_plan=0.;self.error.set_text('')
        self.rotation_anchor=None;self.previous_rotation_request=None
        self.generate_request();self.refresh_controls();self.send()

    def release(self,key):
        self.held.discard(key)
        if key==self.active_key:self.stop_hold('已松键')
        if not self.held:self.input_blocked=False
        self.refresh_controls()

    def generate_request(self):
        if not self.active_key or self.hw is None or self.input_suspended:return
        if self.connection and time.monotonic()-self.last_status>.25:
            self.input_blocked=True;self.stop_hold('反馈过期，已停止；请松键后重按');return
        try:
            axis,direction=KEYS[self.active_key]
            r=make_request(self.kin,self.hw,self.arm,axis,direction,
                           self.translation.get_value(),self.rotation.get_value(),self.speed.get_value(),
                           rotation_anchor=self.rotation_anchor)
            self.sequence+=1
            r.update(id=self.sequence,created=self.connection.latest.get('stamp',0) if self.connection else time.monotonic())
            validate_velocity(self.hw,r,r['created'],previous=self.previous_rotation_request)
            self.rotation_anchor=r['rotation_anchor'];self.previous_rotation_request=r
            self.request=r;self.last_plan=time.monotonic()
            self.phase.set_text(('离线模拟：' if self.args.demo else '')+
                f'{self.active_key.upper()} 长按中 · {AXES[axis]} {"＋" if direction>0 else "−"}；松键停止')
        except (ValueError,OSError) as exc:
            self.input_blocked=True;self.error.set_text(str(exc));self.stop_hold('解算停止，请松键后重按')

    def stop_hold(self,reason,cancel_request=True):
        moving=self.active_key is not None or self.request is not None or self.jogging
        self.active_key=None;self.request=None
        if cancel_request:self.cancel_count+=1
        self.rotation_anchor=None;self.previous_rotation_request=None
        if moving and self.connection:
            self.awaiting_stop=True;self.stop_stamp=self.connection.latest.get('stamp',0)
        if self.args.demo:self.demo_path=None
        self.phase.set_text(reason+'；停止并保持位置')
        self.send();self.refresh_controls()

    def switch_wrist(self, enabled):
        if self.connection:
            self.connection.wrist.switch(enabled)
            self.stop_hold('切换腕部模块，请松键后重按')
            self.input_blocked = bool(self.held)
            self.ready = False
            self.refresh_controls()

    def key_press(self,_,event):
        key=(self.Gdk.keyval_name(event.keyval) or '').lower()
        if key=='escape':self.close();return True
        if key=='space':self.cancel();return True
        if key not in KEYS:return False
        if key in self.releases:self.GLib.source_remove(self.releases.pop(key))
        modifiers=getattr(event,'state',0)
        if modifiers & (self.Gdk.ModifierType.CONTROL_MASK|self.Gdk.ModifierType.MOD1_MASK|self.Gdk.ModifierType.SUPER_MASK):
            self.cancel();return False
        if isinstance(self.window.get_focus(),self.Gtk.Entry):return False
        self.press(key);return True

    def key_release(self,_,event):
        key=(self.Gdk.keyval_name(event.keyval) or '').lower()
        if key not in KEYS:return False
        # X11 autorepeat can emit a synthetic release immediately before press.
        # Defer 25 ms: repeat press cancels this, a real release stops the stream.
        if key not in self.releases:
            def released():
                self.releases.pop(key,None);self.release(key);return False
            self.releases[key]=self.GLib.timeout_add(25,released)
        return True

    def cancel(self):
        self.input_blocked=bool(self.held);self.stop_hold('用户停止')

    def focus_in(self,*_):
        self.had_focus=True;self.input_suspended=False
        if not self.stopped and self.ready and self.active_key is None:
            self.phase.set_text('已返回控制窗口；松开方向键后重新按下即可继续')
        return False

    def focus_out(self,*_):
        if self.had_focus:
            self.input_suspended=True
            # Keep held keys latched until release, so autorepeat on focus
            # return cannot resurrect the previous motion command.
            self.input_blocked=bool(self.held)
            # Focus pauses movement, but is not a user cancellation of enable.
            self.stop_hold('窗口失焦，暂停运动并保持使能；切回后松键重按',cancel_request=False)
        return False

    def send(self):
        if self.connection:
            focused=self.window.is_active() and not self.input_suspended
            self.connection.send(self.request if focused else None,self.cancel_count,focused=focused,
                                 exit=self.stopped,kinematics=self.proof)

    def update_hardware(self,hw):
        self.hw=hw;self.last_status=time.monotonic()
        if self.proof is None:self.proof,self.verified=verify_both(self.kin,hw)
        q=[hw['position'][j-1] for j in ARMS[self.arm]]
        pose=matrix_pose(tcp_in_chest(self.arm,self.kin.forward_tcp(q)[0]))
        for i,v in enumerate(pose):self.values[i].set_text(f'{v:+.3f} '+('mm' if i<3 else '°'))
        self.enabled.set_text(f'上半身使能 {sum(hw["enabled"])}/20 · 厂家正逆解已校验')

    def tick(self):
        if self.closing:return False
        if self.connection:
            while True:
                try:data=self.connection.events.get_nowait()
                except queue.Empty:break
                if 'error' in data:
                    self.error.set_text(data['error']);self.stopped=True;self.ready=False
                    self.stop_hold('控制已结束')
                if 'jog_rejected' in data:
                    self.error.set_text(data['jog_rejected']);self.input_blocked=bool(self.held)
                    self.stop_hold('机器人拒绝本次运动')
                if 'phase' in data:
                    self.phase.set_text(data['phase'])
                    self.ready=(data.get('ready',False) and not self.stopped
                                and self.connection.wrist.ready(data.get('wrist_guard', {}))
                                and self.connection.wrist.released)
                    self.jogging=data.get('jogging',False)
                    if self.ready and not self.jogging and data.get('stamp',0)>self.stop_stamp:
                        self.awaiting_stop=False
                if data.get('hardware'):
                    try:self.update_hardware(data['hardware'])
                    except (ValueError,OSError) as exc:
                        self.error.set_text(str(exc));self.stopped=True;self.ready=False;self.stop_hold('解算校验失败')
                if data.get('connection_closed'):
                    self.stopped=True;self.ready=False;self.active_key=None;self.request=None
                    self.phase.set_text('控制会话已结束；重新启动程序可重新连接')
                    if not self.connection.latest.get('disabled_confirmed'):
                        self.error.set_text('连接已关闭，未收到全部下使能确认，请现场检查')
        if self.active_key and not self.stopped and time.monotonic()-self.last_plan>=.08:
            self.generate_request()
        if self.args.demo and self.request and self.hw:
            try:
                now=time.monotonic();velocity=validate_velocity(self.hw,self.request,now,
                    previous=self.previous_rotation_request)
                q,v=sample(self.demo_path,now-self.demo_time) if self.demo_path else (self.hw['position'][:],[0.]*20)
                self.demo_path=trajectory(q,v,velocity,ARMS[self.arm],self.hw['command_position'])
                self.demo_time=now;self.hw['position']=q;self.hw['command_position']=q[:]
                self.update_hardware(self.hw)
            except ValueError as exc:self.error.set_text(str(exc));self.stop_hold('离线模拟停止')
        if self.connection:
            state = self.connection.latest.get('wrist_guard', {})
            self.wrist_controls.update(state, not self.connection.wrist.ready(state))
        self.send();self.refresh_controls();return True

    def close(self,*_):
        if self.closing:return True
        self.closing=self.stopped=True;self.ready=False;self.active_key=None;self.request=None;self.send()
        for timer in self.releases.values():self.GLib.source_remove(timer)
        self.releases.clear()
        if self.connection:
            def finish():
                self.connection.finish();self.GLib.idle_add(self.ui_closed)
            threading.Thread(target=finish,daemon=True).start()
        else:self.ui_closed()
        return True

    def run(self):self.Gtk.main()


def main():
    parser=argparse.ArgumentParser(description='厂家七轴解算 + ROS TCP 十二键连续控制')
    parser.add_argument('--host',default='192.168.58.1');parser.add_argument('--user',default='fairino')
    parser.add_argument('--arm',choices=tuple(ARMS),default='left')
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--check',action='store_true');modes.add_argument('--prepare-check',action='store_true')
    modes.add_argument('--demo',action='store_true')
    args=parser.parse_args()
    if any(s.startswith('-') or '@' in s for s in (args.host,args.user)):parser.error('无效 SSH 参数')
    kin=VendorKinematics()
    if args.demo:
        w=Window(args,kin)
        q=[math.radians(v) for v in (93,89.7,-90,104.8,0,0,0,-86,89.7,90,105.1,0,0,0,10.6,0,0,-48.9,91.2,-43.1)]
        w.update_hardware({'position':q,'command_position':q[:],'enabled':[False]*20})
        w.ready=True;w.phase.set_text('离线演示：不连接机器人');w.refresh_controls();w.run();return 0
    checking=args.check or args.prepare_check
    print('检查模式：不使能、不运动。' if checking else '连接后校验厂家正逆解；窗口就绪会使能全部 J1–J20。')
    print('TCP 已设为法兰安装面中心：腕心偏移 [-106, 0, 0] mm，不含夹爪长度。')
    print('坐标已对齐胸部：X 前 / Y 左 / Z 上，左右臂相同；随胸部倾斜，不是固定地面坐标。')
    print('安装映射：'+CHEST_FRAME_ID)
    print('长按 W/S A/D R/F I/K J/L U/O 控制六维正负方向，松键减速停止；默认 2°/s，上限 4°/s。')
    connection=Connection(args,worker_command=remote_command(args),log_directory=HERE/'logs')
    print('日志：',connection.log_path)
    if checking:
        passed=False;failed=False;last_hw=None
        end=time.monotonic()+45
        while not connection.done.is_set() and time.monotonic()<end:
            connection.send();time.sleep(.05)
        while not connection.events.empty():
            data=connection.events.get_nowait()
            if data.get('hardware'):last_hw=data['hardware']
            if data.get('checked'):passed=True
            if 'error' in data:failed=True;print('失败：',data['error'])
        connection.finish()
        if passed and last_hw:
            _,details=verify_both(kin,last_hw)
            print(json.dumps({'model':MODEL_ID,'kinematics':details,'enabled':sum(last_hw['enabled'])},ensure_ascii=False,indent=2))
        return 0 if passed and not failed else 1
    try:
        w=Window(args,kin,connection)
        signal.signal(signal.SIGINT,lambda *_:w.close());signal.signal(signal.SIGTERM,lambda *_:w.close())
        w.run()
    finally:
        if not connection.done.is_set():connection.finish()
    return 0


if __name__=='__main__':
    try:raise SystemExit(main())
    except (OSError,RuntimeError,ValueError) as exc:print('启动失败：',exc,file=sys.stderr);raise SystemExit(1)
