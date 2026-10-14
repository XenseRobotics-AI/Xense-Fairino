#!/usr/bin/env python3
"""Display two independent live laser scans over the robot's SSH connection."""
import argparse
import getpass
import json
import math
import os
import queue
import signal
import shlex
import threading
import time
from pathlib import Path
import tkinter as tk
from tkinter import ttk
import pexpect

ROOT=Path(__file__).resolve().parent
SERIALS=['20260907-000045','20260907-000046']
POSITIONS={'20260907-000045':'前雷达','20260907-000046':'后雷达'}

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--host',default='192.168.58.3')
    p.add_argument('--user',default='fairino')
    args=p.parse_args()
    password=os.environ.pop('FAIRINO_SSH_PASSWORD',None)
    if password is None:password=getpass.getpass(f'SSH 密码（{args.user}@{args.host}）: ')
    root=tk.Tk();root.title('法奥双雷达实时扫描 · LS3-2027DE');root.geometry('1160x700')
    ttk.Label(root,text='双雷达实时激光图',font=('',21,'bold')).pack(pady=(13,5))
    ttk.Label(root,text='左：前雷达 .101　右：后雷达 .100；两幅图使用各自雷达坐标系。').pack()
    toolbar=ttk.Frame(root);toolbar.pack(fill='x',padx=20,pady=10)
    ttk.Label(toolbar,text='显示半径（米）').pack(side='left')
    extent=tk.DoubleVar(value=8)
    ttk.Scale(toolbar,from_=1,to=20,variable=extent,orient='horizontal',length=250).pack(side='left',padx=8)
    freeze=tk.BooleanVar(value=False);ttk.Checkbutton(toolbar,text='暂停画面',variable=freeze).pack(side='left',padx=20)
    status=tk.StringVar(value='正在连接机器人…');ttk.Label(toolbar,textvariable=status).pack(side='right')
    body=ttk.Frame(root);body.pack(fill='both',expand=True,padx=12)
    canvases={};labels={};latest={};arrival={}
    for serial in SERIALS:
        pane=ttk.Frame(body);pane.pack(side='left',fill='both',expand=True,padx=5)
        text=tk.StringVar(value=POSITIONS[serial]+' · '+serial+' · 等待数据');labels[serial]=text
        ttk.Label(pane,textvariable=text,anchor='center').pack(fill='x',pady=5)
        canvas=tk.Canvas(pane,bg='#f9fbfd',highlightthickness=1,highlightbackground='#ced9e2')
        canvas.pack(fill='both',expand=True);canvases[serial]=canvas
    ttk.Label(root,text='蓝点：测到的物体位置　橙色三角：雷达位置　单位：米；关闭窗口后，机器人雷达服务继续运行。').pack(pady=10)
    events=queue.Queue(maxsize=30);stopping=threading.Event();holder={}
    log=ROOT/'viewer-latest.jsonl'
    def enqueue(item):
        try:events.put_nowait(item)
        except queue.Full:
            try:events.get_nowait()
            except queue.Empty:pass
            events.put_nowait(item)
    def worker():
        child=None
        try:
            script=(ROOT/'remote_stream.py').read_text()
            command='source /opt/ros/humble/setup.bash && exec env -u FASTRTPS_DEFAULT_PROFILES_FILE ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp python3 -u -c '+shlex.quote(script)
            remote='bash -c '+shlex.quote(command)
            child=pexpect.spawn('ssh',['-o','ConnectTimeout=5','-o','StrictHostKeyChecking=accept-new',
                                    '-o','UserKnownHostsFile=/tmp/fairino-known-hosts',
                                    '-o','ServerAliveInterval=3','-o','ServerAliveCountMax=2',
                                    args.user+'@'+args.host,remote],encoding='utf-8',timeout=3,maxread=65536)
            holder['child']=child
            state=child.expect(['(?i)password:',pexpect.EOF,pexpect.TIMEOUT],timeout=8)
            if state==0:child.sendline(password)
            elif state==1:raise RuntimeError('SSH 连接失败：'+child.before[-300:])
            with log.open('w') as f:
                seen=set()
                while not stopping.is_set():
                    try:line=child.readline()
                    except pexpect.TIMEOUT:continue
                    if not line:break
                    try:item=json.loads(line)
                    except json.JSONDecodeError:continue
                    enqueue(item)
                    if item.get('type')!='scan':
                        f.write(json.dumps(item,ensure_ascii=False)+'\n');f.flush()
                    elif item['serial'] not in seen:
                        seen.add(item['serial'])
                        first={k:v for k,v in item.items() if k!='ranges_m'}
                        first['type']='scan_ready'
                        f.write(json.dumps(first,ensure_ascii=False)+'\n');f.flush()
            if not stopping.is_set():enqueue({'type':'error','message':'连接已结束'})
        except Exception as e:enqueue({'type':'error','message':str(e)})
        finally:
            if child is not None and child.isalive():
                child.sendintr()
                try:child.expect(pexpect.EOF,timeout=5)
                except Exception:child.close(force=True)
    thread=threading.Thread(target=worker,daemon=True);thread.start()
    def draw(serial,data):
        c=canvases[serial];w=c.winfo_width();h=c.winfo_height();r=extent.get()
        scale=min(w-52,h-52)/(2*r);cx=w/2;cy=h/2
        c.delete('all')
        step=1 if r<=5 else 2 if r<=10 else 5
        for value in range(-int(r),int(r)+1,step):
            x=cx+value*scale;y=cy-value*scale
            c.create_line(x,cy-r*scale,x,cy+r*scale,fill='#dfe7ef')
            c.create_line(cx-r*scale,y,cx+r*scale,y,fill='#dfe7ef')
            c.create_text(x,cy+r*scale+14,text=str(value),fill='#697c8c',font=('',9))
            if value:c.create_text(cx-r*scale-16,y,text=str(value),fill='#697c8c',font=('',9))
        c.create_line(cx-r*scale,cy,cx+r*scale,cy,fill='#aebdcb')
        c.create_line(cx,cy-r*scale,cx,cy+r*scale,fill='#aebdcb')
        for i in range(0,len(data['ranges_m']),2):
            rr=data['ranges_m'][i]
            if rr is None or rr>r:continue
            angle=data['angle_min']+i*data['angle_increment']
            x=cx+rr*math.cos(angle)*scale;y=cy-rr*math.sin(angle)*scale
            c.create_oval(x-1.5,y-1.5,x+1.5,y+1.5,fill='#087fab',outline='')
        c.create_polygon(cx+8,cy,cx-5,cy-5,cx-5,cy+5,fill='#e87532')
        c.create_text(w-25,cy+14,text='+X',fill='#647b8a')
        c.create_text(cx+15,17,text='+Y',fill='#647b8a')
    def tick():
        if stopping.is_set():return
        while True:
            try:item=events.get_nowait()
            except queue.Empty:break
            if item.get('type')=='scan':
                serial=item['serial'];latest[serial]=item;arrival[serial]=time.monotonic()
                labels[serial].set(f"{POSITIONS[serial]} · {serial} · {item['ip']}\n{item['hz']:.1f} Hz · {item['points']} 点/帧")
            elif item.get('type')=='error':status.set(item['message'])
        if len(arrival)==2:
            age=max(time.monotonic()-t for t in arrival.values())
            status.set('实时接收中' if age<2 else f'数据未更新：{age:.1f} 秒')
        if not freeze.get():
            for serial,data in latest.items():draw(serial,data)
        root.after(150,tick)
    def close():
        stopping.set();status.set('正在断开显示连接…')
        child=holder.get('child')
        if child is not None and child.isalive():child.sendintr()
        root.after(200,wait_close)
    def wait_close():
        if thread.is_alive():root.after(200,wait_close)
        else:root.destroy()
    signal.signal(signal.SIGTERM,lambda *_:root.after(0,close))
    root.protocol('WM_DELETE_WINDOW',close);root.after(100,tick);root.mainloop()

if __name__=='__main__':main()
