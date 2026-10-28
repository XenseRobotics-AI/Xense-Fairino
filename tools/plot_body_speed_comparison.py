#!/usr/bin/env python3
"""Compare saved torso trajectories and measured joint feedback, offline only."""
from __future__ import annotations

import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from plot_body_trajectory_audit import configure_font

ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT / 'apps/fairino_upper_probe/body_control'
sys.path.insert(0, str(CORE))
from body_core import load_config


def snapshot(path):
    data = path.read_bytes()
    rows = [json.loads(line) for line in data.splitlines() if line.strip()]
    return rows


def select(rows, index, model):
    events = [i for i, row in enumerate(rows) if 'body_plan' in row]
    event = events[index-1]
    end = events[index] if index < len(events) else len(rows)
    record = rows[event]
    plan = record['body_plan']
    stage = plan['stages'].index('升降与姿态调节')
    nodes = plan['points'][100*stage:100*(stage+1)+1]
    start = record['client_received_monotonic'] + nodes[0]['time']
    planned_end = record['client_received_monotonic'] + plan['duration']
    states = [r for r in rows[event+1:end] if r.get('hardware')]
    first_motion = next(r['phase'] for r in states)
    stopped = next((r for r in states if r['phase'] != first_motion), None)
    observed_end = stopped['client_received_monotonic'] if stopped else planned_end
    cap = min(planned_end, observed_end) + 1.2
    fresh = []
    previous_hardware_stamp = None
    for r in states:
        hardware_stamp = round(r['stamp']-r['hw_age'], 7)
        adjusted_time = r['client_received_monotonic']-r['hw_age']
        if hardware_stamp == previous_hardware_stamp:
            continue
        previous_hardware_stamp = hardware_stamp
        if start <= adjusted_time <= cap:
            fresh.append(r)
    states = fresh
    t = np.array([r['client_received_monotonic']-r['hw_age']-start for r in states])
    actual = np.array([r['hardware']['position'] for r in states])
    command = np.array([r['hardware']['command_position'] for r in states])
    height = np.array([model.state(q)['height'] for q in actual])
    target_height = np.array([model.state(q)['height'] for q in command])
    pitch_error = np.degrees((actual-command)[:,17:20].sum(axis=1))
    j20_error = np.degrees((actual-command)[:,19])
    return dict(plan=plan, t=t, actual_height=height, command_height=target_height,
                pitch_error=pitch_error, j20_error=j20_error,
                observed_end=observed_end-start, planned_end=planned_end-start,
                final_phase=stopped['phase'] if stopped else '未知',
                sample_count=len(states), start_height=model.state(nodes[0]['positions'])['height'])


def main():
    configure_font()
    model = load_config()
    old = snapshot(CORE/'logs/20261023-171132.jsonl')
    new = snapshot(CORE/'logs/20261026-092322.jsonl')
    cases = [select(old,1,model), select(new,11,model), select(new,5,model)]
    labels = ['10月23日 · 5°/s · 高位下降',
              '10月26日 · 30°/s · 高位下降（第11次）',
              '10月26日 · 30°/s · 短距离下降（第5次）']
    fig, axes = plt.subplots(3, 3, figsize=(18,11), facecolor='#f7f9fb')
    fig.subplots_adjust(left=.065,right=.97,top=.83,bottom=.15,hspace=.44,wspace=.27)
    fig.suptitle('升降加速前后 · 指令与实际反馈',fontsize=24,color='#203545',weight='bold',y=.97)
    fig.text(.5,.925,'去除重复反馈、按反馈年龄近似对齐；各列独立定标；虚线为完成 / 中断通知，保留之后反馈',
             ha='center',fontsize=12,color='#496173')
    for col,(case,label) in enumerate(zip(cases,labels)):
        t=case['t']
        ax=axes[0,col]
        ax.plot(t,case['command_height'],color='#61768b',ls='--',label='硬件接口目标')
        ax.plot(t,case['actual_height'],color='#148b9c',lw=1.8,label='关节反馈推算')
        outcome = '（途中取消）' if col == 0 else '（完成）'
        ax.set_title(label+'\n'+f"计划 {case['start_height']:.1f}% → {case['plan']['target']['height']:.1f}% {outcome}",fontsize=12,pad=11)
        ax.set_ylabel('模型高度 / %')
        ax.legend(fontsize=9,frameon=False)
        axes[1,col].plot(t,case['pitch_error'],color='#9c5d94',lw=1.4,marker='.',ms=3)
        axes[1,col].set_ylabel('俯仰跟踪误差 / °\nJ18–J20 角度误差之和')
        axes[2,col].plot(t,case['j20_error'],color='#c27631',lw=1.4,marker='.',ms=3)
        axes[2,col].set_ylabel('J20 实际 − 指令 / °')
        for ax in axes[:,col]:
            ax.axvline(case['observed_end'],color='#738294',ls=':',lw=1.2)
            ax.grid(True,color='#e0e7ed',lw=.7)
            ax.set_xlabel('主升降开始后 / s',fontsize=10)
            ax.set_facecolor('white')
            ax.tick_params(labelsize=9)
            for spine in ax.spines.values():spine.set_color('#cbd5dc')
        axes[0,col].text(.98,.03,f"采样点 {case['sample_count']} 个",ha='right',transform=axes[0,col].transAxes,fontsize=9)
    fig.text(.065,.085,'注意：角度误差的缓慢变化通常包含跟踪滞后，不能直接当作振动幅度。两次初始姿态和升降范围也不完全相同。',
             fontsize=12,color='#9a4529')
    fig.text(.065,.052,'约 10 Hz 状态日志，仅能观察所采到的反馈变化；俯仰为关节模型推算，不是 IMU 测量。未连接机器人、未修改控制参数。',
             fontsize=11,color='#496173')
    path=ROOT/'logs/20261026-lift-speed-comparison/speed-comparison.png'
    path.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(path,dpi=160)
    plt.close(fig)
    print(path)


if __name__=='__main__':
    main()
