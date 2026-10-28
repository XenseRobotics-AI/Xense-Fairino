#!/usr/bin/env python3
"""Plot offline planned torso derivatives; no ROS or device/network access."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
from matplotlib import font_manager
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import numpy as np

from verify_body_trajectory_offline import (
    CORE, ROOT, DEG, derivative, evaluate, power_coefficients,
    read_first_historical_plan, stage_nodes,
)

COLORS = ['#788b9c', '#9263bb', '#d67027', '#168aa3', '#43865e']
JOINTS = [15,16,17,18,19]
METRICS = [('速度', '°/s', 'peak_speed_deg_s'),
           ('加速度', '°/s²', 'peak_acceleration_deg_s2'),
           ('加速度变化率 jerk', '°/s³', 'peak_jerk_deg_s3')]


def configure_font():
    candidates = [Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'),
                  Path('/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc')]
    for path in candidates:
        if path.exists():
            font_manager.fontManager.addfont(str(path))
            family = font_manager.FontProperties(fname=str(path)).get_name()
            plt.rcParams.update({'font.family': family, 'axes.unicode_minus': False,
                                 'font.size': 11, 'axes.titlesize': 15})
            return str(path)
    raise RuntimeError('需要本地 Noto Sans CJK 或 WenQuanYi 中文字体')


def sample_stage(nodes):
    times, values = [], [[],[],[]]
    for left,right in zip(nodes,nodes[1:]):
        c,h = power_coefficients(left,right)
        u = np.linspace(0.,1.,13)
        times.extend((left['time']-nodes[0]['time']+u*h).tolist())
        for order in range(1,4):
            values[order-1].extend((evaluate(derivative(c,order,h),u)*DEG).tolist())
    return np.asarray(times), [np.asarray(array) for array in values]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report',type=Path,default=ROOT/'logs/20261026-lift-offline-verification/body-trajectory.json')
    parser.add_argument('--output',type=Path,default=ROOT/'logs/20261026-lift-offline-verification/body-trajectory-audit.png')
    args=parser.parse_args()
    font=configure_font()
    report=json.loads(args.report.read_text())
    history=next(case for case in report['cases'] if case['name']=='historical_20261023_171132')
    small=next(case for case in report['cases'] if case['name']=='height_99.9_to_100')
    source=Path(report['history']['path'])
    planned,line=read_first_historical_plan(source)
    if history['stage_count']!=3 or len(planned['points'])!=history['node_count']:
        raise ValueError('图模板要求已核验的历史三阶段轨迹，报告与历史计划不一致')
    fig=plt.figure(figsize=(18,14),facecolor='#f7f9fb')
    grid=fig.add_gridspec(4,3,left=.075,right=.975,top=.81,bottom=.12,
                          height_ratios=[1,1,1,.9],hspace=.62,wspace=.30)
    fig.text(.5,.97,'升降轨迹离线审计 · 规划目标曲线',ha='center',va='top',
             fontsize=25,color='#203545',weight='bold')
    fig.text(.5,.930,'历史任务：2026-10-23 17:11:32  ·  J16–J20  ·  独立五次多项式求导',
             ha='center',fontsize=13,color='#496173')
    fig.text(.5,.904,'每一列使用该阶段的局部时间；横轴、纵轴均单独定标，短阶段未被长阶段压缩',
             ha='center',fontsize=12,color='#496173')
    handles=[Line2D([0],[0],color=color,lw=2.7,label=f'J{joint+1}')
             for joint,color in zip(JOINTS,COLORS)]
    fig.legend(handles=handles,ncol=5,loc='upper center',bbox_to_anchor=(.5,.891),
               frameon=False,fontsize=12)
    for column,stage in enumerate(history['stages']):
        nodes=stage_nodes(planned['points'],column)
        times,values=sample_stage(nodes)
        duration=stage['duration_s']
        for row,(label,unit,peak_key) in enumerate(METRICS):
            ax=fig.add_subplot(grid[row,column],facecolor='white')
            ax.axhline(0,color='#b8c3cb',linewidth=.8,zorder=1)
            for joint,color in zip(JOINTS,COLORS):
                if np.max(np.abs(values[row][:,joint]))>1e-6:
                    ax.plot(times,values[row][:,joint],color=color,lw=2.0,zorder=2)
            ax.set_xlim(0,duration)
            ax.margins(y=.14)
            ax.grid(True,color='#e5eaef',linewidth=.7)
            for spine in ax.spines.values():
                spine.set_color('#cbd5dc')
            ax.tick_params(colors='#506577',labelsize=10)
            ax.set_xlabel('本阶段时间 / s',fontsize=10,color='#506577',labelpad=4)
            ax.set_ylabel(f'{label}\n{unit}',fontsize=11,color='#203545')
            if row==0:
                ax.set_title(f'{column+1}  {stage["label"]}\n持续 {duration:.3f} 秒',
                             loc='left',pad=12,color='#203545')
            peak=max(stage[peak_key])
            ax.text(.98,.95,f'绝对值峰值 {peak:,.3f}',transform=ax.transAxes,
                    ha='right',va='top',fontsize=10,color='#203545',
                    bbox=dict(facecolor='white',alpha=.9,edgecolor='none',pad=3))
    ax=fig.add_subplot(grid[3,:2]);ax.axis('off')
    short=small['stages'][0]
    ax.text(0,1.18,'高位小幅升降：99.9% → 100%（离线构造用例）',
            transform=ax.transAxes,fontsize=15,weight='bold',color='#203545')
    ax.text(0,.99,f'关节速度上限 5°/s；实际运动段 {short["duration_s"]:.3f} 秒；不含前置 0.1 秒',
            transform=ax.transAxes,fontsize=11,color='#496173')
    table_rows=[]
    for label,unit,key in METRICS:
        table_rows.append([f'{label}峰值（{unit}）']+
                          [f'{short[key][j]:,.3f}' for j in (17,18,19)])
    table=ax.table(cellText=table_rows,colLabels=['量','J18','J19','J20'],
                   colWidths=[.46,.18,.18,.18],loc='center',bbox=[0,-.06,1,.89],
                   cellLoc='center')
    table.auto_set_font_size(False);table.set_fontsize(11)
    for (row,col),cell in table.get_celld().items():
        cell.set_edgecolor('#dbe3e9');cell.set_linewidth(.8)
        cell.set_facecolor('#e7eff5' if row==0 else 'white')
        cell.set_text_props(color='#203545',weight='bold' if row==0 else 'normal')
    note=fig.add_subplot(grid[3,2]);note.axis('off')
    note.text(0,1.18,'如何读这张图',transform=note.transAxes,fontsize=15,
              weight='bold',color='#203545')
    note.text(0,.97,'• 零值关节曲线重合于 y=0。\n\n'
              '• 段间位置、速度和加速度连续；\n  jerk 不保证连续。\n\n'
              '• 速度上限不等于加速度或\n  jerk 上限。',
              transform=note.transAxes,fontsize=11,color='#496173',va='top',linespacing=1.35)
    fig.text(.075,.064,'这些曲线是规划目标，不是实测动力响应；数值峰值不能直接等同于实机受力或安全界限。',
             fontsize=13,color='#9a4529',weight='bold')
    fig.text(.075,.038,f'数据：{source.name} 第 {line} 行 body_plan；离线报告 body-trajectory.json。未连接机器人。',
             fontsize=10,color='#647789')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(args.output,dpi=160,facecolor=fig.get_facecolor())
    plt.close(fig)
    print('Font:',font)
    print('Image:',args.output)


if __name__=='__main__':
    main()
