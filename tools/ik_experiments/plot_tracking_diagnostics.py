"""Plot saved offline tracking traces; no robot or external services."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'logs/20261026-native-pico-history/error_analysis'
plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False})
fig,axes=plt.subplots(2,1,figsize=(11.8,7.4),sharex=True,layout='constrained')
colors=('#1766ad','#d45b18')
for filename,label,color in zip(('trace-10-30-lag60_delay40.jsonl','trace-30-120-lag60_delay40.jsonl'),
                              ('10 deg/s; 30 deg/s²','30 deg/s; 120 deg/s²'),colors):
    time=np.arange(2001)*.01; values=np.full((len(time),2),np.nan)
    rows=[json.loads(line) for line in (OUT/filename).read_text().splitlines()]
    for row in rows:
        index=round(row['t']/.01)
        if index<len(time): values[index]=row['error_mm_deg']
    for i,ax in enumerate(axes): ax.plot(time,values[:,i],color=color,lw=1.6,label=label)
for ax in axes:
    for a,b in ((1.95,3.06),(7.43,8.19)): ax.axvspan(a,b,color='#dddddd',alpha=.6,zorder=-1)
    ax.grid(alpha=.22);ax.set_xlim(0,20);ax.set_ylim(bottom=0)
    ax.set_xticks(np.arange(0,21,2))
axes[0].set_ylabel('TCP position error (mm)')
axes[1].set_ylabel('TCP orientation error (deg)')
axes[1].set_xlabel('Time in archived window (s)')
axes[0].legend(loc='upper left',ncols=2)
axes[0].scatter([11.08],[118.40947],color=colors[0],zorder=3)
axes[0].annotate('118.4 mm at 11.08 s',xy=(11.08,118.40947),xytext=(12.3,116),
                arrowprops={'arrowstyle':'->','color':colors[0]},color=colors[0])
axes[1].scatter([17.39],[69.93948],color=colors[0],zorder=3)
axes[1].annotate('69.94 deg at 17.39 s',xy=(17.39,69.93948),xytext=(10.5,68),
                arrowprops={'arrowstyle':'->','color':colors[0]},color=colors[0])
fig.suptitle('Same archived TCP goals: different joint motion budgets\n'
             'Simulated response: tau 60 ms + feedback delay 40 ms; gray intervals have no active goal',fontsize=13)
fig.savefig(OUT/'tracking_errors.png',dpi=180)
fig.savefig(OUT/'tracking_errors.pdf')
print(OUT/'tracking_errors.png')
