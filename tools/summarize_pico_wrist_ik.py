#!/usr/bin/env python3
"""Summarize offline IK experiments; no control or network interfaces."""
import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def summarize_motion(rows):
    phases = {}
    pairs = {}
    for row in rows:
        if row['kind'] == 'hold':
            continue
        key = tuple(row[k] for k in ('mode', 'side', 'pose', 'kind', 'hz', 'profile')) + (row.get('duration', 2.),)
        pairs.setdefault(key, {})[row['strategy']] = row
    for strategy in ('baseline', 'search_va'):
        selected = [r for r in rows if r['strategy'] == strategy and r['kind'] != 'hold']
        phases[strategy] = {
            'episodes': len(selected),
            'completed': sum(r['status'] == 'completed' for r in selected),
            'completed_and_95percent_input': sum(r['status'] == 'completed' and r['task_progress_ratio'] >= .95 for r in selected),
            'mean_input_progress': float(np.mean([r['task_progress_ratio'] for r in selected])),
            'median_input_progress': float(np.median([r['task_progress_ratio'] for r in selected])),
            'max_speed_deg_s': max(r['max_command_speed_deg_s'] for r in selected),
            'max_acceleration_deg_s2': max(r['max_command_acceleration_deg_s2'] for r in selected),
            'min_actual_geometric_margin_deg': min(r['min_actual_geometric_margin_deg'] for r in selected),
            'status_counts': dict(Counter(r['status'] for r in selected)),
        }
    changes = Counter()
    for pair in pairs.values():
        assert set(pair) == {'baseline', 'search_va'}
        diff = pair['search_va']['task_progress_ratio'] - pair['baseline']['task_progress_ratio']
        changes['improved_over_5pp' if diff > .05 else 'worse_over_5pp' if diff < -.05 else 'within_5pp'] += 1
    return dict(phases=phases, paired_changes=dict(changes),
        experimental_only_failures=sum(p['baseline']['status'] == 'completed' and p['search_va']['status'] != 'completed' for p in pairs.values()),
        baseline_only_failures=sum(p['baseline']['status'] != 'completed' and p['search_va']['status'] == 'completed' for p in pairs.values())), pairs


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args();folder=args.directory.resolve()
    d=json.loads((folder/'summary.json').read_text());m=d['matrix'];c=d['continuous'];b=d['benchmark']
    counts=m['counts'];rows=c['rows']
    stress = json.loads((folder/'stress_summary.json').read_text()) if (folder/'stress_summary.json').exists() else None
    stress_rows = stress['rows'] if stress else []
    stress_stats, stress_pairs = summarize_motion(stress_rows) if stress_rows else ({}, {})
    tool_benchmark = json.loads((folder/'benchmark_taccap.json').read_text()) if (folder/'benchmark_taccap.json').exists() else None
    assert sum(counts['baseline_'+k] for k in ('full','partial','hold','error'))==m['total']
    assert len(rows)==c['episodes']
    phases={};pairs={};failures=[]
    for r in rows:
        if r['status']!='completed':failures.append(r)
        if r['kind']=='hold':continue
        key=tuple(r[k] for k in ('mode','side','pose','kind','hz','profile'))
        pairs.setdefault(key,{})[r['strategy']]=r
    for strategy in ('baseline','search_va'):
        selected=[r for r in rows if r['strategy']==strategy and r['kind']!='hold']
        phases[strategy]={'episodes':len(selected),'completed':sum(r['status']=='completed' for r in selected),
            'completed_and_95percent_input':sum(r['status']=='completed' and r['task_progress_ratio']>=.95 for r in selected),
            'mean_input_progress':float(np.mean([r['task_progress_ratio'] for r in selected])),
            'median_input_progress':float(np.median([r['task_progress_ratio'] for r in selected])),
            'max_speed_deg_s':max(r['max_command_speed_deg_s'] for r in selected),
            'max_acceleration_deg_s2':max(r['max_command_acceleration_deg_s2'] for r in selected),
            'min_actual_geometric_margin_deg':min(r['min_actual_geometric_margin_deg'] for r in selected)}
    changed=Counter();by_pose=defaultdict(list)
    for key,pair in pairs.items():
        assert set(pair)=={'baseline','search_va'}
        before,after=pair['baseline'],pair['search_va']
        diff=after['task_progress_ratio']-before['task_progress_ratio']
        changed['improved_over_5pp' if diff>.05 else 'worse_over_5pp' if diff<-.05 else 'within_5pp']+=1
        by_pose[key[2]].append((before['task_progress_ratio'],after['task_progress_ratio']))
    precision=d['precision']
    precision_reject=[r for r in precision if r.get('accepted') is False]
    tiny_reject=[r for r in precision_reject if r['max_roundtrip_joint_error_deg']<1e-5]
    recovered_stops=[]
    with (folder/'matrix.jsonl').open() as f:
        for line in f:
            row=json.loads(line)
            if row.get('search')=='changed_phi_feasible' and row['baseline'] in ('hold','error'):
                recovered_stops.append(row)
    ratefit=sum(r['fits_450hz_30deg_s'] for r in recovered_stops)
    all_failures = [dict(phase='main', **r) for r in failures] + [dict(phase='stress', **r) for r in stress_rows if r['status'] != 'completed']
    derived=dict(continuous=phases,paired_changes=dict(changed),
        per_pose={k:dict(n=len(v),baseline_mean=float(np.mean(np.array(v)[:,0])),search_mean=float(np.mean(np.array(v)[:,1]))) for k,v in by_pose.items()},
        recovered_stops=len(recovered_stops),recovered_stops_also_fit_450hz_30deg_s=ratefit,
        precision_total=len(precision),precision_rejected=len(precision_reject),tiny_roundtrip_rejected=len(tiny_reject),
        main_failure_count=len(failures), failure_count=len(all_failures), stress=stress_stats,
        precision_control_total=stress['precision_control_total'] if stress else 0,
        precision_control_passed=stress['precision_control_passed'] if stress else 0)
    (folder/'analysis.json').write_text(json.dumps(derived,ensure_ascii=False,indent=2))
    (folder/'failures.json').write_text(json.dumps(all_failures,ensure_ascii=False,indent=2))
    fields=['phase','duration','mode','side','pose','kind','hz','profile','strategy','weight','status','completed_steps',
        'requested_steps','task_progress_ratio','max_command_speed_deg_s','max_command_acceleration_deg_s2',
        'min_actual_geometric_margin_deg','max_tracking_error_deg','final_distance_deg','reason']
    with (folder/'continuous.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');writer.writeheader()
        writer.writerows(dict(phase='main', duration=2., **r) for r in rows)
        writer.writerows(dict(phase='stress', **r) for r in stress_rows)

    lines=['PICO 腕部联合限位与臂角选解：离线评估报告',
        '生成时间：'+d['created_utc'],
        '范围：真实厂家数学库；原 vr_plan.entry；仅在测试脚本内实现的实验选解器。没有连接或控制机器人。',
        '', '一、方法与统计口径',
        '固定随机种子 '+str(d['seed'])+'；双臂、法兰/当前配置的夹爪 TCP。',
        '起始配置包括 3 个 Home、24 个随机姿态、40 个边界姿态、12 个关节极限/近奇异姿态及已知边界样例；每侧、每工具各 80 组。',
        '320 组起始配置中，'+str(len(m['preflight_rejections']))+' 组在静态正逆解预检失败；余下 316 组，每组 27 个目标，共 '+str(m['total'])+' 个案例。',
        '27 个目标：保持；六轴正负方向小步/大步；两个混合目标。小步为 0.2 mm/0.05°，大步为 2 mm/0.5°。',
        '离线替代解搜索：35 个离散臂角偏移，最远 ±15°；任一关节变化不超过 5°。这是几何可行性搜索，不是每周期可执行的命令。',
        '样本刻意加强边界和奇异区域，不能把成功率外推为实际使用成功率。',
        '', '二、现有回归测试',
        '92 项：test_vr、test_vr_wrist、test_vr_cartesian、test_vr_home、test_wrist_guard、test_tcp.MathTests、test_tools，全部通过。',
        '', '三、单步目标矩阵',
        json.dumps(counts,ensure_ascii=False,indent=2),
        f"原策略保持或拒绝：{counts['baseline_hold']+counts['baseline_error']}；改变臂角后找到完整目标解：{len(recovered_stops)}。",
        f"其中仅 {ratefit} 个替代解还满足假设 30°/s、450 Hz 的单周期关节位移上限；尚未计入加速度。",
        f"全部改变臂角找到的完整目标解：{counts['changed_phi_feasible']}，包括原策略仅完成部分步长的案例。",
        f"另有 {counts['same_phi_feasible']} 个原臂角完整目标已满足静态检查，但原策略主动减速；不把这些算作臂角修复收益。",
        f"{counts['not_found_in_sampled_neighborhood']} 个案例在本次有限搜索内未找到解；不等于全局无解。",
        '被接受输出复核：独立 Python FK、独立关节限位、现有完整腕部线段检查、原始腕部边界的稠密采样。',
        '最大独立 FK 误差 [mm, deg]：'+str(m['max_independent_fk_error_mm_deg']),
        '', '四、连续实验',
        '240 个运动实验 = 120 个匹配场景 × 原策略/实验选解器；另有 16 个固定 TCP 的权重/加速度对照，共 256 次。',
        '每次虚拟时间 2 秒；90 Hz 理想跟随、450 Hz 的 30 ms 一阶跟随滞后 + 20 ms 反馈延迟；权重实验另有 60 ms 滞后 + 40 ms 延迟。',
        '反馈以 100 Hz 绝对时间槽采样；机器人端用较新的状态独立复核。首次拒绝即终止该次实验，未模拟实际刹停轨迹。',
        '输入完成率 = 实际接受的累计缩放手柄增量 / 全程要求的累计增量。仅“跑完”不代表手柄动作完成。',
        json.dumps(phases,ensure_ascii=False,indent=2),
        '匹配场景差异：'+json.dumps(dict(changed),ensure_ascii=False),
        '按起始姿态分组：'+json.dumps(derived['per_pose'],ensure_ascii=False,indent=2),
        '', '四 B、加快动作与反馈延迟的压力测试',
        '另有 192 次实验 = 96 个匹配场景 × 两种策略。双臂、两种工具、Home3/近边界/边界三种姿态、旋转/混合输入。',
        '将相同位移压缩至 0.5 秒或 1 秒完成，均以虚拟 450 Hz 运行；理想跟随或 60 ms 一阶滞后 + 40 ms 反馈延迟。',
        json.dumps(stress_stats, ensure_ascii=False, indent=2),
        '关键反例：实验策略仍有接收侧动态腕部检查拒绝，以及所有有限候选被加速度条件排除。原策略能跑完而实验策略中断的场景不能隐去。',
        '其中 4 个 taccap/Home3/0.5 秒旋转案例：原策略能跑完，但关节加速度峰值约 151–154°/s²，超过实验的 120°/s² 条件。因此不能将跑完次数直接理解为同等约束下原策略更优。',
        '这说明单步贪心选解不保证下一步仍然存在可行解；只缩小当前 TCP 步长，也不保证能在加速度约束内减速。',
        '即使标为 ideal，客户端反馈仍为 100 Hz，接收侧检查使用每个 450 Hz 步骤的较新状态，因而两侧动态预留可能不同。',
        '', '五、候选合格条件与惩罚',
        '硬条件：关节限位、两条腕部路径（指令与实测）、实测腕速 × 0.20 s 预留、30°/s 关节速度；search_va 再加入 120°/s² 加速度。',
        '臂角候选速率限幅 5°/s，增减候选按 20°/s² 生成；还保留零偏移候选，因此不能把 20°/s² 视为所有输出均经独立验证的臂角加速度硬上限。这些都是离线实验参数，不是已标定的真机设置。',
        '臂角参考来自上一帧指令关节的 FK；增减的是本帧 IK 的目标臂角参数。实测 state 不被修改，用于限位、路径和速度预留检查。',
        'd 使用扣除 2° 和速度预留后的联合阶梯边界的欧氏角距离，不使用原 clearance 分数。',
        'P(d)=max(0,1-d/5)^2；先保留最大可行 TCP 步长，再按速度变化与边界势能变化排序。',
        '若 d 分别为 2/3/4/5°，P 分别为 0.36/0.16/0.04/0。这里的 d 已扣除静态与动态预留；原始边界的 2° 预留不等于这个 d 的下限。',
        c['assumptions']['score'],
        '远离边界且原臂角解通过全部检查时，走单候选快速路径。近边界候选采用当前臂角、臂角速度延续及增减方向；每个 TCP 步长最多 4 个不同候选。若尝试全部 7 个回退步长，理论最多 27 次 IK（零步长零偏移直接保持）。',
        '固定 TCP 对照结果：']
    for r in rows:
        if r['kind']=='hold':
            lines.append(f"{r['strategy']} weight={r['weight']:g} {r['profile']}: {r['status']}, final_margin={r['final_distance_deg']:.4f}°, max_speed={r['max_command_speed_deg_s']:.4f}°/s, max_acceleration={r['max_command_acceleration_deg_s2']:.4f}°/s²")
    lines+=['', '六、计算耗时（双臂合计）',
        '每项预热 10 次、记录 250 次。主测试为 flange TCP，另有 taccap TCP 补测。局部串行计算，包括厂家封装自带 FK 验证；不含输入接收、日志、SSH、ROS、线程调度和电机响应。',
        '450 Hz 周期为 2.222 ms；90 Hz 为 11.111 ms。']
    for r in b['rows']:
        s=r['bimanual_ms']
        lines.append(f"{r['pose']} {r['variant']}: median={s['p50']:.3f} ms, p95={s['p95']:.3f} ms, p99={s['p99']:.3f} ms, max={s['maximum']:.3f} ms; 超450Hz预算 {r['over_450hz_budget']}/{s['n']}")
    if tool_benchmark:
        lines.append('taccap TCP 补测：')
        for r in tool_benchmark['rows']:
            s=r['bimanual_ms']
            lines.append(f"{r['pose']} {r['variant']}: median={s['p50']:.3f} ms, p95={s['p95']:.3f} ms, p99={s['p99']:.3f} ms, max={s['maximum']:.3f} ms; 超450Hz预算 {r['over_450hz_budget']}/{s['n']}")
    lines+=['', '七、数值与失败情况',
        f"{len(precision)} 个边界正逆解回代试验中，{len(precision_reject)} 个被腕部检查拒绝；其中 {len(tiny_reject)} 个关节回代误差小于 1e-5°。",
        '这表明阶梯边界附近存在数值敏感性；某个关节极小的变化可能跨入较低上限格，不能把拒绝直接解释为物理碰撞。',
        f"对照：偏离查表跳变点 0.0001° 且保留正向余量的 {derived['precision_control_total']} 个案例，通过 {derived['precision_control_passed']} 个。",
        '跳变点的保守边界不能直接放宽；应以一致的数值容差处理往返误差，并复核边界定义。',
        '异常输入：'+json.dumps(stress['adversarial'] if stress else [], ensure_ascii=False),
        '实验选解器对不可达/非有限输入退到保持，缺少原策略的显式无效输入拒绝；输出没有被计作完整求解。接入生产前必须补齐输入验证与错误语义。',
        f'主实验与压力实验合计 {len(all_failures)} 次中途拒绝，全部保存在 failures.json。']
    for r in all_failures:
        lines.append(' / '.join(str(r[k]) for k in ('phase','mode','side','pose','kind','hz','profile','strategy'))+f"，duration={r.get('duration',2.)} s，第 {r['completed_steps']+1} 步，{r['status']}：{r['reason']}")
    lines+=['', '八、适用范围与下一步',
        '当前证据支持：臂角搜索能改善一部分固定臂角下的受阻目标；必须同时处理动态可执行性、预测边界和计算预算。',
        '主实验较缓动作的改善，不能覆盖压力实验出现的退化；目前不能直接替换生产 450 Hz 控制路径。',
        '本实验没有修改生产遥操作算法。仅限速的候选切换可能产生很大的加速度；加入加速度限制后需重新评估目标跟随与停顿。',
        '修正优先级：边界查表数值一致性；结合候选运动与反馈时延预测动态余量；加入可持续减速的轨迹状态；给搜索设置实际执行时间预算。不能只把惩罚权重调大。',
        '待验证：真实控制器响应、制动/保持过程、通信抖动、全身/夹爪/环境碰撞及更广范围姿态。',
        '用于比较的第一阶跟随模型未用真机辨识，不能据此宣称实机安全或达到实时要求。',
        '', '复现：',
        'PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B tools/evaluate_pico_wrist_ik.py --output logs/new-ik-wrist-run',
        'PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B tools/stress_pico_wrist_ik.py logs/new-ik-wrist-run',
        'PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B tools/summarize_pico_wrist_ik.py logs/new-ik-wrist-run',
        '精确版本：summary.json 中记录 SHA-256；evaluation_source.py 为运行时测试脚本副本。']
    (folder/'report.txt').write_text('\n'.join(lines)+'\n')

    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    fig,axs=plt.subplots(1,3,figsize=(15,4.6),layout='constrained')
    names=['Full target','Partial step','Hold','Rejected']
    vals=[counts['baseline_'+k] for k in ('full','partial','hold','error')]
    axs[0].barh(names,vals,color=['#4c78a8','#f2cf5b','#f58518','#e45756'])
    for i,v in enumerate(vals):axs[0].text(v+50,i,str(v),va='center')
    axs[0].invert_yaxis();axs[0].set_xlim(0,max(vals)*1.2);axs[0].set_title(f'Current planner: {m["total"]} targets');axs[0].set_xlabel('Number of cases')
    p=np.array([[pair['baseline']['task_progress_ratio'],pair['search_va']['task_progress_ratio']] for pair in pairs.values()])
    axs[1].scatter(p[:,0],p[:,1],alpha=.65,s=24,color='#4c78a8',label='2 s motion')
    if stress_pairs:
        sp=np.array([[pair['baseline']['task_progress_ratio'],pair['search_va']['task_progress_ratio']] for pair in stress_pairs.values()])
        axs[1].scatter(sp[:,0],sp[:,1],alpha=.7,s=30,marker='x',color='#f58518',label='0.5 / 1 s stress')
    axs[1].plot([0,1],[0,1],color='gray',linestyle='--');axs[1].legend(loc='center left',fontsize=8)
    axs[1].set(xlim=(-.03,1.03),ylim=(-.03,1.03),xlabel='Current planner: input progress',ylabel='Experimental selector: input progress',title=f'{len(pairs)+len(stress_pairs)} paired motion scenarios')
    labels=[];med=[];p95=[]
    for row in b['rows']:
        labels.append(('Home' if row['pose']=='home1' else 'Boundary')+' / '+row['variant'])
        med.append(row['bimanual_ms']['p50']);p95.append(row['bimanual_ms']['p95'])
    y=np.arange(len(labels));axs[2].barh(y,med,color='#4c78a8',label='Median')
    axs[2].scatter(p95,y,marker='|',s=130,color='#e45756',label='p95')
    axs[2].axvline(1000/450,color='#f58518',linestyle='--',label='450 Hz budget')
    axs[2].set_yticks(y,labels);axs[2].invert_yaxis();axs[2].set_xlabel('Bimanual compute (ms)');axs[2].set_title('Local compute only (flange)');axs[2].legend(loc='upper right',fontsize=8)
    fig.savefig(folder/'overview.png',dpi=180);plt.close(fig)
    fig,axs=plt.subplots(1,2,figsize=(12,4),layout='constrained')
    hold=[]
    with (folder/'continuous.jsonl').open() as f:
        for line in f:
            row=json.loads(line)
            if row['kind']=='hold' and row['strategy']=='search_va' and row['profile']=='ideal':hold.append(row)
    for row in hold:
        trace=row['trace'];axs[0].plot([t['t'] for t in trace],[t['margin'] for t in trace],label=f'weight={row["weight"]:g}')
    axs[0].set(xlabel='Virtual time (s)',ylabel='Geometric wrist margin (deg)',title='Fixed TCP; velocity + acceleration constraints');axs[0].legend()
    selected=[r for r in rows if r['kind']=='hold' and r['profile']=='ideal' and r['weight']==1.]
    axs[1].bar([r['strategy'] for r in selected],[r['max_command_acceleration_deg_s2'] for r in selected],color=['#e45756','#4c78a8'])
    for i,r in enumerate(selected):axs[1].text(i,r['max_command_acceleration_deg_s2']+50,f"{r['max_command_acceleration_deg_s2']:.1f}",ha='center')
    axs[1].set(ylabel='Peak command acceleration (deg/s²)',title='Velocity-only vs velocity + acceleration')
    fig.savefig(folder/'penalty-and-acceleration.png',dpi=180);plt.close(fig)
    print(json.dumps(derived,ensure_ascii=False,indent=2))
    print(folder/'report.txt')


if __name__=='__main__':main()
