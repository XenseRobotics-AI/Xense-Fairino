#!/usr/bin/env python3
"""Create a compact auditable report from the native IK offline evaluation."""
import argparse
import json
from pathlib import Path
import re


def aggregate(rows):
    return dict(episodes=len(rows),completed=sum(r['status']=='completed' for r in rows),
        maximum_command_error_mm_deg=[max(r['max_command_error_mm_deg'][i] for r in rows) for i in (0,1)],
        maximum_actual_error_mm_deg=[max(r['max_actual_error_mm_deg'][i] for r in rows) for i in (0,1)],
        minimum_excursion_ratio=[min(r['excursion_ratio'][i] for r in rows if r['excursion_ratio'][i] is not None) for i in (0,1)],
        peak_command_speed_deg_s=max(r['peak_command_velocity_deg_s'] for r in rows),
        peak_command_acceleration_deg_s2=max(r['peak_command_acceleration_deg_s2'] for r in rows),
        final_command_speed_deg_s=max(max(map(abs,r['final_command_velocity_deg_s'])) for r in rows),
        final_actual_speed_deg_s=max(max(map(abs,r['final_actual_velocity_deg_s'])) for r in rows))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    out=parser.parse_args().directory.resolve();s=json.loads((out/'summary.json').read_text())
    phases={phase:aggregate([r for r in s[phase]['rows'] if r['predictive']]) for phase in ('continuous','stress','release')}
    baseline=aggregate([r for r in s['continuous']['rows'] if not r['predictive']])
    stop_times=[r['command_stop_time_s'] for r in s['release']['rows'] if r['command_stop_time_s'] is not None]
    bench=[r for r in s['benchmark']['rows'] if r['variant']=='predictive']
    timing=dict(samples=sum(r['bimanual_ms']['n'] for r in bench),
        minimum_group_median_ms=min(r['bimanual_ms']['p50'] for r in bench),
        maximum_group_median_ms=max(r['bimanual_ms']['p50'] for r in bench),
        maximum_group_p95_ms=max(r['bimanual_ms']['p95'] for r in bench),
        maximum_group_p99_ms=max(r['bimanual_ms']['p99'] for r in bench),
        maximum_ms=max(r['bimanual_ms']['maximum'] for r in bench),
        samples_over_450hz=sum(r['over_450hz_budget'] for r in bench))
    timing['budget_ms']=1000/450
    timing['headroom_at_worst_group_p99_ms']=timing['budget_ms']-timing['maximum_group_p99_ms']
    analysis=dict(phases=phases,baseline_continuous=baseline,predictive_bimanual_timing=timing,
        maximum_command_stop_seconds=max(stop_times) if stop_times else None,
        all_release_commands_stopped=len(stop_times)==s['release']['episodes'],
        maximum_final_braking_displacement_mm_deg=[max(r['final_error_mm_deg'][i] for r in s['release']['rows']) for i in (0,1)],
        maximum_final_actual_error_to_stop_point_mm_deg=[max(r['final_actual_error_to_command_mm_deg'][i] for r in s['release']['rows']) for i in (0,1)])
    (out/'analysis.json').write_text(json.dumps(analysis,ensure_ascii=False,indent=2))
    unit_log=(out/'unit-tests.log').read_text() if (out/'unit-tests.log').exists() else ''
    unit_count=re.search(r'Ran (\d+) tests',unit_log)
    lines=['ART7 C++约束IK、动态余量和停止过程：最终离线验收',
        '本报告对应summary.json的源码SHA-256。生产遥操作、现有WristGuard与独立FK未修改；未连接或控制机器人。',
        '', '实现与接口',
        'art7_native.cpp：标准C++17实现FK、解析Jacobian、SO(3)、有界QP、腕部矩形并集、迭代与双路径检查。',
        'art7_native.py：ctypes接口，自动以g++ -O3构建内容寻址缓存；输入检查与原Python guard独立复核仍保留。',
        'dynamic_wrist_policy.py：PredictiveWristIK。step生成候选，命令被接受后commit(result)；brake生成单调减速步，停止后保持最终停点。',
        '所有公开关节角/速度为rad/rad每秒，TCP平移为mm。success表示TCP误差达标，feasible只表示候选符合约束，stopped只表示命令速度归零。',
        '', '联合限位已经参与求解',
        '静态求解使用原阶梯边界的矩形并集，越界普通候选会在允许区域内重新优化七个关节，不锁定臂角。',
        '周期求解同时约束关节位置、整周期速度与加速度；内部迭代不会重新获得一个周期的位移额度。',
        '预测策略按参考、实测、近期已发指令的最坏位置计算矩形剩余距离g，取实测与近期命令速度上界u。',
        '候选速度上限w满足：T*max(u,w)+(feedback_age+dt)*w+w²/(2*a)<=g。T=原guard的0.20秒。',
        '等价地，分别求两个二次不等式的正根并取小值：原速度的预留不能被更慢的新命令替代，剩余空间仍需容纳延迟和减速。',
        'a不超过本周期命令加速度上限。预算给出的是局部可执行条件，没有扩大原始保护边界，也没有启用自动回撤例外。',
        '零余量轴允许速度为零，其余关节仍可补偿TCP；矩形随状态改变，不是永久使用一个全局缩小范围。',
        '矩形计算已向量化；与旧循环1500个随机状态逐项对照最大差0。',
        '', '测试覆盖与统计口径',
        ('统一数学/接口/动态/制动测试：'+unit_count.group(1)+'项，全部通过。') if unit_count and '\nOK\n' in unit_log else '单元测试日志见unit-tests.log。',
        '独立模型检查：'+json.dumps(s['core']['model'],ensure_ascii=False),
        '静态目标矩阵：'+json.dumps(s['core']['static']['counts'],ensure_ascii=False),
        '静态误差验收标准为0.02mm和0.01°。884个小幅目标覆盖双臂、法兰/夹爪TCP、Home、随机和腕部边界姿态；不是全工作空间成功率。',
        '8个关键见证覆盖±40°自定义关节范围、原联合边界，均达到完整TCP目标容差。',
        '', '连续与压力场景',
        '基础对照：32个场景分别运行普通native和预测策略，共64次；每次1秒，虚拟450Hz、反馈100Hz。',
        '基础动作峰值为[0.5,-0.25,0.125]mm和[0.175,-0.1,0.075]°；理想跟随或30ms一阶滞后+20ms反馈延迟。',
        '压力测试：192次，左右臂、两种TCP、Home3/近边界/边界、旋转/混合动作、0.5/1秒、基础幅度与4倍幅度、理想或60ms滞后+40ms反馈延迟。',
        '完成表示全程未被规划或接收端拒绝；不等于每帧都精确跟随。另计实际运动幅度与TCP误差，不能用全程保持冒充成功。',
        '普通native基础结果：'+json.dumps(baseline,ensure_ascii=False),
        '预测策略分组结果：'+json.dumps(phases,ensure_ascii=False,indent=2),
        '快速压力动作会因速度/加速度与余量约束而出现滞后和残差。最大实际误差必须与完成数一并阅读。',
        '', '专用制动',
        '48个停止场景在动作进行到四分之一周期时发出停止请求，然后继续0.5秒跟随模拟。',
        '每周期v_next=sign(v_previous)*max(abs(v_previous)-a_max*dt,0)，q_next=q_reference+dt*v_next。以相同预测bounds及原双路径guard验证后，强制等式位置目标执行。',
        '这避免继续追踪松手瞬间的TCP造成反复修正。每轴命令速度不反向、绝对值不增，停下后保持最终停止位置。',
        'release组的error字段以松手瞬间TCP为参考，包含必要的减速位移；不能解释为最终停点的跟随误差。',
        '最大最终制动位移[mm,deg]：'+str(analysis['maximum_final_braking_displacement_mm_deg'])+'；尾段结束实测相对命令停点最大误差[mm,deg]：'+str(analysis['maximum_final_actual_error_to_stop_point_mm_deg']),
        '全部命令停止：'+str(analysis['all_release_commands_stopped'])+'；最长命令停止时间：'+str(analysis['maximum_command_stop_seconds'])+'秒。',
        '模拟尾段结束时最大实测关节速度：'+str(phases['release']['final_actual_speed_deg_s'])+'°/s；这不是实际电机停止时间验证。',
        '', '双臂完整调用耗时',
        '每项预热20次、记录500次；独占计时，未与其他测试并发。包含Python输入/输出检查、C++求解及预测策略；预测计时还包含reset/commit。',
        '目标0.05mm/0.02°高于收敛阈值，最多4次内部迭代。构造/首次编译、通信、调度和电机响应不计入。',
        json.dumps(timing,ensure_ascii=False,indent=2)]
    for r in s['benchmark']['rows']:
        t=r['bimanual_ms'];lines.append(f"{r['mode']}/{r['pose']}/{r['variant']}: p50={t['p50']:.4f}ms, p95={t['p95']:.4f}ms, p99={t['p99']:.4f}ms, max={t['maximum']:.4f}ms, 超450Hz预算={r['over_450hz_budget']}/{t['n']}")
    lines += ['', '条件与未覆盖部分',
        '离线跟随模型为正的一阶、无过冲响应；真实速度和位置须受最后反馈及之后已发送命令的凸包约束，反馈年龄须不超过配置上限（本次40或60ms）。',
        '30°/s、120°/s²是本轮实验默认值。可用制动能力、真实跟随误差、反馈时延和过冲需实机辨识；没有据此宣称真机安全认证或硬实时保证。',
        '保留原腕部模型范围，没有新增全身、夹爪、环境碰撞检查。生产路径未切换到新后端。',
        '', '复现',
        'PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m unittest discover -s tools/ik_experiments -p "test_*.py" -v',
        'PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B tools/evaluate_art7_native.py --output logs/new-native-run --sections core,continuous,stress,release',
        '待其余测试停止后：PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B tools/evaluate_art7_native.py --output logs/new-native-run --resume --sections benchmark',
        'PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B tools/summarize_art7_native.py logs/new-native-run']
    (out/'report.txt').write_text('\n'.join(lines)+'\n')
    print(json.dumps(analysis,ensure_ascii=False,indent=2))
    print(out/'report.txt')


if __name__=='__main__':main()
