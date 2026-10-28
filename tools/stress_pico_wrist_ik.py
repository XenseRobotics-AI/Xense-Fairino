#!/usr/bin/env python3
"""Extra fast-motion / feedback-delay / precision controls, entirely offline."""
import argparse
from collections import Counter
import json
from pathlib import Path

from evaluate_pico_wrist_ik import (VendorKinematics, tool_config, run_episode,
    MIRROR, GUARD, cap, np)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args();out=args.directory.resolve();out.mkdir(parents=True,exist_ok=True)
    (out/'stress_source.py').write_text(Path(__file__).read_text())
    kins={mode:VendorKinematics(tools=tool_config(mode)) for mode in ('flange','taccap')}
    precision=[]
    for mode,kin in kins.items():
        for side in ('left','right'):
            for sixth in (0.,2.,4.,6.,10.,30.,50.,52.,60.,68.):
                # Move 0.0001 deg off the lookup-table breakpoint; keep a positive cap slack.
                sixth=sixth-.0001 if sixth else .0001
                for slack in (.05,.5):
                    q=np.array([45.,75.,-100.,85.,135.,sixth,cap(sixth+2)-2-slack])
                    if side=='right':q*=MIRROR
                    q=np.radians(q).tolist();tcp,phi=kin.forward_tcp(q,side)
                    result=kin.inverse_tcp(tcp,phi,q,side)
                    try:GUARD.arm_segment(q,result,[0.]*7);passed=True
                    except ValueError:passed=False
                    precision.append(dict(mode=mode,side=side,sixth=sixth,slack=slack,passed=passed))
    (out/'precision_controls.json').write_text(json.dumps(precision,indent=2))
    rows=[];total=2*2*3*2*2*2*2
    with (out/'stress.jsonl').open('w') as f:
        for mode,kin in kins.items():
            for side in ('left','right'):
                for pose in ('home3','near_boundary','known_boundary'):
                    for kind in ('rotate','mixed'):
                        for duration in (.5,1.):
                            for profile in ('ideal','lag60_delay40'):
                                for strategy in ('baseline','search_va'):
                                    row=run_episode(kin,mode,side,pose,kind,450.,profile,strategy,duration=duration)
                                    row['duration']=duration
                                    f.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n');f.flush()
                                    rows.append({k:v for k,v in row.items() if k!='trace'})
                                    if len(rows)%8==0:print(f'stress {len(rows)}/{total}: {row["status"]}',flush=True)
    # Adversarial inputs that must not be called successfully solved commands.
    from evaluate_pico_wrist_ik import named_pose, hw, entry, ExperimentalPlanner
    refusal=[];kin=kins['flange'];q=named_pose('home1','left');tcp=np.array(kin.forward_tcp(q.tolist(),'left')[0])
    for name,goal in [('far_unreachable',tcp.copy()),('nonfinite',tcp.copy())]:
        goal[0,3]=5000. if name=='far_unreachable' else float('nan')
        for strategy in ('baseline','search_va'):
            try:
                if strategy=='baseline':
                    r=entry(kin,hw(q,'left'),'left',goal,reference=q.tolist());fraction=r['wrist_scale']
                else:
                    r=ExperimentalPlanner(kin,'left').choose(q,q,np.zeros(7),goal);fraction=r['fraction']
                refusal.append(dict(case=name,strategy=strategy,result='partial_or_hold' if fraction<1 else 'full',fraction=fraction))
            except ValueError as exc:refusal.append(dict(case=name,strategy=strategy,result='rejected',reason=str(exc)))
    result=dict(episodes=len(rows),rows=rows,counts=dict(Counter(r['strategy']+'/'+r['status'] for r in rows)),
                precision_control_total=len(precision),precision_control_passed=sum(r['passed'] for r in precision),
                adversarial=refusal)
    (out/'stress_summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))
    print(json.dumps({k:v for k,v in result.items() if k!='rows'},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
