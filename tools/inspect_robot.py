"""Passive audit: read files, ROS states, controller list, scans. Never enable/reset."""
import argparse,base64,json,shlex,subprocess
from pathlib import Path
from datetime import datetime
from ssh_read import run
ROOT=Path(__file__).resolve().parents[1]

def setup(host):
    prefix='source /opt/ros/humble/setup.bash\n'
    if host.endswith('.1'):
        prefix+='source /opt/fairino_art_plugin/plugin/install/setup.bash\nexport ROS_DOMAIN_ID=0\n'
    else:
        prefix+='source /opt/fairino_AMR_project_B/reference_chassis_runtime/install/hardware_state_broadcaster_msgs/share/hardware_state_broadcaster_msgs/local_setup.bash\nexport ROS_DOMAIN_ID=1\n'
    return prefix+'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0\nunset FASTRTPS_DEFAULT_PROFILES_FILE\n'

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--part',choices=['all','upper','base'],default='all');p.add_argument('--user',default='fairino');a=p.parse_args()
    hosts={'all':['192.168.58.1','192.168.58.3'],'upper':['192.168.58.1'],'base':['192.168.58.3']}[a.part]
    source=(ROOT/'tools/ros_observer.py').read_text()
    log=ROOT/'logs'/datetime.now().strftime('%Y%m%d-%H%M%S-status');log.mkdir(parents=True,exist_ok=True)
    failed=False
    for host in hosts:
        try:
            out=run(host,setup(host)+'python3 -u -c '+shlex.quote(source),user=a.user,timeout=40)
            line=next(x for x in out.splitlines() if x.startswith('OBSERVE_JSON '))
            report=json.loads(line[13:]);(log/(host+'.json')).write_text(json.dumps(report,ensure_ascii=False,indent=2))
            hw=report.get('hardware')
            if not hw:raise RuntimeError('未收到硬件反馈')
            print(host,'axes',len(hw['joint_names']),'enabled',sum((int(x) & 0x006F) == 0x0027 for x in hw['stateword']),
                'global',hex(int(hw['hexerrorcode_curerrcode'])),'faults',dict((j,e) for j,e in zip(hw['joint_names'],hw['errorcode']) if e))
            if host.endswith('.1'):print('GPIO',report.get('gpio'))
            else:
                print('upper relay',report.get('upper_estop'))
                for topic,s in report['scans'].items():print(topic,round(s['hz'],1),'Hz',s['points'],'points')
                if '/scan' not in report['scans']:print('/scan 未观察到消息；不代表导航已可用')
        except Exception as e:failed=True;print(host,'检查失败:',e)
    print('只读记录:',log)
    return int(failed)
if __name__=='__main__':raise SystemExit(main())
