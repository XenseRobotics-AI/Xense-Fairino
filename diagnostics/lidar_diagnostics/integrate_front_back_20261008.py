"""Apply confirmed front/back addresses and reload only the lidar launchers."""
from pathlib import Path
import datetime
import json
import os
import shutil
import signal
import subprocess
import time

ROOT=Path('/opt/fairino_AMR_project_B')
CONFIG=ROOT/'AMR_SAFT/install/amr_lidar_bringup/share/amr_lidar_bringup/config'
STAMP=datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
BACKUP=ROOT/'config/backups'/('lidar-'+STAMP)
REPORT={'backup':str(BACKUP),'changes':[],'stopped':[]}

def command(args):
    r=subprocess.run(args,capture_output=True,text=True,timeout=20)
    if r.returncode:raise RuntimeError(str(args)+': '+r.stdout+r.stderr)
    return r.stdout.strip()

def processes():
    result={}
    for p in Path('/proc').iterdir():
        if not p.name.isdecimal():continue
        try:args=p.joinpath('cmdline').read_bytes().decode().strip('\0').split('\0')
        except (OSError,UnicodeDecodeError):continue
        result[int(p.name)]=args
    return result

assert command(['systemctl','is-active','amr_startup.service'])=='active'
main_pid=int(command(['systemctl','show','amr_startup.service','-p','MainPID','--value']))
procs=processes()
protected=[pid for pid,args in procs.items() if args and (
    args[0].endswith('/HAL_RK3588_Debug_07_030')
    or args[0].endswith('/fairino_control_node')
    or args[0].endswith('/amr_server_node'))]
REPORT['amr_main_pid']=main_pid
REPORT['protected_pids']=protected
updates=[('front','192.168.58.11','192.168.192.101','20260907-000045'),
         ('back','192.168.58.10','192.168.192.100','20260907-000046')]
for position,old,new,serial in updates:
    p=CONFIG/('lidar_'+position+'_keli.yaml')
    text=p.read_text()
    assert text.count('hostname: "'+old+'"')==1,(str(p),'unexpected address')
BACKUP.mkdir(parents=True,exist_ok=False)
for position,old,new,serial in updates:
    p=CONFIG/('lidar_'+position+'_keli.yaml')
    shutil.copy2(p,BACKUP/p.name)
    p.write_text(p.read_text().replace('hostname: "'+old+'"','hostname: "'+new+'"'))
    REPORT['changes'].append({'file':str(p),'position':position,'serial':serial,'old':old,'new':new})

# Stop only the older viewer's acquisition worker so it releases both UDP sessions.
workers=[pid for pid,args in procs.items() if len(args)>=4 and args[:3]==['python3','-u','-c']
         and 'Run only the two LS3 sensor drivers in an isolated local ROS domain.' in args[3]]
for pid in workers:
    os.kill(pid,signal.SIGTERM);REPORT['stopped'].append({'viewer_worker':pid})
for _ in range(80):
    current=processes()
    viewer_drivers=[pid for pid,args in current.items() if args and args[0].endswith('/sdkeli_ls3')
                    and any(a.startswith('__node:=ls3_viewer_') for a in args)]
    if not any(pid in current for pid in workers) and not viewer_drivers:break
    time.sleep(.1)
else:raise RuntimeError('Viewer still holds lidar sessions; did not launch another driver')

# Each original launcher has its own session. The AMR parent and control processes stay alive.
launchers=[]
for pid,args in procs.items():
    if len(args)>=6 and args[1:5]==['/opt/ros/humble/bin/ros2','launch','amr_lidar_bringup','lidars.launch.py']:
        assert args[5] in ['lidar_instance:=front','lidar_instance:=back']
        assert os.getpgid(pid)==pid
        launchers.append(pid)
assert len(launchers)==2,launchers
for pid in launchers:
    os.killpg(pid,signal.SIGINT);REPORT['stopped'].append({'original_lidar_launcher':pid})
for _ in range(100):
    current=processes()
    drivers=[pid for pid,args in current.items() if args and args[0].endswith('/sdkeli_ls3')]
    if not any(pid in current for pid in launchers) and not drivers:break
    time.sleep(.1)
else:raise RuntimeError('Old lidar driver still active; refused duplicate launch')

# This transient unit covers this boot only. It stops with AMR. The normal startup
# script will load the saved YAML files on the next AMR start or robot boot.
cmd='source /opt/ros/humble/setup.bash && source /opt/fairino_AMR_project_B/AMR_SAFT/install/setup.bash && exec env ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fairino_AMR_project_B/config/fastrtps_udp.xml ros2 launch amr_lidar_bringup lidars.launch.py'
REPORT['unit_start']=command(['systemd-run','--unit=fairino-lidar-reload','--collect',
    '--property=User=fairino','--property=Group=fairino',
    '--property=BindsTo=amr_startup.service','--property=After=amr_startup.service',
    '--property=KillSignal=SIGINT','--property=TimeoutStopSec=15','--property=Restart=no',
    '/bin/bash','-c',cmd])
assert int(command(['systemctl','show','amr_startup.service','-p','MainPID','--value']))==main_pid
for pid in protected:assert Path('/proc',str(pid)).exists(),('control process exited',pid)
REPORT['amr_control_unchanged']=True
print(json.dumps(REPORT,ensure_ascii=False))
