"""Upper-body fault reset only. Requires all 20 axes disabled, stationary, E-stop released."""
import argparse,json,shlex
from pathlib import Path
from datetime import datetime
from ssh_read import run
ROOT=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser(description='单次上半身 reseterror()；不会使能。先查明故障原因。默认只显示用法。')
    p.add_argument('--execute-reset',action='store_true');p.add_argument('--user',default='fairino');a=p.parse_args()
    if not a.execute_reset:p.print_help();return 0
    code=(ROOT/'tools/upper_reset_remote.py').read_text()
    setup='set -e\nsource /opt/ros/humble/setup.bash\nsource /opt/fairino_art_plugin/plugin/install/setup.bash\nexport ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp\nexport FASTRTPS_DEFAULT_PROFILES_FILE=/home/fairino/fastrtps_udp.xml\n'
    output=run('192.168.58.1',setup+'python3 -u -c '+shlex.quote(code),user=a.user,timeout=45)
    for line in output.splitlines():
        if line.startswith('RESET_RESULT '):
            report=json.loads(line[13:]);log=ROOT/'logs';log.mkdir(exist_ok=True)
            path=log/(datetime.now().strftime('%Y%m%d-%H%M%S')+'-reset-upper.json')
            path.write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False,indent=2));print(path)
            return 0 if report.get('cleared_and_disabled') else 1
    print(output);return 1
if __name__=='__main__':raise SystemExit(main())
