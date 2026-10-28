"""Ship this exact source tree into a temporary directory on the ROS host."""
import base64
import hashlib
import json
from pathlib import Path
import shlex
import zlib

ROOT = Path(__file__).resolve().parents[3]


def source_bundle():
    groups = {
        'apps/fairino_upper_probe': ('wrist_guard.py', 'probe_core.py', 'robot_probe.py'),
        'apps/fairino_upper_probe/body_control': ('body_core.py',),
        'apps/fairino_tcp_probe': ('tcp_core.py', 'tcp_frames.py', 'tcp_jog.py',
                                   'tcp_robot.py', 'tool_frames.py'),
        'scripts/pico4/fairino': ('vr_core.py', 'vr_robot.py', 'vr_lift.py', 'vr_lift_presets.py', 'vr_timing.py'),
        'tools/ik_experiments': ('__init__.py', 'art7_qp.py', 'art7_native.py', 'art7_native.cpp'),
        'scripts/pico4/native_ik_test': ('__init__.py', 'protocol.py', 'planner.py', 'server.py'),
    }
    return {f'{directory}/{name}': (ROOT/directory/name).read_text()
            for directory, names in groups.items() for name in names}


def remote_command(args):
    bundle = source_bundle()
    payload = json.dumps(bundle, sort_keys=True).encode()
    digest = hashlib.sha256(payload).hexdigest()
    encoded = base64.b64encode(zlib.compress(payload)).decode()
    argv = ['server.py', '--tools-json', json.dumps(args.tools, allow_nan=False)]
    for name in ('command_hz', 'feedback_hz', 'feedback_age_ms', 'input_timeout_ms'):
        argv += ['--'+name.replace('_', '-'), str(getattr(args, name))]
    if not args.execute:
        argv += ['--check']
    else:
        argv += ['--execute']
    # No persistent installation, shell interpolation of source, or prebuilt
    # architecture-specific .so. The ROS host builds its own content-hash cache.
    loader = f'''import base64,json,pathlib,runpy,sys,tempfile,zlib
bundle=json.loads(zlib.decompress(base64.b64decode({encoded!r})))
with tempfile.TemporaryDirectory(prefix='art7-pico-native-') as temporary:
 root=pathlib.Path(temporary)
 for relative,source in bundle.items():
  path=root/relative
  path.parent.mkdir(parents=True,exist_ok=True)
  path.write_text(source)
 for relative in ('','scripts/pico4/fairino','apps/fairino_tcp_probe','apps/fairino_upper_probe','apps/fairino_upper_probe/body_control'):
  sys.path.insert(0,str(root/relative))
 print('@UPPERPROBE '+json.dumps({{'source_sha256':{digest!r},'event':'native_source_bundle'}}),flush=True)
 sys.argv={argv!r}
 runpy.run_path(str(root/'scripts/pico4/native_ik_test/server.py'),run_name='__main__')
'''
    setup = '\n'.join([
        'set -e', 'source /opt/ros/humble/setup.bash',
        'source /opt/fairino_art_plugin/plugin/install/setup.bash',
        'export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp',
        'export FASTRTPS_DEFAULT_PROFILES_FILE=/home/fairino/fastrtps_udp.xml',
        'export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1',
        'echo UPPERPROBE_CONNECTED', 'exec python3 -u -c '+shlex.quote(loader),
    ])
    command = 'bash -c '+shlex.quote(setup)
    if len(command.encode()) >= 120000:
        raise ValueError('远程源码包超过 SSH 单参数限制')
    return command
