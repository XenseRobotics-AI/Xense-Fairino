"""Describe the actual route to each robot, without changing networking."""
import json
from pathlib import Path
import subprocess


def route_info(host, runner=subprocess.run, sysfs=Path('/sys/class/net')):
    result = {'host': host, 'kind': '未知', 'interface': '', 'source': '', 'gateway': ''}
    try:
        p = runner(['ip', '-j', 'route', 'get', host], capture_output=True, text=True, timeout=3)
        if p.returncode:
            raise ValueError(p.stderr.strip() or '无路由')
        route = json.loads(p.stdout)[0]
        device = route.get('dev', '')
        result.update(interface=device, source=route.get('prefsrc', route.get('src', '')),
                      gateway=route.get('gateway', ''))
        path = sysfs / device
        if (path / 'wireless').exists() or (path / 'phy80211').exists():
            result['kind'] = '无线'
        elif device == 'lo':
            result['kind'] = '本机'
        elif (path / 'device').exists() and (path / 'type').read_text().strip() == '1':
            result['kind'] = '有线'
        else:
            result['kind'] = '虚拟/其他'
    except (OSError, ValueError, IndexError, subprocess.SubprocessError) as exc:
        result['error'] = str(exc)
    return result


def describe(route):
    if route.get('error'):
        return f"{route['host']} · 路由不可用"
    return f"{route['kind']} · {route['interface']} · 本机 {route['source']} → {route['host']}"


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', action='append', help='目标 IP；默认检查底盘和上身路由')
    args = parser.parse_args()
    routes = [route_info(host) for host in (args.host or ['192.168.58.3', '192.168.58.1'])]
    print(json.dumps(routes, ensure_ascii=False, indent=2))
    return int(any(route.get('error') for route in routes))


if __name__ == '__main__':
    raise SystemExit(main())
