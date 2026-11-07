#!/usr/bin/env python3
"""Desktop map viewer; all robot access is through a read-only SSH observer."""
import argparse
import base64
import datetime
import getpass
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import sys
import threading
import time
import zlib

import numpy as np
import pexpect

HERE = Path(__file__).resolve().parent
from fairino_slam.paths import LOG_ROOT
from .client import execute as map_action

PREFIX = 'MAP_VIEWER '


class Stream:
    def __init__(self, args, password):
        self.args = args
        self.password = password
        self.control_password = password
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.latest = {}
        self.thread = threading.Thread(target=self.run, daemon=True)

    def snapshot(self):
        with self.lock:
            return dict(self.latest)

    def put(self, item):
        with self.lock:
            if item['type'] == 'clear_map':
                self.latest.pop('map', None)
            self.latest[item['type']] = item
            if item['type'] == 'frame':
                self.latest['received'] = time.monotonic()

    def run(self):
        child = None
        try:
            script = (HERE / 'remote_stream.py').read_text()
            command = (
                'source /opt/ros/humble/setup.bash && '
                'export ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp; '
                'export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/fairino_AMR_project_B/config/fastrtps_udp.xml; '
                'exec python3 -u -c ' + shlex.quote(script))
            if self.args.check:
                command += ' --duration ' + str(self.args.duration)
            child = pexpect.spawn('ssh', [
                '-tt', '-o', 'StrictHostKeyChecking=accept-new', '-o', 'ConnectTimeout=8',
                '-o', 'ServerAliveInterval=3', '-o', 'ServerAliveCountMax=2',
                self.args.user + '@' + self.args.host, 'bash -c ' + shlex.quote(command)],
                encoding='utf-8', timeout=1, maxread=1048576, echo=False)
            asked = False
            tail = ''
            connected = time.monotonic()
            while not self.stop.is_set():
                event = child.expect([r'(?i)password:', r'\r?\n', pexpect.EOF, pexpect.TIMEOUT])
                if event == 0:
                    if asked:
                        raise RuntimeError('SSH 密码验证失败')
                    child.sendline(self.password)
                    self.password = None
                    asked = True
                elif event == 1:
                    line = child.before.strip()
                    if line.startswith(PREFIX):
                        self.put(json.loads(line[len(PREFIX):]))
                    elif line:
                        tail = (tail + '\n' + line)[-1000:]
                elif event == 2:
                    child.close()
                    if child.exitstatus != 0 or not self.args.check:
                        raise RuntimeError('SSH 数据连接已结束：' + tail[-500:])
                    break
                if time.monotonic() - connected > 20 and 'frame' not in self.snapshot():
                    raise TimeoutError('连接后未收到地图观察进程的数据：' + tail[-500:])
        except Exception as exc:
            self.put({'type': 'error', 'message': str(exc)})
        finally:
            self.password = None
            if child is not None and child.isalive():
                child.sendcontrol('c')
                try:
                    child.expect(pexpect.EOF, timeout=3)
                except (pexpect.TIMEOUT, pexpect.EOF):
                    child.close(force=True)


def matching_health(frame, lag=0):
    match = frame.get('scan_match') or {}
    if not match.get('source_confirmed'):
        return '等待 SLAM 的雷达匹配结果；TF 正在跟随里程计', '#8b5d0e'
    moved = (match.get('travel_since_match', 0) >= match.get('distance_threshold', .1)
             or match.get('rotation_since_match', 0) >= match.get('angle_threshold', math.radians(5)))
    age = match.get('age')
    if age is None:
        if moved and match.get('waiting_age', 0)+max(0, lag) > 3:
            return '运动后仍未收到首帧雷达匹配；当前 TF 随轮速推算', '#b53a36'
        return '尚未收到首帧雷达匹配；TF 正在跟随里程计', '#8b5d0e'
    age += max(0, lag)
    if age <= 2:
        return f'最近雷达匹配 {age:.1f} 秒前', '#34745c'
    if moved:
        return f'运动后已 {age:.1f} 秒未获新雷达匹配；当前 TF 主要随轮速推算', '#b53a36'
    return f'静止或移动未达取帧阈值 · 最近雷达匹配 {age:.1f} 秒前', '#526579'


def health(state):
    if 'error' in state:
        return state['error']['message'], '#b53a36'
    if 'frame' not in state:
        return '正在连接，等待地图和雷达…', '#8b5d0e'
    lag = time.monotonic() - state.get('received', 0)
    if lag > 2:
        return f'连接数据已 {lag:.1f} 秒未更新，画面仅为最后一帧', '#b53a36'
    frame = state['frame']
    raw_mode = frame.get('odometry_mode') == 'raw'
    mode_text = '原始轮速直通，未使用 EKF' if raw_mode else 'EKF 里程计'
    if frame.get('map_mode') == 'empty':
        return f'无底图 · {mode_text}；启动底盘并确认里程计更新后，点击「准备新建图」。', '#8b5d0e'
    if frame.get('map_mode') == 'saved':
        if 'map' not in state:
            return '已有地图模式 · 等待静态地图', '#8b5d0e'
        localization = frame.get('localization') or {}
        if localization.get('active'):
            return ('已有地图定位 · ' + localization.get('reason', '等待定位状态'),
                    '#34745c' if localization.get('ready') else '#8b5d0e')
        if frame.get('tf_error'):
            return '已有地图保持不变 · 定位未就绪；右侧雷达在机器人坐标系中实时显示', '#8b5d0e'
        return '已有地图保持不变 · 仅更新定位和实时雷达', '#34745c'
    if frame.get('mapping_paused'):
        filtered = frame.get('topics', {}).get('/odom', {})
        values = filtered.get('latest') or {}
        variance = values.get('position_variance', values.get('x_variance'))
        age = filtered.get('age')
        if not raw_mode and age is not None and age + lag <= .5 and variance is not None and math.isfinite(variance) and variance > 1:
            return (f'建图已暂停，不再采集 · EKF 位置方差 {variance:.2f} m² 超过当前阈值 1 m²；工作地图仍可保存。', '#b53a36')
        return '建图已暂停，不再采集 · 工作地图保留，可另存；输入恢复后需明确点击「继续建图」。', '#b53a36'
    topics = frame['topics']
    source = frame.get('odometry_source') or frame.get('ekf_source')
    if not source:
        return '里程计来源尚未确认；地图与位姿精度未验证', '#8b5d0e'
    age = topics.get(source, {}).get('age')
    if age is None or age + lag > .5:
        return f'原始输入 {source} 无新数据；当前位姿不适合建图', '#b53a36'
    filtered = topics.get('/odom', {})
    if filtered.get('age') is None or filtered['age'] + lag > .5:
        return '建图里程计未更新；暂不开始建图', '#b53a36'
    values = filtered.get('latest') or {}
    variance = values.get('position_variance', values.get('x_variance'))
    if variance is None or not math.isfinite(variance):
        return '里程计位置方差无有效数据；暂不开始建图', '#b53a36'
    if not raw_mode and variance > 1:
        return f'EKF 位置方差 {variance:.2f} m² 超过当前建图阈值 1 m²；请检查配置与判据。', '#b53a36'
    if frame.get('tf_error'):
        return frame['tf_error'], '#8b5d0e'
    map_age = topics.get('/map', {}).get('age')
    if map_age is None or map_age + lag > 15:
        return '地图尚未收到或已超过 15 秒未更新', '#8b5d0e'
    message, color = matching_health(frame, lag)
    return f'{mode_text} · {message}', color


def setup_plotting():
    import matplotlib
    from matplotlib import font_manager
    font = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if font.exists():
        font_manager.fontManager.addfont(str(font))
        matplotlib.rcParams['font.family'] = font_manager.FontProperties(fname=str(font)).get_name()
    matplotlib.rcParams['axes.unicode_minus'] = False


def footprint_outline(frame, key, lag=0.):
    """Close a fresh map-frame Nav2 polygon; points already include robot pose."""
    footprints = frame.get('footprints')
    item = footprints.get(key) if isinstance(footprints, dict) else None
    if not isinstance(item, dict) or item.get('frame_id') != frame.get('frame'):
        return []
    age = item.get('age')
    if (any(isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) for value in (age, lag))
            or age < 0 or age + max(0., lag) > 1.):
        return []
    points = item.get('points')
    if not isinstance(points, (list, tuple)) or not 3 <= len(points) <= 256:
        return []
    if any(not isinstance(point, (list, tuple)) or len(point) != 2
           or any(isinstance(value, bool) or not isinstance(value, (int, float))
                  or not math.isfinite(value) for value in point) for point in points):
        return []
    points = [list(point) for point in points]
    area = sum(a[0]*b[1]-b[0]*a[1] for a, b in zip(points, points[1:]+points[:1]))
    if abs(area) < 1e-9:
        return []
    return points + [points[0]]


class MapPlot:
    def __init__(self):
        from matplotlib.figure import Figure
        self.figure = Figure(figsize=(12, 7), dpi=100, facecolor='#f6f8fa')
        self.show_status_in_plot = True
        layout = self.figure.add_gridspec(1, 2, width_ratios=[2, 1], wspace=.22)
        self.ax = self.figure.add_subplot(layout[0])
        self.local_ax = self.figure.add_subplot(layout[1])
        self.local_ax.set(xlabel='机器人 X / m', ylabel='机器人 Y / m', aspect='equal', xlim=(-5, 5), ylim=(-5, 5))
        self.local_ax.set_title('实时雷达 · 机器人周围 10 m', fontsize=10)
        self.local_ax.grid(alpha=.2)
        self.local_front = self.local_ax.scatter([], [], s=3, c='#168caf')
        self.local_back = self.local_ax.scatter([], [], s=3, c='#ef852b')
        self.local_ax.annotate('', xy=(.5, 0), xytext=(0, 0), arrowprops={'arrowstyle': '-|>', 'color': '#c53451', 'lw': 2})
        self.figure.subplots_adjust(left=.07, right=.98, bottom=.09, top=.86)
        self.title = self.figure.suptitle('等待实时 /map', fontsize=11)
        self.ax.set(xlabel='地图 X / m', ylabel='地图 Y / m', aspect='equal', xlim=(-5, 5), ylim=(-5, 5))
        self.ax.set_facecolor('#d5d8dc')
        self.ax.grid(alpha=.15)
        self.map_artist = None
        self.map_object = None
        self.bounds = None
        self.arrow = None
        self.empty_text = self.ax.text(.5, .5, '尚无底图', transform=self.ax.transAxes, ha='center', va='center', color='#64748b', fontsize=16)
        self.front = self.ax.scatter([], [], s=3, c='#168caf', label='前雷达', zorder=3)
        self.back = self.ax.scatter([], [], s=3, c='#ef852b', label='后雷达', zorder=3)
        self.robot, = self.ax.plot([], [], 'o', color='#c53451', markersize=6, label='TF 估计位置', zorder=4)
        self.footprint_artists = {}
        for key, color, style, label in (
                ('local', '#16803c', '-', '局部碰撞轮廓'),
                ('global', '#b56a08', '--', '全局代价地图轮廓')):
            artist, = self.ax.plot([], [], color=color, linestyle=style,
                                   linewidth=1.8, zorder=6, label=label)
            self.footprint_artists[key] = artist
        self.navigation_pose = None
        self.goal_marker, = self.ax.plot([], [], marker='x', linestyle='none', color='#dc2626',
                                        markersize=15, markeredgewidth=3, zorder=7, label='导航目标')
        self.path_artist, = self.ax.plot([], [], '--', color='#7c3aed', linewidth=1.8,
                                        zorder=5, label='预览路径')
        self.goal_arrow = None
        self.goal_text = self.ax.text(0, 0, '', color='#b91c1c', fontsize=9, zorder=8)
        self.pose_note = self.ax.text(0, 0, '', fontsize=9, color='#b7790c', zorder=6)
        self.pose_note.set_visible(False)
        self.ax.legend(loc='upper right', fontsize=9)

    def update(self, state, follow=False):
        from matplotlib.transforms import Affine2D
        grid = None if state.get('frame', {}).get('map_mode') == 'empty' else state.get('map')
        self.empty_text.set_visible(grid is None)
        self.ax.get_legend().set_visible(grid is not None)
        if grid is None and self.map_object is not None:
            self.map_artist.remove()
            self.map_artist = self.map_object = self.bounds = None
            self.ax.set_xlim(-5, 5)
            self.ax.set_ylim(-5, 5)
        if grid is not None and grid is not self.map_object:
            cells = np.frombuffer(zlib.decompress(base64.b64decode(grid['data'])), dtype=np.int8)
            cells = cells.reshape((grid['height'], grid['width']))
            gray = np.where(cells < 0, .78, .98 - np.clip(cells, 0, 100) * .0098)
            x, y, angle = grid['origin']
            w, h = grid['width'] * grid['resolution'], grid['height'] * grid['resolution']
            affine = Affine2D().rotate(angle).translate(x, y)
            if self.map_artist is not None:
                self.map_artist.remove()
            self.map_artist = self.ax.imshow(gray, cmap='gray', vmin=0, vmax=1, origin='lower',
                                             extent=(0, w, 0, h), interpolation='nearest',
                                             transform=affine + self.ax.transData, zorder=0)
            corners = affine.transform([[0, 0], [w, 0], [w, h], [0, h]])
            self.bounds = (corners[:, 0].min(), corners[:, 0].max(), corners[:, 1].min(), corners[:, 1].max())
            first = self.map_object is None
            self.map_object = grid
            if first:
                self.fit()
        frame = state.get('frame', {})
        lag = time.monotonic() - state.get('received', 0)
        fresh = 0 <= lag <= 2 and 'error' not in state
        scans = frame.get('scans', {}) if fresh else {}
        local = frame.get('local_scans', {}) if fresh else {}
        self.local_front.set_offsets(np.asarray(local.get('/front/scan', [])).reshape(-1, 2))
        self.local_back.set_offsets(np.asarray(local.get('/back/scan', [])).reshape(-1, 2))
        self.front.set_offsets(np.asarray(scans.get('/front/scan', [])).reshape(-1, 2))
        self.back.set_offsets(np.asarray(scans.get('/back/scan', [])).reshape(-1, 2))
        if self.arrow is not None:
            self.arrow.remove()
            self.arrow = None
        pose = frame.get('pose') if fresh else None
        self.robot.set_data([pose[0]] if pose else [], [pose[1]] if pose else [])
        # Footprints arrive in map coordinates from the observer, including
        # Nav2's padding and pose. Do not rotate/translate or pad them again.
        revision = frame.get('map_revision')
        footprint_visible = bool(fresh and pose and grid is not None
                                 and grid.get('frame') == frame.get('frame')
                                 and type(revision) is int
                                 and grid.get('map_revision') == revision)
        for key, artist in self.footprint_artists.items():
            outline = footprint_outline(frame, key, lag) if footprint_visible else []
            artist.set_data([point[0] for point in outline], [point[1] for point in outline])
            artist.set_visible(bool(outline))
        candidate = frame.get('map_mode') == 'saved' and not (frame.get('localization') or {}).get('ready')
        pose_color = '#d18a16' if candidate else '#c53451'
        self.robot.set_color(pose_color)
        self.pose_note.set_visible(bool(pose and candidate))
        if pose:
            x, y, angle = pose
            self.pose_note.set_position((x+.15, y+.15))
            self.pose_note.set_text('候选定位 · 未就绪' if candidate else '')
            self.arrow = self.ax.annotate('', xy=(x + .6 * math.cos(angle), y + .6 * math.sin(angle)),
                                          xytext=(x, y), arrowprops={'arrowstyle': '-|>', 'color': pose_color, 'lw': 2}, zorder=4)
            if follow:
                self.ax.set_xlim(x - 5, x + 5)
                self.ax.set_ylim(y - 5, y + 5)
        stamp = datetime.datetime.fromtimestamp(frame['time']).strftime('%H:%M:%S') if frame else '--:--:--'
        mode = {'saved': '已有地图（固定底图）', 'mapping': '工作地图', 'empty': '无底图', 'unknown': '等待地图模式'}.get(frame.get('map_mode'), '地图')
        self.title.set_text(f'{mode} · {stamp}' + ('\n'+health(state)[0] if self.show_status_in_plot else ''))

    def set_navigation(self, pose=None, points=(), label=''):
        self.navigation_pose = dict(pose) if pose else None
        self.goal_marker.set_data([pose['x']] if pose else [], [pose['y']] if pose else [])
        self.path_artist.set_data([p[0] for p in points], [p[1] for p in points])
        if self.goal_arrow is not None:
            self.goal_arrow.remove()
            self.goal_arrow = None
        self.goal_text.set_visible(bool(pose))
        if pose:
            x, y, yaw = pose['x'], pose['y'], pose['yaw']
            self.goal_text.set_position((x+.12, y+.12))
            self.goal_text.set_text(f"目标 ({x:.2f}, {y:.2f}) · {label}")
            self.goal_arrow = self.ax.annotate('', xy=(x+.45*math.cos(yaw), y+.45*math.sin(yaw)),
                                              xytext=(x, y), zorder=6,
                                              arrowprops={'arrowstyle': '->', 'color': '#dc2626'})

    def fit(self):
        if self.bounds:
            x0, x1, y0, y1 = self.bounds
            if self.navigation_pose:
                x, y = self.navigation_pose['x'], self.navigation_pose['y']
                x0, x1, y0, y1 = min(x0, x), max(x1, x), min(y0, y), max(y1, y)
            self.ax.set_xlim(x0 - .5, x1 + .5)
            self.ax.set_ylim(y0 - .5, y1 + .5)


def save_snapshot(state, plot, output):
    output.mkdir(parents=True, exist_ok=True)
    path = output / (datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f') + '-map')
    path.with_suffix('.json').write_text(json.dumps(
        {k: v for k, v in state.items() if k != 'received'}, ensure_ascii=False), encoding='utf-8')
    plot.figure.savefig(path.with_suffix('.png'), dpi=140)
    return path.with_suffix('.png')


def window(stream, output):
    from .panel import window as gtk_window
    gtk_window(stream, output)


def main():
    parser = argparse.ArgumentParser(description='默认查看已有地图；提供明确的新建、暂停、另存和返回操作。')
    parser.add_argument('--host', default='192.168.58.3')
    parser.add_argument('--user', default='fairino')
    parser.add_argument('--check', action='store_true', help='短时只读检查并保存图像，不打开窗口')
    parser.add_argument('--duration', type=float, default=12, help='--check 的采样秒数，默认 12')
    parser.add_argument('--output', type=Path, default=LOG_ROOT / 'map-viewer')
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_.:-]+', args.host) or not re.fullmatch(r'[A-Za-z0-9_-]+', args.user):
        parser.error('无效的 SSH 主机或用户名')
    if not math.isfinite(args.duration) or not 3 <= args.duration <= 3600:
        parser.error('duration 必须在 3 到 3600 秒之间')
    password = os.environ.pop('FAIRINO_SSH_PASSWORD', None)
    if password is None:
        password = getpass.getpass(f'SSH 密码（{args.user}@{args.host}）: ')
    if args.check:
        import matplotlib
        matplotlib.use('Agg')
    setup_plotting()
    stream = Stream(args, password)
    password = None
    if not args.check:
        window(stream, args.output)
        return 0
    print(f'只读观察 {args.duration:g} 秒：地图、雷达、TF 和里程计…', flush=True)
    stream.thread.start()
    try:
        stream.thread.join(args.duration + 25)
    except KeyboardInterrupt:
        pass
    stream.stop.set()
    stream.thread.join(5)
    state = stream.snapshot()
    print(health(state)[0])
    if 'map' not in state:
        if state.get('frame', {}).get('map_mode') != 'empty':
            print('未收到地图。')
            return 1
        print('地图库为空，按预期没有 /map；保存空白底图与实时雷达预览。')
    plot = MapPlot()
    plot.update(state)
    print('图像：', save_snapshot(state, plot, args.output))
    frame = state.get('frame', {})
    print(json.dumps({'ekf_source': frame.get('ekf_source'), 'topics': frame.get('topics'),
                      'pose': frame.get('pose'), 'tf_error': frame.get('tf_error'),
                      'scan_points': {k: len(v) for k, v in frame.get('scans', {}).items()}},
                     ensure_ascii=False))
    return 1 if 'error' in state or 'frame' not in state else 0


if __name__ == '__main__':
    raise SystemExit(main())
