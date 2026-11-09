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
ROOT = HERE.parent.parent
PREFIX = 'MAP_VIEWER '
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from apps.workbench.slam_support import ensure_slam
ensure_slam()
from fairino_slam.mapping.client import execute as map_action
from fairino_slam.mapping.viewer import Stream, footprint_outline


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


class MapPlot:
    """Desktop-only map layers. Telemetry is displayed, never used as control input."""
    def __init__(self):
        from matplotlib.figure import Figure
        from matplotlib.collections import LineCollection
        from matplotlib.patches import Ellipse
        if str(ROOT) not in sys.path:
            sys.path.insert(0, str(ROOT))
        from apps.navigation.footprint import ChassisFootprint
        from apps.map_viewer.map_rendering import COLORS, PoseTrail, RobotGlyph, PosePreview
        self.colors = COLORS
        self.figure = Figure(figsize=(12, 7), dpi=100, facecolor=COLORS['paper'])
        self.show_status_in_plot = True
        self.ax = self.figure.add_axes([.065, .095, .91, .83])
        self.ax.set(xlabel='地图 X / m', ylabel='地图 Y / m', aspect='equal',
                    xlim=(-5, 5), ylim=(-5, 5), facecolor=COLORS['unknown'])
        self.ax.set_axisbelow(True)
        self.ax.grid(color='#99aab5', alpha=.13, linewidth=.6)
        self.ax.tick_params(colors=COLORS['muted'], labelsize=9, length=3)
        self.ax.xaxis.label.set(color=COLORS['ink'], size=9)
        self.ax.yaxis.label.set(color=COLORS['ink'], size=9)
        for spine in self.ax.spines.values():
            spine.set_color('#b6c2c9')
        self.title = self.figure.suptitle('', fontsize=10, color=COLORS['ink'], y=.985)
        self.mode_label = self.ax.text(.014, .982, '等待地图', transform=self.ax.transAxes,
                                       va='top', color=COLORS['ink'], fontsize=10, zorder=20,
                                       bbox=dict(facecolor=COLORS['paper'], edgecolor='none', alpha=.92, pad=5))
        self.connection_note = self.ax.text(.5, .982, '', transform=self.ax.transAxes,
                                           ha='center', va='top', fontsize=10, color='#8a5e28', zorder=21,
                                           bbox=dict(facecolor='#fff7e6', edgecolor='none', alpha=.95, pad=5))
        self.connection_note.set_visible(False)
        self.local_ax = self.ax.inset_axes([.735, .66, .245, .29], zorder=22)
        self.local_ax.set(xlabel='X / m', ylabel='Y / m', aspect='equal', xlim=(-5, 5),
                          ylim=(-5, 5), facecolor=COLORS['paper'])
        self.local_ax.set_title('本体雷达 · 10 m', fontsize=9, color=COLORS['ink'], pad=5)
        self.local_ax.set_xticks([-4, 0, 4])
        self.local_ax.set_yticks([-4, 0, 4])
        self.local_ax.tick_params(labelsize=7, colors=COLORS['muted'], length=2, pad=2)
        self.local_ax.xaxis.label.set(size=7, color=COLORS['muted'])
        self.local_ax.xaxis.labelpad = 1
        self.local_ax.yaxis.label.set(size=7, color=COLORS['muted'])
        self.local_ax.yaxis.labelpad = 1
        for spine in self.local_ax.spines.values():
            spine.set_color('#aebfc6')
        self.local_ax.grid(color='#cad3d8', alpha=.55, linewidth=.5)
        self.local_front = self.local_ax.scatter([], [], s=3, color=COLORS['front'], zorder=3)
        self.local_back = self.local_ax.scatter([], [], s=3, color=COLORS['back'], zorder=3)
        urdf = ROOT / 'models' / 'amr_8dof.urdf'
        footprint = ChassisFootprint(urdf).points
        self.robot_shape = RobotGlyph(self.ax, urdf, footprint)
        self.pose_preview = PosePreview(self.ax, footprint)
        self.local_robot = RobotGlyph(self.local_ax, urdf, footprint)
        self.local_robot.update([0., 0., 0.], selected=False)
        self.map_artist = self.map_object = self.bounds = None
        self.arrow = None
        self.empty_text = self.ax.text(.5, .5, '尚无底图', transform=self.ax.transAxes,
                                       ha='center', va='center', color=COLORS['muted'], fontsize=18)
        self.front = self.ax.scatter([], [], s=5, color=COLORS['front'], linewidths=0, zorder=6, label='前雷达')
        self.back = self.ax.scatter([], [], s=5, color=COLORS['back'], linewidths=0, zorder=6, label='后雷达')
        # Retain the historic pose-data artist API; the actual robot is a metric
        # footprint, not a point. Existing panel consumers can still read data.
        self.robot, = self.ax.plot([], [], color='#c53451', visible=False)
        self.last_pose = None
        self.last_session = None
        self.last_map_identity = None
        self.last_frame = {}
        self.trail = PoseTrail()
        self.trail_artist = LineCollection([], colors=COLORS['trail'], linewidths=1.7,
                                            alpha=.65, zorder=3, label='TF 估计轨迹')
        self.ax.add_collection(self.trail_artist)
        self.particles = self.ax.scatter([], [], color=COLORS['particles'], s=6,
                                         linewidths=0, alpha=.4, zorder=14, label='AMCL 粒子')
        self.particle_headings = LineCollection([], colors=COLORS['particles'], linewidths=.6, alpha=.25, zorder=14)
        self.ax.add_collection(self.particle_headings)
        self.particle_note = self.ax.text(.014, .12, '', transform=self.ax.transAxes,
                                          color=COLORS['particles'], fontsize=8, zorder=20, visible=False,
                                          bbox=dict(facecolor=COLORS['paper'], edgecolor='none', alpha=.9, pad=3))
        self.uncertainty = Ellipse((0, 0), 0, 0, facecolor=COLORS['particles'],
                                   edgecolor=COLORS['particles'], alpha=.24, linewidth=1.2,
                                   zorder=14, visible=False, label='AMCL 95% 区域')
        self.ax.add_patch(self.uncertainty)
        self.uncertainty_note = self.ax.text(0, 0, '', color='#7955a3', fontsize=8,
                                             zorder=16, visible=False)
        self.global_path, = self.ax.plot([], [], '-', color=COLORS['global'], linewidth=2.1,
                                         zorder=5, label='控制器全局路径')
        self.local_path, = self.ax.plot([], [], '-', color=COLORS['local'], linewidth=2.4,
                                        zorder=6, label='局部预测路径')
        self.footprint_artists = {}
        for key, color, title in (('local', '#e59b38', '局部碰撞轮廓'),
                                  ('global', '#617bc6', '全局保守轮廓')):
            artist, = self.ax.plot([], [], '--', color=color, linewidth=1.3,
                                    zorder=7, label=title)
            self.footprint_artists[key] = artist
        self.navigation_pose = None
        self.goal_marker, = self.ax.plot([], [], marker='x', linestyle='none', color='#c45c4b',
                                        markersize=12, markeredgewidth=2, zorder=15, label='导航目标')
        self.path_artist, = self.ax.plot([], [], '--', color=COLORS['preview'], linewidth=1.6,
                                        zorder=4, label='导航预览')
        self.goal_arrow = None
        self.goal_text = self.ax.text(0, 0, '', color='#9b4e40', fontsize=8, zorder=16)
        self.pose_note = self.ax.text(0, 0, '', fontsize=9, color='#b7790c', zorder=16, visible=False,
                                     bbox=dict(facecolor=COLORS['paper'], edgecolor='none', alpha=.9, pad=3))
        self.scale_artist, = self.ax.plot([], [], color=COLORS['ink'], linewidth=2,
                                         transform=self.ax.transAxes, zorder=20, solid_capstyle='butt')
        self.scale_text = self.ax.text(.035, .035, '', transform=self.ax.transAxes,
                                       fontsize=8, color=COLORS['ink'], zorder=20,
                                       bbox=dict(facecolor=COLORS['paper'], edgecolor='none', alpha=.9, pad=2))
        self.layers = dict.fromkeys(('particles', 'scans', 'trail', 'uncertainty', 'local_radar', 'paths', 'footprints'), True)
        self.ax.callbacks.connect('xlim_changed', self._scale)
        self.ax.callbacks.connect('ylim_changed', self._scale)
        self._legend_signature = None
        self._set_view(-5, 5, -5, 5)
        self.figure.canvas.mpl_connect('resize_event', self._canvas_resized)
        self._scale()
        self._legend()

    def _set_view(self, x0, x1, y0, y1):
        # Expand the visible extent to fill the canvas without distorting metres.
        box = self.ax.get_position(original=True)
        width, height = self.figure.get_size_inches()
        ratio = max(.1, width*box.width/(height*box.height))
        x, y = (x0+x1)/2, (y0+y1)/2
        sx, sy = max(x1-x0, .01), max(y1-y0, .01)
        if sx/sy < ratio:
            sx = sy*ratio
        else:
            sy = sx/ratio
        self.ax.set_xlim(x-sx/2, x+sx/2)
        self.ax.set_ylim(y-sy/2, y+sy/2)

    def _canvas_resized(self, *_):
        self._set_view(*self.ax.get_xlim(), *self.ax.get_ylim())

    def _scale(self, *_):
        span = abs(self.ax.get_xlim()[1]-self.ax.get_xlim()[0])
        if not math.isfinite(span) or span <= 0:
            return
        target = span*.15
        power = 10**math.floor(math.log10(target))
        length = max(v*power for v in (1, 2, 5, 10) if v*power <= target)
        end = .035+length/span
        self.scale_artist.set_data([.035, .035, end, end], [.066, .06, .06, .066])
        self.scale_text.set_text(f'{length:g} m')

    def _legend(self):
        from matplotlib.lines import Line2D
        from matplotlib.patches import Patch
        # Full layer explanations live in the sidebar. Keep the canvas key to
        # at most two short rows even at small embedded window sizes.
        handles = [Patch(facecolor='#d4efea', edgecolor=self.colors['robot'], label='机器人')]
        if self.layers['scans'] and (len(self.front.get_offsets()) or len(self.back.get_offsets())):
            handles.append(Line2D([], [], marker='.', color=self.colors['front'], linestyle='none', label='前 / 后雷达'))
        if self.layers['trail'] and len(self.trail_artist.get_segments()):
            handles.append(Line2D([], [], color=self.colors['trail'], label='TF 估计轨迹'))
        if self.layers['particles'] and len(self.particles.get_offsets()) or self.uncertainty.get_visible():
            handles.append(Patch(facecolor=self.colors['particles'], alpha=.5, label='AMCL 粒子 / 95%'))
        if self.layers['paths'] and (len(self.global_path.get_xdata()) or len(self.path_artist.get_xdata())):
            handles.append(Line2D([], [], color=self.colors['global'], label='全局 / 预览'))
        if self.layers['paths'] and len(self.local_path.get_xdata()):
            handles.append(Line2D([], [], color=self.colors['local'], label='局部预测'))
        signature = tuple(a.get_label() for a in handles)
        if signature == self._legend_signature:
            return
        self._legend_signature = signature
        self.ax.legend(handles=handles, loc='upper left', bbox_to_anchor=(.008, .925),
                       fontsize=7.5, ncol=3, framealpha=.93, edgecolor='#dae2e6',
                       facecolor=self.colors['paper'], labelcolor=self.colors['ink'],
                       handlelength=1.2, columnspacing=.8, borderpad=.4,
                       labelspacing=.35, handletextpad=.4).set_zorder(30)

    def set_layer(self, name, enabled):
        if name not in self.layers:
            raise ValueError(f'未知地图图层：{name}')
        self.layers[name] = bool(enabled)
        self._apply_layers()
        self._legend()

    def _apply_layers(self):
        for artist in (self.front, self.back, self.local_front, self.local_back):
            artist.set_visible(self.layers['scans'])
        self.local_ax.set_visible(self.layers['local_radar'])
        self.trail_artist.set_visible(self.layers['trail'])
        self.particles.set_visible(self.layers['particles'])
        self.particle_headings.set_visible(self.layers['particles'])
        self.particle_note.set_visible(self.layers['particles'] and bool(len(self.particles.get_offsets())))
        self.uncertainty.set_visible(self.layers['uncertainty'] and getattr(self, '_ellipse_valid', False))
        # The layer key identifies the ellipse; avoid a second label over the robot.
        self.uncertainty_note.set_visible(False)
        for artist in (self.path_artist, self.global_path, self.local_path):
            artist.set_visible(self.layers['paths'])
        for artist in self.footprint_artists.values():
            artist.set_visible(self.layers['footprints'] and bool(len(artist.get_xdata())))
        target_visible = self.layers['paths'] and self.navigation_pose is not None
        self.goal_marker.set_visible(target_visible)
        self.goal_text.set_visible(target_visible)
        if self.goal_arrow is not None:
            self.goal_arrow.set_visible(target_visible)

    def clear_trail(self):
        self.trail.clear()
        self.trail_artist.set_segments([])
        self._legend()

    def set_pose_preview(self, start, end=None, kind='initial'):
        """Show an unsent drag pose; caller owns redraw and session invalidation."""
        return self.pose_preview.set(start, end, kind)

    def clear_pose_preview(self):
        self.pose_preview.clear()

    def zoom(self, factor):
        if type(factor) not in (int, float) or not math.isfinite(factor) or factor <= 0:
            raise ValueError('缩放倍率必须为有限正数')
        x0, x1 = self.ax.get_xlim()
        y0, y1 = self.ax.get_ylim()
        # factor > 1 zooms in. Keep panning centre and physical aspect unchanged.
        sx = max(.75, min(500., (x1-x0)/factor))/2
        sy = max(.75, min(500., (y1-y0)/factor))/2
        x, y = (x0+x1)/2, (y0+y1)/2
        self.ax.set_xlim(x-sx, x+sx)
        self.ax.set_ylim(y-sy, y+sy)

    def robot_focus(self):
        if self.last_pose is None:
            return False
        x, y = self.last_pose[:2]
        self._set_view(x-2.5, x+2.5, y-2.5, y+2.5)
        return True

    def update(self, state, follow=False):
        from matplotlib.colors import LinearSegmentedColormap
        from matplotlib.transforms import Affine2D
        from apps.map_viewer.map_rendering import points_array, pose_value, session_key, covariance_ellipse
        frame = state.get('frame', {})
        session = session_key(frame)
        token = frame.get('display_session')
        map_token = token[:3] if isinstance(token, list) else [frame.get('map_mode'), frame.get('saved_map_source')]
        map_identity = json.dumps(map_token, sort_keys=True, default=str)
        if self.last_session is not None and session != self.last_session:
            if self.last_map_identity != map_identity:
                self.clear_trail()
            self.last_pose = None
            self.trail.pause()
        self.last_session, self.last_frame = session, frame
        self.last_map_identity = map_identity
        grid = None if frame.get('map_mode') == 'empty' else state.get('map')
        self.empty_text.set_visible(grid is None)
        if grid is None and self.map_object is not None:
            self.map_artist.remove()
            self.map_artist = self.map_object = self.bounds = None
            self.ax.set_xlim(-5, 5)
            self.ax.set_ylim(-5, 5)
        if grid is not None and grid is not self.map_object:
            cells = np.frombuffer(zlib.decompress(base64.b64decode(grid['data'])), dtype=np.int8)
            cells = cells.reshape((grid['height'], grid['width']))
            values = np.where(cells < 0, np.nan, np.clip(cells, 0, 100)/100.)
            cmap = LinearSegmentedColormap.from_list('map_slate', [self.colors['free'], self.colors['occupied']])
            cmap = cmap.with_extremes(bad=self.colors['unknown'])
            x, y, angle = grid['origin']
            w, h = grid['width']*grid['resolution'], grid['height']*grid['resolution']
            affine = Affine2D().rotate(angle).translate(x, y)
            if self.map_artist is not None:
                self.map_artist.remove()
            self.map_artist = self.ax.imshow(values, cmap=cmap, vmin=0, vmax=1, origin='lower',
                                             extent=(0, w, 0, h), interpolation='nearest',
                                             transform=affine+self.ax.transData, zorder=0)
            corners = affine.transform([[0, 0], [w, 0], [w, h], [0, h]])
            self.bounds = (corners[:, 0].min(), corners[:, 0].max(), corners[:, 1].min(), corners[:, 1].max())
            first = self.map_object is None
            self.map_object = grid
            if first:
                self.fit()
        received = state.get('received')
        lag = time.monotonic()-received if type(received) in (int, float) and math.isfinite(received) else float('inf')
        fresh = bool(frame) and 0 <= lag <= 2 and 'error' not in state
        scans = (frame.get('scans') or {}) if fresh else {}
        local = (frame.get('local_scans') or {}) if fresh else {}
        for artist, values in ((self.front, scans.get('/front/scan', [])), (self.back, scans.get('/back/scan', [])),
                               (self.local_front, local.get('/front/scan', [])), (self.local_back, local.get('/back/scan', []))):
            artist.set_offsets(points_array(values))
        pose = pose_value(frame.get('pose')) if fresh and not frame.get('tf_error') else None
        # Nav2 footprints already include robot pose and padding in map frame.
        # Bind to the displayed map revision so old polygons never cross maps.
        revision = frame.get('map_revision')
        footprint_visible = bool(fresh and pose is not None and grid is not None
                                 and grid.get('frame') == frame.get('frame')
                                 and type(revision) is int
                                 and grid.get('map_revision') == revision)
        for key, artist in self.footprint_artists.items():
            outline = footprint_outline(frame, key, lag) if footprint_visible else []
            artist.set_data([p[0] for p in outline], [p[1] for p in outline])
        candidate = frame.get('map_mode') == 'saved'  and not (frame.get('localization') or {}).get('ready')
        self.robot.set_data([pose[0]] if pose else [], [pose[1]] if pose else [])
        self.robot.set_color('#d18a16' if candidate else '#c53451')
        if pose is not None:
            self.last_pose = pose
            self.trail.add(pose, frame.get('time'), session)
        else:
            self.trail.pause()
        self.trail_artist.set_segments(self.trail.segments())
        self.trail_artist.set_color([self.colors['trail'] if key == session else '#a6b0b7'
                                     for key in self.trail.segment_sessions()] or [self.colors['trail']])
        self.robot_shape.update(self.last_pose, stale=pose is None, candidate=candidate)
        self.local_robot.update([0., 0., 0.], stale=not fresh, selected=False)
        self.pose_note.set_visible(bool(self.last_pose and (pose is None or candidate)))
        if self.last_pose:
            x, y, _ = self.last_pose
            self.pose_note.set_position((x+.48, y+.45))
            self.pose_note.set_text('最后位置 · 已停止更新' if pose is None else '候选定位 · 未就绪')
            self.pose_note.set_color(self.colors['muted'] if pose is None else self.colors['candidate'])
        self.connection_note.set_visible(bool(frame) and not fresh)
        self.connection_note.set_text('连接数据已过期 · 显示最后位置，已停止更新')
        if follow and pose is not None:
            self.robot_focus()
        localization = frame.get('localization') or {}
        particles = frame.get('particles') or {}
        particles_valid = (fresh and particles.get('valid') and localization.get('active')
                           and localization.get('estimate_visible'))
        particle_points = points_array(particles.get('points', []), 4) if particles_valid else np.empty((0, 4))
        age = particles.get('age')
        age_text = f'{age+max(lag, 0):.1f} 秒前' if type(age) in (int, float) and math.isfinite(age) and age >= 0 else '年龄未知'
        retained = '静止保留 · ' if particles.get('state') == 'retained' else ''
        self.particle_note.set_text(f'AMCL 粒子 {particles.get("count", len(particle_points))} 个 · {retained}{age_text}')
        self.particles.set_offsets(particle_points[:, :2])
        weights = np.maximum(particle_points[:, 3], 0)
        self.particles.set_sizes(1+5*weights/max(weights.max(initial=0), 1e-12))
        starts = particle_points[:, :2]
        ends = starts+.09*np.column_stack((np.cos(particle_points[:, 2]), np.sin(particle_points[:, 2])))
        self.particle_headings.set_segments(np.stack((starts, ends), axis=1) if len(starts) else [])
        localization = frame.get('localization') or {}
        amcl_pose = pose_value(localization.get('pose'))
        ellipse = covariance_ellipse(localization.get('covariance_xy'))
        self._ellipse_valid = bool(fresh and localization.get('active') and localization.get('estimate_visible') and amcl_pose and ellipse)
        if self._ellipse_valid:
            # This covariance belongs to the last AMCL observation, not the moving TF pose.
            self.uncertainty.set_center(amcl_pose[:2])
            self.uncertainty.width, self.uncertainty.height, self.uncertainty.angle = ellipse
            self.uncertainty_note.set_position((amcl_pose[0]+max(ellipse[0]/2+.1, .55), amcl_pose[1]-.45))
            self.uncertainty_note.set_text('AMCL 95% · 上次观测')
        paths = frame.get('navigation_paths') or {}
        for name, artist in (('global', self.global_path), ('local', self.local_path)):
            item = paths.get(name) or {}
            age = item.get('age')
            valid = fresh and item.get('valid')
            points = points_array(item.get('points', [])) if valid else np.empty((0, 2))
            artist.set_data(points[:, 0], points[:, 1])
        stamp = datetime.datetime.fromtimestamp(frame['time']).strftime('%H:%M:%S') if frame.get('time') else '--:--:--'
        mode = {'saved': '已有地图', 'mapping': '建图中', 'empty': '无底图', 'unknown': '等待地图'}.get(frame.get('map_mode'), '地图')
        self.mode_label.set_text(f'{mode}  ·  {stamp}')
        self.title.set_visible(self.show_status_in_plot)
        self.title.set_text(health(state)[0] if self.show_status_in_plot else '')
        self.ax.set_position([.065, .095, .91, .83 if self.show_status_in_plot else .875])
        self._canvas_resized()
        self._apply_layers()
        self._legend()

    def set_navigation(self, pose=None, points=(), label=''):
        from apps.map_viewer.map_rendering import points_array
        self.navigation_pose = dict(pose) if pose else None
        self.goal_marker.set_data([pose['x']] if pose else [], [pose['y']] if pose else [])
        points = points_array(points)
        self.path_artist.set_data(points[:, 0], points[:, 1])
        if self.goal_arrow is not None:
            self.goal_arrow.remove()
            self.goal_arrow = None
        self.goal_text.set_visible(bool(pose))
        if pose:
            x, y, yaw = pose['x'], pose['y'], pose['yaw']
            self.goal_text.set_position((x+.2, y+.3))
            self.goal_text.set_text(f'目标 · {label}' if label else '导航目标')
            self.goal_arrow = self.ax.annotate('', xy=(x+.45*math.cos(yaw), y+.45*math.sin(yaw)),
                                              xytext=(x, y), zorder=15,
                                              arrowprops={'arrowstyle': '->', 'color': '#c45c4b', 'lw': 1.5})
        self._apply_layers()
        self._legend()

    def fit(self):
        if self.bounds:
            x0, x1, y0, y1 = self.bounds
            if self.navigation_pose:
                x, y = self.navigation_pose['x'], self.navigation_pose['y']
                x0, x1, y0, y1 = min(x0, x), max(x1, x), min(y0, y), max(y1, y)
            self._set_view(x0-.5, x1+.5, y0-.5, y1+.5)


def save_snapshot(state, plot, output):
    output.mkdir(parents=True, exist_ok=True)
    path = output / (datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f') + '-map')
    path.with_suffix('.json').write_text(json.dumps(
        {k: v for k, v in state.items() if k != 'received'}, ensure_ascii=False), encoding='utf-8')
    plot.figure.savefig(path.with_suffix('.png'), dpi=140)
    return path.with_suffix('.png')


def window(stream, output):
    # One GTK component is used both here and inside the workbench.
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    from apps.map_viewer.panel import window as gtk_window
    gtk_window(stream, output)


def main():
    parser = argparse.ArgumentParser(description='默认查看已有地图；提供明确的新建、暂停、另存和返回操作。')
    parser.add_argument('--host', default='192.168.58.3')
    parser.add_argument('--user', default='fairino')
    parser.add_argument('--check', action='store_true', help='短时只读检查并保存图像，不打开窗口')
    parser.add_argument('--demo', action='store_true', help='离线模拟地图、定位粒子与运动，不连接机器人')
    parser.add_argument('--duration', type=float, default=12, help='--check 的采样秒数，默认 12')
    parser.add_argument('--output', type=Path, default=ROOT / 'logs' / 'map-viewer')
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_.:-]+', args.host) or not re.fullmatch(r'[A-Za-z0-9_-]+', args.user):
        parser.error('无效的 SSH 主机或用户名')
    if not math.isfinite(args.duration) or not 3 <= args.duration <= 3600:
        parser.error('duration 必须在 3 到 3600 秒之间')
    if args.demo:
        if str(ROOT) not in sys.path:
            sys.path.insert(0, str(ROOT))
        from apps.map_viewer.demo import DemoStream
        if args.check:
            import matplotlib
            matplotlib.use('Agg')
        setup_plotting()
        stream = DemoStream()
        if args.check:
            plot = MapPlot()
            for step in range(120):
                stream.elapsed = step*.3
                plot.update(stream.snapshot())
            print('离线模拟图像：', save_snapshot(stream.snapshot(), plot, args.output))
        else:
            window(stream, args.output)
        return 0
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
