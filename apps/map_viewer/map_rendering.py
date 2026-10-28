"""Pure desktop geometry for the map canvas; no robot commands or ROS imports."""
from collections import deque
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


COLORS = {'paper': '#f8faf9', 'free': '#fbfcfa', 'unknown': '#dce3e6',
          'occupied': '#344654', 'ink': '#334654', 'muted': '#788994',
          'robot': '#087f83', 'front': '#14a3b8', 'back': '#d89135',
          'particles': '#8b66ba', 'global': '#3478cf', 'local': '#e68936',
          'trail': '#369a98', 'preview': '#778bba', 'candidate': '#d18a16'}


def points_array(values, columns=2):
    """Keep only complete finite rows; malformed telemetry must not break drawing."""
    try:
        points = np.asarray(values, dtype=float)
        if points.ndim != 2 or points.shape[1] < columns:
            return np.empty((0, columns))
        points = points[:, :columns]
        return points[np.isfinite(points).all(axis=1)]
    except (ValueError, TypeError):
        return np.empty((0, columns))


def pose_value(value):
    try:
        result = np.asarray(value, dtype=float)
        if result.shape == (3,) and np.isfinite(result).all():
            return result.tolist()
    except (TypeError, ValueError):
        pass
    return None


def session_key(frame):
    token = frame.get('display_session')
    if token is None:
        loc = frame.get('localization') or {}
        token = [frame.get('map_mode'), frame.get('saved_map_source'),
                 loc.get('id'), frame.get('odometry_epoch')]
    return json.dumps(token, ensure_ascii=False, sort_keys=True, default=str)


def covariance_ellipse(covariance):
    """95% XY ellipse (chi-square(2) = 5.991), never synthesized from a scalar."""
    try:
        xx, xy, yy = covariance
        matrix = np.asarray([[xx, xy], [xy, yy]], dtype=float)
        if not np.isfinite(matrix).all():
            return None
        values, vectors = np.linalg.eigh(matrix)
        if values.min() < -1e-12:
            return None
        values = np.maximum(values, 0)
        direction = vectors[:, 1]
        return (2*math.sqrt(5.991*values[1]), 2*math.sqrt(5.991*values[0]),
                math.degrees(math.atan2(direction[1], direction[0])))
    except (TypeError, ValueError, np.linalg.LinAlgError):
        return None


class PoseTrail:
    """Bounded TF estimate history, broken across sessions, gaps and pose jumps."""
    def __init__(self, maximum=1800):
        self.samples = deque(maxlen=maximum)
        self.previous = None
        self.broken = True

    def clear(self):
        self.samples.clear()
        self.previous = None
        self.broken = True

    def pause(self):
        self.broken = True

    def add(self, pose, stamp, session):
        if pose_value(pose) is None or type(stamp) not in (float, int) or not math.isfinite(stamp):
            self.pause()
            return
        new_segment = self.broken
        if self.previous is not None:
            old, when, old_session = self.previous
            elapsed = stamp-when
            distance = math.hypot(pose[0]-old[0], pose[1]-old[1])
            turn = abs(math.atan2(math.sin(pose[2]-old[2]), math.cos(pose[2]-old[2])))
            new_segment |= session != old_session or elapsed > 2 or elapsed < 0 or distance > 1 or turn > 1.2
            if not new_segment and (elapsed == 0 or (distance < .02 and turn < math.radians(2))):
                # A stationary update refreshes the gap clock, without filling the history.
                self.previous = (old, stamp, session)
                return
        self.samples.append((tuple(pose[:2]), bool(new_segment), session))
        self.previous = (tuple(pose), stamp, session)
        self.broken = False

    def segments(self):
        result = []
        previous = None
        for point, starts_segment, _session in self.samples:
            if previous is not None and not starts_segment:
                result.append((previous, point))
            previous = point
        return result

    def segment_sessions(self):
        return [session for index, (_point, starts_segment, session) in enumerate(self.samples)
                if index and not starts_segment]


class RobotGlyph:
    """Metric URDF footprint with wheel locations and a clearly visible +X nose.

    Wheels are geometry cues, not inferred steering feedback. All patches share
    the exact map transform; zoom never changes the robot's physical dimensions.
    """
    def __init__(self, axes, urdf, footprint):
        from matplotlib.patches import Polygon
        self.axes = axes
        self.points = np.asarray(footprint, dtype=float)
        self.halo = Polygon(self.points*1.25, closed=True, facecolor=COLORS['robot'],
                            edgecolor='none', alpha=.12, zorder=8)
        self.body = Polygon(self.points, closed=True, facecolor='#d4efea',
                            edgecolor=COLORS['robot'], linewidth=1.8, zorder=10)
        self.deck = Polygon(self.points*.72, closed=True, facecolor=COLORS['robot'],
                            edgecolor='white', linewidth=.6, alpha=.9, zorder=11)
        front = self.points[:, 0].max()
        self.nose = Polygon([[front+.10, 0], [front-.10, .12], [front-.10, -.12]],
                            closed=True, facecolor=COLORS['robot'], edgecolor='white',
                            linewidth=.8, zorder=15)
        self.wheels = []
        root = ET.parse(Path(urdf)).getroot()
        for joint in root.findall('joint'):
            child = joint.find('child')
            if child is None or not child.get('link', '').endswith('_steering_link'):
                continue
            x, y, _ = [float(v) for v in joint.find('origin').get('xyz').split()]
            # The runtime URDF wheel is a 65 mm radius, 30 mm wide cylinder.
            wheel_link = root.find("./link[@name='%s']" % child.get('link').replace('_steering_link', '_wheel_link'))
            cylinder = wheel_link.find('visual/geometry/cylinder')
            r, width = float(cylinder.get('radius')), float(cylinder.get('length'))
            vertices = [[x-r, y-width/2], [x+r, y-width/2], [x+r, y+width/2], [x-r, y+width/2]]
            self.wheels.append(Polygon(vertices, closed=True, facecolor=COLORS['ink'],
                                       edgecolor='white', linewidth=.5, zorder=12))
        self.artists = [self.halo, self.body, self.deck, *self.wheels, self.nose]
        for artist in self.artists:
            axes.add_patch(artist)
            artist.set_visible(False)
        self.pose = None

    def update(self, pose, *, stale=False, candidate=False, selected=True):
        from matplotlib.transforms import Affine2D
        self.pose = pose_value(pose)
        for artist in self.artists:
            artist.set_visible(self.pose is not None)
        if self.pose is None:
            return
        x, y, yaw = self.pose
        transform = Affine2D().rotate(yaw).translate(x, y) + self.axes.transData
        for artist in self.artists:
            artist.set_transform(transform)
        color = COLORS['muted'] if stale else COLORS['candidate'] if candidate else COLORS['robot']
        self.halo.set_visible(selected and not stale)
        self.halo.set_facecolor(color)
        self.body.set_facecolor('#e0e5e7' if stale else '#fff1d6' if candidate else '#d4efea')
        self.body.set_edgecolor(color)
        self.deck.set_facecolor(color)
        self.nose.set_facecolor(color)
        for wheel in self.wheels:
            wheel.set_facecolor(COLORS['muted'] if stale else COLORS['ink'])


class PosePreview:
    """Unsent mouse-picked pose, deliberately independent of live TF and trails.

    The chassis outline uses real map metres. The arrow follows the pointer:
    its length only helps choose a heading and does not specify a movement.
    Neither update nor clear schedules a draw or changes the map limits.
    """
    MIN_DIRECTION_LENGTH = .05

    def __init__(self, axes, footprint):
        from matplotlib.patches import FancyArrowPatch, Polygon
        import matplotlib.patheffects as effects
        self.axes = axes
        self.points = np.asarray(footprint, dtype=float)
        self.body = Polygon(self.points, closed=True, facecolor='#fff0d9',
                            edgecolor='#d56b08', linewidth=2, linestyle='--',
                            alpha=.9, zorder=35, visible=False)
        axes.add_patch(self.body)
        self.arrow = FancyArrowPatch((0., 0.), (0., 0.), arrowstyle='-|>',
                                     mutation_scale=25, linewidth=3,
                                     color='#d56b08', shrinkA=0, shrinkB=0,
                                     zorder=37, visible=False)
        self.arrow.set_path_effects([effects.Stroke(linewidth=6, foreground='white'), effects.Normal()])
        axes.add_patch(self.arrow)
        self.origin, = axes.plot([], [], marker='o', markersize=9, linestyle='none',
                                  markerfacecolor='#d56b08', markeredgecolor='white',
                                  markeredgewidth=2, zorder=38, visible=False)
        self.note = axes.text(.985, .025, '', transform=axes.transAxes, ha='right', va='bottom',
                              fontsize=10, color='#8d4605', linespacing=1.5, zorder=39,
                              visible=False,
                              bbox=dict(boxstyle='round,pad=.6', facecolor='#fff7e9',
                                        edgecolor='#d56b08', linewidth=1.2, alpha=.97))
        self.artists = [self.body, self.arrow, self.origin, self.note]
        self.pose = None
        self.start = self.end = self.kind = None

    @staticmethod
    def _point(value):
        try:
            point = np.asarray(value, dtype=float)
            if point.shape == (2,) and np.isfinite(point).all():
                return tuple(point.tolist())
        except (ValueError, TypeError, OverflowError):
            pass
        return None

    def set(self, start, end=None, kind='initial'):
        from matplotlib.transforms import Affine2D
        if kind not in ('initial', 'goal'):
            raise ValueError('位姿预览类型必须为 initial 或 goal')
        start_value = self._point(start)
        end_value = self._point(end) if end is not None else None
        if start_value is None or (end is not None and end_value is None):
            self.clear()
            return False
        x, y = start_value
        dx, dy = (end_value[0]-x, end_value[1]-y) if end_value is not None else (0., 0.)
        length = math.hypot(dx, dy)
        if not math.isfinite(length):
            self.clear()
            return False
        self.start, self.end, self.kind = start_value, end_value, kind
        color, ink, fill = ('#d56b08', '#8d4605', '#fff7e9') if kind == 'initial' else ('#c74740', '#9b302c', '#fff0ed')
        title = '初始位姿预览 · 尚未发送' if kind == 'initial' else '导航目标预览 · 尚未发送'
        self.origin.set_data([x], [y])
        self.origin.set_markerfacecolor(color)
        self.origin.set_visible(True)
        self.note.set_color(ink)
        self.note.get_bbox_patch().set(facecolor=fill, edgecolor=color)
        self.note.set_visible(True)
        direction_valid = length >= self.MIN_DIRECTION_LENGTH
        self.body.set_visible(direction_valid)
        self.arrow.set_visible(direction_valid)
        self.pose = None
        if direction_valid:
            yaw = math.atan2(dy, dx)
            self.pose = [x, y, yaw]
            self.body.set_transform(Affine2D().rotate(yaw).translate(x, y)+self.axes.transData)
            self.body.set_edgecolor(color)
            self.body.set_facecolor(fill)
            self.arrow.set_positions(start_value, end_value)
            self.arrow.set_color(color)
            detail = f'X {x:.2f} m   Y {y:.2f} m   朝向 {math.degrees(yaw):.1f}°'
            hint = '箭头长度仅表示方向 · 松开后确认'
        else:
            detail = f'X {x:.2f} m   Y {y:.2f} m'
            hint = '按住鼠标，向机器人前方拖动设置朝向'
        self.note.set_text(f'{title}\n{detail}\n{hint}')
        return True

    def clear(self):
        self.pose = self.start = self.end = self.kind = None
        self.origin.set_data([], [])
        self.note.set_text('')
        for artist in self.artists:
            artist.set_visible(False)
