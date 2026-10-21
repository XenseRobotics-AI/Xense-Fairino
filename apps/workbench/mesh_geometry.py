"""Vendor visual meshes and camera math; no ROS, GUI or robot commands."""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np

from .model import ROOT, URDF, SHA256, origin
from apps.fairino_tcp_probe.tool_frames import visual_mounts, flange_from_tip, load_geometry

MESHES = ROOT / 'models/whole_body_meshes'


@dataclass
class Visual:
    link: str
    local: np.ndarray
    vertices: np.ndarray  # xyz + unit normal, triangle list
    color: tuple
    tool_part: str = ''
    side: str = ''
    jaw_pivot: tuple = ()
    jaw_close_angle: float = 0.


def stl_vertices(data):
    if len(data) < 84:
        raise ValueError('STL 数据不完整')
    count, = struct.unpack_from('<I', data, 80)
    if len(data) != 84 + count * 50 or not count:
        raise ValueError('外观网格不是完整的二进制 STL')
    dtype = np.dtype([('normal', '<f4', (3,)), ('points', '<f4', (3, 3)), ('attribute', '<u2')])
    triangles = np.frombuffer(data, dtype=dtype, offset=84)['points'].copy()
    if not np.isfinite(triangles).all():
        raise ValueError('外观网格含无效坐标')
    normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    keep = lengths > 1e-12
    triangles, normals = triangles[keep], normals[keep] / lengths[keep, None]
    return np.ascontiguousarray(np.concatenate(
        (triangles, np.repeat(normals[:, None, :], 3, axis=1)), axis=2).reshape(-1, 6), dtype=np.float32)


def load_visuals(urdf=URDF, directory=MESHES):
    data = urdf.read_bytes()
    manifest = json.loads((directory/'manifest.json').read_text())
    if hashlib.sha256(data).hexdigest() != manifest['urdf_sha256'] or manifest['urdf_sha256'] != SHA256:
        raise ValueError('外观网格与已校验 URDF 版本不一致')
    visuals = []
    for link in ET.fromstring(data).findall('link'):
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                raise ValueError('厂家模型出现未支持的外观类型')
            name = Path(mesh.get('filename')).name
            raw = (directory/name).read_bytes()
            if hashlib.sha256(raw).hexdigest() != manifest['files'][name]['sha256']:
                raise ValueError('外观网格校验失败：'+name)
            vertices = stl_vertices(raw)
            scale = np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
            if scale.shape != (3,) or not np.isfinite(scale).all() or np.any(scale <= 0):
                raise ValueError('外观网格缩放无效')
            vertices[:, :3] *= scale
            vertices[:, 3:] /= scale
            vertices[:, 3:] /= np.linalg.norm(vertices[:, 3:], axis=1)[:, None]
            color = visual.find('material/color')
            rgba = tuple(map(float, color.get('rgba').split())) if color is not None else (.84, .88, .91, 1.)
            visuals.append(Visual(link.get('name'), origin(visual), vertices, rgba))
    return visuals


def tool_local(data, side, part):
    adapter, gripper = visual_mounts(data, side)
    if part == 'tcp':
        local = np.asarray(flange_from_tip(data, side), dtype=float)
        local[0, 3] -= 106.
    else:
        local = np.asarray(adapter if part == 'adapter' else gripper, dtype=float)
    local[:3, 3] /= 1000.
    return local


def jaw_transform(visual, opening):
    """Display-only approximation. Never changes the fixed control TCP."""
    result = np.eye(4)
    if not visual.jaw_pivot:
        return result
    if not math.isfinite(opening) or not 0 <= opening <= 1:
        raise ValueError('显示夹爪开度无效')
    angle = visual.jaw_close_angle*(1.-opening)
    c, s = math.cos(angle), math.sin(angle)
    turn = np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])
    pivot = np.asarray(visual.jaw_pivot)
    result[:3, :3] = turn
    result[:3, 3] = pivot-turn@pivot
    return result


def load_tool_visuals(data=None, directory=ROOT/'models/taccap'):
    data = load_geometry() if data is None else data
    manifest = json.loads((directory/'manifest.json').read_text())
    if manifest['geometry_id'] != data['geometry_id'] or manifest['units'] != 'mm':
        raise ValueError('夹爪外观与工具配置版本不一致')
    meshes = {}
    for name in ('adapter', 'gripper_body', 'jaw_positive', 'jaw_negative'):
        raw = (directory/(name+'.stl')).read_bytes()
        if hashlib.sha256(raw).hexdigest() != manifest['files'][name+'.stl']['sha256']:
            raise ValueError('夹爪显示网格校验失败：'+name)
        meshes[name] = stl_vertices(raw)
        meshes[name][:, :3] /= 1000.
    # Fixed TCP marker: octahedron, 5 mm radius, in the same transform as IK.
    points = np.array([[0, 0, .005], [0, 0, -.005], [.005, 0, 0], [0, .005, 0],
                       [-.005, 0, 0], [0, -.005, 0]], dtype=np.float32)
    faces = np.array([[0, 2, 3], [0, 3, 4], [0, 4, 5], [0, 5, 2],
                      [1, 3, 2], [1, 4, 3], [1, 5, 4], [1, 2, 5]])
    tri = points[faces]
    normals = np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0])
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    meshes['tcp'] = np.concatenate((tri, np.repeat(normals[:, None, :], 3, axis=1)), axis=2).reshape(-1, 6)
    colors = {'adapter': (.61, .64, .68, 1.), 'tcp': (.25, .95, .55, 1.)}
    visuals = []
    for side in ('left', 'right'):
        for part, vertices in meshes.items():
            motion = manifest['articulation']['jaws'].get(part, {})
            visuals.append(Visual(side+'_waist2_Link', tool_local(data, side, part), vertices,
                colors.get(part, (.35, .39, .44, 1.)), part, side,
                tuple(v/1000 for v in motion.get('pivot_mm', [])),
                math.radians(motion.get('close_angle_deg', 0.))))
    return visuals


def look_at(eye, target):
    eye, target = np.asarray(eye), np.asarray(target)
    forward = target-eye
    forward /= np.linalg.norm(forward)
    side = np.cross(forward, [0., 0., 1.])
    side /= np.linalg.norm(side)
    up = np.cross(side, forward)
    matrix = np.eye(4)
    matrix[:3, :3] = [side, up, -forward]
    matrix[:3, 3] = -matrix[:3, :3] @ eye
    return matrix


def perspective(aspect, fov=36., near=.03, far=30.):
    f = 1 / math.tan(math.radians(fov)/2)
    return np.array([[f/aspect, 0, 0, 0], [0, f, 0, 0],
                     [0, 0, (far+near)/(near-far), 2*far*near/(near-far)], [0, 0, -1, 0]])
