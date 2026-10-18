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

MESHES = ROOT / 'models/whole_body_meshes'


@dataclass
class Visual:
    link: str
    local: np.ndarray
    vertices: np.ndarray  # xyz + unit normal, triangle list
    color: tuple


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
