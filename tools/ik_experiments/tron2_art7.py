"""ART7 model adapter for the vendored TRON2 C++ QP solver.

The original algorithm runs unchanged. Its TRON2 joint names are only aliases
in a generated ART7 DH model. Distances at this boundary are millimetres; the
native Pinocchio model uses metres. No robot SDK or transport is imported.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
import sysconfig
import xml.etree.ElementTree as ET

import numpy as np

from tcp_core import ALPHA, BOUNDS_DEG, D_MM, FLANGE_OFFSET_MM, THETA, check_arm_limits
from tool_frames import for_arm, validate_runtime

ROOT = Path(__file__).resolve().parents[2]
VENDOR = ROOT / 'third_party/tron2_ik'
CACHE = ROOT / '.local/tron2-ik'
NAMES = ('proximal_pitch', 'proximal_roll', 'proximal_yaw', 'elbow',
         'wrist_yaw', 'wrist_pitch', 'wrist_roll')


def source_digest():
    """A native build cannot silently outlive a source or Python ABI change."""
    digest = hashlib.sha256(sysconfig.get_config_var('SOABI').encode())
    for path in sorted((VENDOR / 'cpp').rglob('*')):
        if path.is_file():
            digest.update(path.relative_to(VENDOR).as_posix().encode())
            digest.update(path.read_bytes())
    digest.update((VENDOR / 'setup.py').read_bytes())
    return digest.hexdigest()[:20]


def _load_api():
    extension_dir = CACHE / 'builds' / source_digest() / 'tron2_ik'
    suffix = sysconfig.get_config_var('EXT_SUFFIX')
    if not (extension_dir / ('_native' + suffix)).is_file():
        raise RuntimeError('TRON2 IK 尚未编译；请运行 .local/pico4/env/bin/python '
                           'tools/ik_experiments/setup_tron2_ik.py --install')
    # Append optional dependencies so they never replace the existing PICO
    # NumPy. We use only Pinocchio's C++ libraries, not its Python bindings.
    dependencies = str(CACHE / 'site-packages')
    if dependencies not in sys.path:
        sys.path.append(dependencies)
    source = str(VENDOR / 'src')
    if source not in sys.path:
        sys.path.insert(0, source)
    import tron2_ik
    if Path(tron2_ik.__file__).resolve().parent != VENDOR / 'src/tron2_ik':
        raise RuntimeError('已加载的 tron2_ik 与本项目固定源码不一致')
    if str(extension_dir) not in tron2_ik.__path__:
        tron2_ik.__path__.append(str(extension_dir))
    return tron2_ik


def dependencies_available():
    """Only absence is optional; corrupt/ABI-incompatible installs must fail."""
    suffix = sysconfig.get_config_var('EXT_SUFFIX')
    extension = CACHE / 'builds' / source_digest() / 'tron2_ik' / ('_native'+suffix)
    return extension.is_file() and (CACHE/'site-packages/cmeel.prefix/lib').is_dir()


def _numbers(values):
    return ' '.join(format(float(value), '.17g') for value in values)


def _rpy(rotation):
    pitch = math.atan2(-rotation[2, 0], math.hypot(rotation[0, 0], rotation[1, 0]))
    if abs(math.cos(pitch)) < 1e-10:
        return (0., pitch, math.atan2(-rotation[0, 1], rotation[1, 1]))
    return (math.atan2(rotation[2, 1], rotation[2, 2]), pitch,
            math.atan2(rotation[1, 0], rotation[0, 0]))


def model_xml(tools):
    """Build two independent arm-base DH trees, without TRON2 geometry."""
    tools = validate_runtime(tools)
    robot = ET.Element('robot', name='ART7_for_TRON2_Demo')
    ET.SubElement(robot, 'link', name='base_Link')

    def joint(name, parent, child, xyz, rpy, bounds=None):
        ET.SubElement(robot, 'link', name=child)
        element = ET.SubElement(robot, 'joint', name=name,
                                type='fixed' if bounds is None else 'revolute')
        ET.SubElement(element, 'parent', link=parent)
        ET.SubElement(element, 'child', link=child)
        ET.SubElement(element, 'origin', xyz=_numbers(xyz), rpy=_numbers(rpy))
        if bounds is not None:
            ET.SubElement(element, 'axis', xyz='0 0 1')
            # These high velocity values leave the upstream *iteration* trust
            # box active. They are not actuator limits or real motion rates.
            lo, hi = map(math.radians, bounds)
            ET.SubElement(element, 'limit', lower=str(lo+1e-12), upper=str(hi-1e-12),
                          effort='1', velocity='100')

    for side, letter in (('left', 'L'), ('right', 'R')):
        parent = 'base_Link'
        for name, d, alpha, theta, bounds in zip(NAMES, D_MM, ALPHA, THETA, BOUNDS_DEG):
            rotating = f'{name}_{letter}_rotation'
            link = f'{name}_{letter}_Link'
            # Standard DH = Tz(d) Rz(theta+q) Rx(alpha).
            joint(f'{name}_{letter}_Joint', parent, rotating,
                  (0., 0., d/1000.), (0., 0., theta), bounds)
            joint(f'{name}_{letter}_dh', rotating, link, (0., 0., 0.), (alpha, 0., 0.))
            parent = link
        tool = np.asarray(for_arm(tools, side), dtype=float)
        offset = (np.asarray(FLANGE_OFFSET_MM)+tool[:3, 3])/1000.
        joint(f'grasper_base_{letter}_fixed', parent, f'grasper_base_{letter}_Link',
              offset, _rpy(tool[:3, :3]))
    return ET.tostring(robot, encoding='utf-8', xml_declaration=True)


def _target(value):
    result = np.asarray(value, dtype=float).copy()
    if result.shape != (4, 4) or not np.isfinite(result).all():
        raise ValueError('TRON2 target must be a finite 4×4 transform')
    if (not np.allclose(result[3], [0., 0., 0., 1.], atol=1e-9)
            or not np.allclose(result[:3, :3].T @ result[:3, :3], np.eye(3), atol=1e-7)
            or abs(np.linalg.det(result[:3, :3])-1.) > 1e-7):
        raise ValueError('TRON2 target must be a rigid transform')
    result[:3, 3] /= 1000.
    return result


def _angles(value):
    array = np.asarray(value)
    if array.shape != (7,) or array.dtype.kind not in 'fi':
        raise ValueError('TRON2 joint reference must contain seven numeric angles')
    return check_arm_limits(array.astype(float).tolist())


class Art7TronIK:
    """True upstream dual-arm QP with current ART7 dimensions and tools.

    Coupled wrist/path protection is deliberately owned by the PICO packet
    adapter. The upstream QP has individual joint bounds only. The original
    TRON2 center-column approximation is inapplicable and explicitly disabled.
    """
    def __init__(self, tools):
        self.tools = json.loads(json.dumps(validate_runtime(tools), allow_nan=False))
        api = _load_api()
        xml = model_xml(self.tools)
        directory = CACHE / 'models'
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / ('art7-'+hashlib.sha256(xml).hexdigest()[:20]+'.urdf')
        if not path.exists():
            path.write_bytes(xml)
        self.model = api.Tron2QPModel(path, left_frame='grasper_base_L_Link',
                                    right_frame='grasper_base_R_Link')
        self.options = api.IKOptions(column_avoidance=False, max_iterations=120,
            position_tolerance=.00002, orientation_tolerance=math.radians(.01))
        # Resolve shared libraries once, before the control loop. The public
        # upstream facade repeats filesystem discovery and CDLL loading for
        # every FK and IK call. Use exactly its native model and conversion
        # contract below, with all Python/C++ validation still in force.
        from tron2_ik.solver import _native_module
        self._api = api
        self._native_type = _native_module().Options

    def _native_options(self):
        """Upstream option conversion without repeated library discovery.

        Validate every invocation: a caller changing a public option must not
        get an old cached value or bypass the upstream Python-only checks.
        """
        options = self.options
        if not isinstance(options, self._api.IKOptions):
            raise TypeError('options must be IKOptions')
        options.validate()
        native = self._native_type()
        for name in options.__dataclass_fields__:
            if not hasattr(native, name):
                continue
            value = getattr(options, name)
            if name == 'posture_target':
                value = [] if value is None else np.asarray(value, dtype=float).reshape(-1).tolist()
            elif name in ('position_tolerance', 'orientation_tolerance'):
                value = 0. if value is None else value
            setattr(native, name, value)
        native.damping = max(options.damping, options.lm_damping)
        return native

    def _forward(self, side, q, options):
        # q has passed _angles/check_arm_limits at each public call boundary;
        # the original C++ FK validates all options and q coordinates again.
        full = q+[0.]*7 if side == 'left' else [0.]*7+q
        try:
            result = np.asarray(self.model._native.forward_kinematics(full, side, options)).reshape(4, 4)
        except ValueError as exc:
            raise self._api.Tron2ConfigurationError(str(exc)) from exc
        result[:3, 3] *= 1000.
        return result

    def forward(self, side, q):
        if side not in ('left', 'right'):
            raise ValueError('Unknown arm side')
        q = _angles(q)
        return self._forward(side, q, self._native_options())

    def solve(self, goals, references):
        if not goals or set(goals)-{'left', 'right'}:
            raise ValueError('TRON2 requires one or both arm goals')
        refs = {side: _angles(references[side]) for side in ('left', 'right')}
        seed = refs['left']+refs['right']
        options = self._native_options()
        targets = {side: _target(goals[side]) if side in goals else
                   _target(self._forward(side, refs[side], options)) for side in refs}
        try:
            result = self.model._native.solve_ik(targets['left'].reshape(-1).tolist(),
                targets['right'].reshape(-1).tolist(), seed, options)
        except ValueError as exc:
            raise self._api.Tron2ConfigurationError(str(exc)) from exc
        result_q = result.q
        entries = {}
        for side, begin in (('left', 0), ('right', 7)):
            if side not in goals:
                continue
            q = check_arm_limits(result_q[begin:begin+7])
            entries[side] = dict(target=q, tcp=self._forward(side, q, options), success=result.success,
                iterations=result.iterations, status=result.status,
                elapsed_ms=result.solve_time_ms, qp_iterations=result.qp_iterations,
                backend='tron2_cpp_eiquadprog_art7')
        return entries
