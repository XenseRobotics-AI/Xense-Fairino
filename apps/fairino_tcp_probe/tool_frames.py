"""Shared CAD mounting and fixed TCP transforms. Pure Python on both hosts."""
import hashlib
import json
import math
from pathlib import Path

if globals().get('__package__'):
    from .tcp_core import ARMS, FLANGE_OFFSET_MM, identity, multiply, rotation, finite_vector
else:
    from tcp_core import ARMS, FLANGE_OFFSET_MM, identity, multiply, rotation, finite_vector

GEOMETRY_ID = 'taccap-step-20261020-v1'
DEFAULT_CONFIG = (Path(__file__).resolve().parents[2]/'models/taccap/tool_config.json'
                  if '__file__' in globals() else None)


def translate(xyz):
    result = identity()
    for i, value in enumerate(xyz):
        result[i][3] = value
    return result


def inverse_rigid(matrix):
    result = identity()
    for i in range(3):
        for j in range(3):
            result[i][j] = matrix[j][i]
        result[i][3] = -sum(matrix[j][i]*matrix[j][3] for j in range(3))
    return result


def validate_geometry(data, require_confirmed=False):
    if not isinstance(data, dict) or data.get('geometry_id') != GEOMETRY_ID or data.get('version') != 1:
        raise ValueError('夹爪工具配置版本不匹配')
    tip = finite_vector(data.get('tip_cad_mm'), 3, 'CAD 固定指尖中心')
    if not (-25 <= tip[0] <= 25 and -20 <= tip[1] <= 60 and -230 <= tip[2] <= -80):
        raise ValueError('夹爪 TCP 超出该 CAD 的合理工具范围')
    arms = data.get('arms')
    if not isinstance(arms, dict) or set(arms) != set(ARMS):
        raise ValueError('工具配置必须包含左右臂')
    for side, spec in arms.items():
        if not isinstance(spec, dict) or type(spec.get('installed')) is not bool or type(spec.get('confirmed')) is not bool:
            raise ValueError('夹爪安装状态必须明确')
        roll = spec.get('roll_deg')
        if type(roll) not in (int, float) or not math.isfinite(roll) or not -180 <= roll <= 180:
            raise ValueError('夹爪安装角度必须在 -180～180°')
        if require_confirmed and spec['installed'] and not spec['confirmed']:
            raise ValueError(f'{side} 夹爪安装角度 / 固定 TCP 尚未核对；请先在工具设置中核对并保存')
    if require_confirmed and not any(v['installed'] for v in arms.values()):
        raise ValueError('夹爪 TCP 模式至少需要安装一侧夹爪')
    return json.loads(json.dumps(data, allow_nan=False))


def load_geometry(path=DEFAULT_CONFIG, require_confirmed=False):
    if path is None:
        raise ValueError('远端必须使用启动时提供的工具配置')
    return validate_geometry(json.loads(Path(path).read_text()), require_confirmed)


def flange_from_adapter(roll_deg=0.):
    # CAD adapter +Y is the outward axis. At zero roll, CAD +X is flange +Y.
    # Positive roll follows the right-hand rule around outward (-flange X).
    return multiply(rotation(0, -math.radians(roll_deg)), rotation(2, math.pi/2))


def adapter_from_gripper():
    # Ø34 gripper boss fits the Ø34.4 x 8 mm adapter recess.
    # Gripper seating face: Z=38.5, centre X=0/Y=-15.75 mm.
    # Adapter seating face: Y=13; boss extends down to adapter Y=5.
    return multiply(translate((0., 51.5, 15.75)), rotation(0, math.pi/2))


def visual_mounts(data, side):
    validate_geometry(data)
    flange = translate(FLANGE_OFFSET_MM)
    adapter = multiply(flange, flange_from_adapter(data['arms'][side]['roll_deg']))
    return adapter, multiply(adapter, adapter_from_gripper())


def flange_from_tip(data, side):
    # Keep the established orientation axes; move the rotation pivot to the
    # chosen fixed jaw-tip centre. Jaw opening never changes this transform.
    mount = multiply(flange_from_adapter(data['arms'][side]['roll_deg']), adapter_from_gripper())
    point = multiply(mount, translate(data['tip_cad_mm']))
    return translate([point[i][3] for i in range(3)])


def tool_config(mode='flange', geometry=None, require_confirmed=False):
    if mode not in ('flange', 'taccap'):
        raise ValueError('未知 TCP 工具类型')
    if mode == 'flange':
        return {'mode': 'flange', 'id': 'flange', 'transforms': {s: identity() for s in ARMS}}
    data = validate_geometry(geometry if geometry is not None else load_geometry(), require_confirmed)
    transforms = {s: flange_from_tip(data, s) if data['arms'][s]['installed'] else identity() for s in ARMS}
    identity_data = dict(geometry_id=GEOMETRY_ID, transforms=transforms)
    digest = hashlib.sha256(json.dumps(identity_data, sort_keys=True, allow_nan=False).encode()).hexdigest()
    return dict(mode='taccap', id=GEOMETRY_ID+':'+digest, transforms=transforms)


def validate_runtime(config):
    if not isinstance(config, dict) or config.get('mode') not in ('flange', 'taccap'):
        raise ValueError('无效 TCP 工具配置')
    transforms = config.get('transforms')
    if not isinstance(transforms, dict) or set(transforms) != set(ARMS):
        raise ValueError('TCP 工具配置缺少左右臂')
    for matrix in transforms.values():
        if not isinstance(matrix, list) or len(matrix) != 4:
            raise ValueError('无效 TCP 工具矩阵')
        for row in matrix:
            finite_vector(row, 4, '工具矩阵')
        for i in range(4):
            for j in range(4):
                if (j != 3 or i == 3) and abs(matrix[i][j]-(i == j)) > 1e-9:
                    raise ValueError('此工具配置仅支持固定平移 TCP')
        if sum(matrix[i][3]**2 for i in range(3)) > 350**2:
            raise ValueError('TCP 工具偏置超过 350 mm')
    if config['mode'] == 'flange':
        if config != tool_config():
            raise ValueError('法兰模式不能携带夹爪偏置')
    else:
        digest = hashlib.sha256(json.dumps(dict(geometry_id=GEOMETRY_ID, transforms=transforms),
            sort_keys=True, allow_nan=False).encode()).hexdigest()
        if config.get('id') != GEOMETRY_ID+':'+digest:
            raise ValueError('TCP 工具配置指纹不匹配')
    return config


def for_arm(config, side=None):
    config = validate_runtime(config)
    if side is None:
        if config['transforms']['left'] != config['transforms']['right']:
            raise ValueError('左右 TCP 不同，必须指定手臂')
        side = 'left'
    if side not in ARMS:
        raise ValueError('未知手臂')
    return config['transforms'][side]


def packet_tool_id(kin):
    return getattr(kin, 'tools', tool_config())['id']
