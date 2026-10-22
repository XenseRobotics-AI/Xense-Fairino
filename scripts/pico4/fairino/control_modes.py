"""Shared, dependency-free direction conventions for the UI and PICO inputs."""

# Operator forward/left/up -> robot forward/left/up. The facing mode preserves
# room directions for translation; the anatomical mirror preserves depth.
POSITION_SIGNS = {
    'first_person': (1., 1., 1.),
    'mirror': (1., -1., 1.),
    'mirror_same_direction': (-1., -1., 1.),
}

MODE_LABELS = {
    'first_person': '第一人称 · 与机器人同向',
    'mirror': '镜像 · 保持前后方向',
    'mirror_same_direction': '镜像遥操 · 面对机器人',
}

CHASSIS_DIRECTIONS = {
    'first_person': '左杆上推向机器人前方、右推向机器人右方；右杆右拨顺时针旋转',
    'mirror': '左杆前后保持、左右反向；右杆旋转方向反向',
    'mirror_same_direction': '面对机器人：左杆前后、左右平移反向；右杆旋转方向保持',
}


def chassis_directions(mode):
    """Yaw is an axial component: XY reflection reverses its sign."""
    if mode not in POSITION_SIGNS:
        raise ValueError('未知底盘遥操方向模式：'+str(mode))
    x, y, _ = POSITION_SIGNS[mode]
    return x, y, x*y
