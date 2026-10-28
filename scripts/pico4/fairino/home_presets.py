"""Home preset paths shared by the standalone PICO client and desktop."""
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DEFAULT_HOME = ROOT/'home_pose.json'
HOME_PRESETS = {
    'default': ('起始位姿 1 · 原对称屈肘', DEFAULT_HOME),
    'right_mirrored': ('起始位姿 2 · 10-20 右臂实测镜像', ROOT/'home_pose_right_mirrored.json'),
    'left_cross_body': ('起始位姿 3 · 左爪身前朝右（腕部余量小）', ROOT/'home_pose_left_cross_body.json'),
}


def resolve_home(preset='default', path=None):
    if preset not in HOME_PRESETS:
        raise ValueError('未知起始位姿：'+str(preset))
    return Path(path) if path is not None else HOME_PRESETS[preset][1]
