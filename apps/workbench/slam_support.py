"""Resolve the SLAM branch bundled in this checkout without installing a wheel."""
from pathlib import Path
import sys


def ensure_slam():
    source = Path(__file__).resolve().parents[2] / 'modules/fairino_slam/src'
    if not (source / 'fairino_slam/__init__.py').is_file():
        raise ImportError('缺少内置 SLAM 模块，请更新完整仓库（modules/fairino_slam）')
    path = str(source)
    if path not in sys.path:
        sys.path.insert(0, path)
    return source
