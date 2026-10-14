"""Reuse the validated local FAIRINO kinematics and ROS lifecycle."""
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
TCP = Path(os.environ.get("FAIRINO_TCP_DIR", str(ROOT / "apps/fairino_tcp_probe")))
SHARED = TCP.parent / "fairino_upper_probe"
for directory in (TCP, SHARED):
    if not directory.is_dir():
        raise RuntimeError(f"缺少已验证的法奥控制程序：{directory}")
    sys.path.insert(0, str(directory))

