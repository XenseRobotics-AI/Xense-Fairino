#!/usr/bin/env python3
"""Run the bundled SLAM CLI using the toolkit environment."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from apps.workbench.slam_support import ensure_slam

ensure_slam()
from fairino_slam.__main__ import main

if __name__ == '__main__':
    raise SystemExit(main())
