#!/usr/bin/env python3
"""Compatibility entry point for the bundled SLAM branch: mapping.client."""
import importlib
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from apps.workbench.slam_support import ensure_slam

ensure_slam()
_implementation = importlib.import_module('fairino_slam.mapping.client')
if __name__ == '__main__':
    raise SystemExit(_implementation.main())
else:
    # Keep legacy imports and patching of module attributes on the same module.
    sys.modules[__name__] = _implementation
