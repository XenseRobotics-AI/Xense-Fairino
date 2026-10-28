"""Compatibility imports for the standalone client's shared continuous input."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'scripts/pico4/fairino'))
from vr_native_input import (MAX_CLUTCH_ROTATION_DEG, NativeClutch,
                             continuous_rotation_vector)

__all__ = ['MAX_CLUTCH_ROTATION_DEG', 'NativeClutch', 'continuous_rotation_vector']
