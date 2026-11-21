"""Load the unmodified TsenseIV2C-main pipeline for a metric Manus hand observation."""
from pathlib import Path
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
LOCAL = ROOT / ".local/dxhand"
VENDOR = ROOT / "third_party/tsensei_retarget"
sys.path[:0] = [str(VENDOR), str(VENDOR / "src")]

from online_control.online_motion_track.pipeline_v3_2.pipeline import MediaPipeHandPipeline


class Retarget:
    def __init__(self, side):
        self.pipeline = MediaPipeHandPipeline(side, collision_mode="off")

    def reset(self):
        self.pipeline.session.retargeter.reset()

    def solve(self, hand):
        observation = SimpleNamespace(
            source="manus", held_last_valid=False, used_fallback_hand=False,
            expected_label=self.pipeline.expected_label, detected_label=hand["label"],
            world_points=hand["world"], points=hand["world"])
        result = self.pipeline.process(observation)
        if not result.tracking:
            raise ValueError(result.message)
        return result.pose
