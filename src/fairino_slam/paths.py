"""Local package resources and writable data; never search the parent toolkit."""
import os
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent
_checkout = PACKAGE_ROOT.parents[1]
_is_checkout = (_checkout / "pyproject.toml").is_file()
PROJECT_ROOT = (_checkout if _is_checkout
                else Path.home() / ".local/share/fairino-slam")
RESOURCE_ROOT = PACKAGE_ROOT / "resources"
# A checkout always uses its editable config, even when it is missing. Never
# silently replace a missing source configuration with bundled defaults.
CONFIG_ROOT = _checkout / "config" if _is_checkout else RESOURCE_ROOT / "config"
ROS_ROOT = _checkout / "ros2" if _is_checkout else RESOURCE_ROOT / "ros2"
GENERATED_ROOT = PROJECT_ROOT / ".local/generated"
LOG_ROOT = Path(os.environ.get("FAIRINO_SLAM_LOG_DIR", str(PROJECT_ROOT / "logs")))
