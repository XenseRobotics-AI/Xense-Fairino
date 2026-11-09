"""Console compatibility imports for the bundled SLAM navigation runtime.

Keep one implementation for target binding, heading policy and chassis sessions.
"""
from apps.workbench.slam_support import ensure_slam
ensure_slam()
from fairino_slam.navigation.desktop import (  # noqa: E402,F401
    Target, DriveSession, active_goal, context, execute, navigation_status,
)
from fairino_slam.navigation.client import configured_heading_mode  # noqa: E402,F401
