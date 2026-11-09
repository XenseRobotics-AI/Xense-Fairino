#!/usr/bin/env bash
set -euo pipefail
SLAM_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$SLAM_ROOT/scripts/python_env.sh"
SLAM_PYTHON=$(fairino_select_python "$SLAM_ROOT")
export PYTHONPATH="$SLAM_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
exec "$SLAM_PYTHON" -m fairino_slam "$@"
