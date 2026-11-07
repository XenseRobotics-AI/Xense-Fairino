#!/usr/bin/env bash
set -euo pipefail
CHECK_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
case "${1:-}" in
  -h|--help) echo '用法：bash scripts/check.sh [--package]。离线检查；完整打包检查在临时目录进行。'; exit 0 ;;
  ''|--package) ;;
  *) echo '用法：bash scripts/check.sh [--package]' >&2; exit 2 ;;
esac
source "$CHECK_ROOT/scripts/python_env.sh"
CHECK_PYTHON=$(fairino_select_python "$CHECK_ROOT")
export FAIRINO_SLAM_PYTHON="$CHECK_PYTHON"
export PYTHONDONTWRITEBYTECODE=1
export MPLBACKEND=Agg
CHECK_TEMP=$(mktemp -d)
trap 'rm -rf -- "$CHECK_TEMP"' EXIT
export MPLCONFIGDIR="$CHECK_TEMP/matplotlib"
for check_script in "$CHECK_ROOT"/scripts/*.sh; do
  bash -n "$check_script"
done
bash "$CHECK_ROOT/scripts/check_controller.sh"
bash "$CHECK_ROOT/scripts/run.sh" test
if [[ "${1:-}" == --package ]]; then
  "$CHECK_PYTHON" "$CHECK_ROOT/scripts/check_package.py"
fi
