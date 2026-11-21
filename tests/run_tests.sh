#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
PY="${FAIRINO_PYTHON:-$ROOT/.venv/bin/python}"
if [[ ! -x "$PY" ]]; then PY=/usr/bin/python3; fi
export FAIRINO_TCP_DIR="$ROOT/apps/fairino_tcp_probe"
export PYTHONDONTWRITEBYTECODE=1
for name in fairino_keyboard fairino_upper_probe fairino_upper_probe/body_control fairino_tcp_probe map_viewer navigation workbench gripper_control; do
  (cd "$ROOT/apps/$name" && "$PY" -m unittest discover -p 'test_*.py')
done
PICO_PY="${PICO_TEST_PYTHON:-$ROOT/.local/pico4/env/bin/python}"
if [[ ! -x "$PICO_PY" ]]; then echo '基础测试通过；PICO 未安装，完整测试尚未完成。' >&2; exit 2; fi
export PYTHONPATH="$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
export XENSE_LOG_DIR="${XENSE_LOG_DIR:-$ROOT/.local/pico4/logs}"
(cd "$ROOT/scripts/pico4/fairino" && "$PICO_PY" -m unittest discover -p 'test_*.py')
(cd "$ROOT/scripts/pico4" && "$PICO_PY" -m unittest test_pico)
FAIRINO_SLAM_PYTHON="$PY" bash "$ROOT/modules/fairino_slam/scripts/check.sh"
DXHAND_PY="$ROOT/.local/dxhand/env/bin/python"
if [[ ! -x "$DXHAND_PY" ]]; then echo '其余测试通过；达显环境未安装（bash setup.sh dxhand），完整测试尚未完成。' >&2; exit 2; fi
"$DXHAND_PY" -m unittest discover -s "$ROOT/tools/dxhand" -p 'test_*.py' -v
