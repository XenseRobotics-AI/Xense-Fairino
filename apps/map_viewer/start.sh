#!/usr/bin/env bash
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ROOT=$(cd -- "$HERE/../.." && pwd)
export PYTHONPATH="$ROOT${PYTHONPATH:+:$PYTHONPATH}"
CHECK='import numpy, pexpect, matplotlib'
GUI=true
for arg in "$@"; do
  case "$arg" in --check|--help|-h) GUI=false ;; esac
done
if [[ "$GUI" == true ]]; then
  CHECK+='; from apps.workbench import gtk_support; from matplotlib.backends.backend_gtk3agg import FigureCanvasGTK3Agg'
fi
for PY in "${MAP_PYTHON:-${FAIRINO_PYTHON:-$ROOT/.venv/bin/python}}" "$ROOT/.local/pico4/env/bin/python" /usr/bin/python3; do
  if [[ -x "$PY" ]] && "$PY" -c "$CHECK" >/dev/null 2>&1; then
    exec "$PY" "$HERE/viewer.py" "$@"
  fi
done
if [[ "$GUI" == true ]]; then
  echo '地图窗口需要 GTK 3、Cairo、matplotlib、numpy 和 pexpect；运行 bash setup.sh console，或用 MAP_PYTHON 指定环境。' >&2
else
  echo '地图只读检查需要 matplotlib、numpy 和 pexpect；可用 MAP_PYTHON 指定环境。' >&2
fi
exit 1
