#!/usr/bin/env bash
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# Prefer the basic environment; the packaged PICO Conda env also provides Tk.
PY="${LIDAR_PYTHON:-${FAIRINO_PYTHON:-$HERE/../../.venv/bin/python}}"
if [[ ! -x "$PY" ]] || ! "$PY" -c 'import tkinter,pexpect' >/dev/null 2>&1; then
  PY="$HERE/../../.local/pico4/env/bin/python"
fi
if [[ ! -x "$PY" ]] || ! "$PY" -c 'import tkinter,pexpect' >/dev/null 2>&1; then
  PY=/usr/bin/python3
fi
if ! "$PY" -c 'import tkinter,pexpect' >/dev/null 2>&1; then
  echo '雷达显示需要 Tk 和 pexpect；运行 bash setup.sh base，或设置 LIDAR_PYTHON。' >&2; exit 1
fi
cd -- "$HERE"
exec "$PY" viewer.py "$@"
