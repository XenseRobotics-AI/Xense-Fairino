#!/usr/bin/env bash
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
MAP_GUI_ROOT="$HERE/../../.local/map-gui/root"
if [[ -d "$MAP_GUI_ROOT/usr/lib/python3.12/tkinter" ]]; then
  export PYTHONPATH="$MAP_GUI_ROOT/usr/lib/python3.12:$MAP_GUI_ROOT/usr/lib/python3.12/lib-dynload${PYTHONPATH:+:$PYTHONPATH}"
  export LD_LIBRARY_PATH="$MAP_GUI_ROOT/usr/lib:$MAP_GUI_ROOT/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  export TK_LIBRARY="$MAP_GUI_ROOT/usr/share/tcltk/tk8.6"
fi
for PY in "${MAP_PYTHON:-${FAIRINO_PYTHON:-$HERE/../../.venv/bin/python}}" "$HERE/../../.local/pico4/env/bin/python" /usr/bin/python3; do
  if [[ -x "$PY" ]] && "$PY" -c 'import tkinter,matplotlib,numpy,pexpect; from PIL import ImageTk' >/dev/null 2>&1; then
    exec "$PY" "$HERE/viewer.py" "$@"
  fi
done
echo '地图窗口需要 Tk、matplotlib、numpy、Pillow/ImageTk 和 pexpect；可用 MAP_PYTHON 指定环境。' >&2
exit 1
