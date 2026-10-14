#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
case "${1:-base}" in
  base)
    if ! /usr/bin/python3 -c 'import gi, tkinter, venv' >/dev/null 2>&1 || ! command -v ssh >/dev/null; then
      echo '请先安装系统依赖：sudo apt install python3-venv python3-dev python3-gi gir1.2-gtk-3.0 python3-tk openssh-client' >&2
      exit 1
    fi
    /usr/bin/python3 -m venv --system-site-packages "$ROOT/.venv"
    "$ROOT/.venv/bin/python" -m pip install 'pexpect==4.9.0'
    "$ROOT/.venv/bin/python" -c 'import gi, tkinter, pexpect; gi.require_version("Gtk","3.0"); from gi.repository import Gtk; print("基础依赖正常")'
    ;;
  pico) exec bash "$ROOT/scripts/pico4/setup.sh" ;;
  *) echo '用法：bash setup.sh base|pico' >&2; exit 2 ;;
esac
