#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
base_setup() {
  if ! /usr/bin/python3 -c 'import gi, venv; gi.require_version("Gtk", "3.0")' >/dev/null 2>&1 || ! command -v ssh >/dev/null; then
    echo '系统依赖未就绪，请先执行：' >&2
    echo 'sudo apt install python3-venv python3-dev python3-gi gir1.2-gtk-3.0 python3-tk openssh-client curl build-essential cmake unzip libusb-1.0-0 fonts-noto-cjk' >&2
    exit 1
  fi
  if [[ ! -x "$ROOT/.venv/bin/python" ]]; then
    /usr/bin/python3 -m venv --without-pip --system-site-packages "$ROOT/.venv"
  fi
  if ! "$ROOT/.venv/bin/python" -m pip --version >/dev/null 2>&1; then
    if ! "$ROOT/.venv/bin/python" -m ensurepip --upgrade >/dev/null 2>&1; then
      mkdir -p "$ROOT/.local/setup"
      curl -fLsS --connect-timeout 20 --max-time 120 --retry 3 \
        https://bootstrap.pypa.io/get-pip.py -o "$ROOT/.local/setup/get-pip.py"
      "$ROOT/.venv/bin/python" "$ROOT/.local/setup/get-pip.py"
    fi
  fi
  "$ROOT/.venv/bin/python" -m pip install -r "$ROOT/requirements/base.txt"
  "$ROOT/.venv/bin/python" -c 'import gi, pexpect; gi.require_version("Gtk","3.0"); from gi.repository import Gtk; print("基础环境就绪")'
}
case "${1:-all}" in
  base) base_setup ;;
  pico) exec bash "$ROOT/scripts/pico4/setup.sh" ;;
  diagnostics)
    base_setup
    "$ROOT/.venv/bin/python" -m pip install -r "$ROOT/requirements/diagnostics.txt"
    ;;
  all)
    base_setup
    "$ROOT/.venv/bin/python" -m pip install -r "$ROOT/requirements/diagnostics.txt"
    bash "$ROOT/scripts/pico4/setup.sh"
    echo '全部项目环境已准备好。查看功能：bash run.sh help；验证：bash run.sh test'
    ;;
  help|-h|--help)
    echo '用法：bash setup.sh all|base|pico|diagnostics'
    echo '所有环境和下载缓存均创建在本仓库的 .venv/ 与 .local/ 中。'
    ;;
  *) echo '用法：bash setup.sh all|base|pico|diagnostics' >&2; exit 2 ;;
esac
