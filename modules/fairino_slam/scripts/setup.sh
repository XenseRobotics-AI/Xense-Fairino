#!/usr/bin/env bash
set -euo pipefail
SLAM_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$SLAM_ROOT/scripts/python_env.sh"
case "${1:-cli}" in
  help|-h|--help)
    echo '用法：bash scripts/setup.sh [cli|gui|dev]。优先使用指定 Python 或已激活 Conda，否则创建/复用 .venv；不连接机器人。'
    exit 0 ;;
  cli|gui|dev) SLAM_MODE="${1:-cli}" ;;
  *) echo '用法：bash scripts/setup.sh [cli|gui|dev]' >&2; exit 2 ;;
esac
if [[ -z "${FAIRINO_SLAM_PYTHON:-}" && -z "${CONDA_PREFIX:-}" &&
      ! -e "$SLAM_ROOT/.venv" && ! -L "$SLAM_ROOT/.venv" ]]; then
  SLAM_SYSTEM_PYTHON=$(command -v -- "${FAIRINO_SLAM_SYSTEM_PYTHON:-python3}") || {
    echo '找不到创建 .venv 所需的 Python；可通过 FAIRINO_SLAM_SYSTEM_PYTHON 指定。' >&2
    exit 2
  }
  fairino_validate_python "$SLAM_SYSTEM_PYTHON"
  "$SLAM_SYSTEM_PYTHON" -m venv --system-site-packages "$SLAM_ROOT/.venv"
fi
SLAM_ENV_PYTHON=$(fairino_select_python "$SLAM_ROOT")
echo "安装使用 Python：$SLAM_ENV_PYTHON"
if [[ "$SLAM_MODE" == gui ]]; then
  "$SLAM_ENV_PYTHON" -c 'import gi; gi.require_version("Gtk", "3.0"); gi.require_foreign("cairo")' || {
    echo "此 Python 无法导入 GTK3/gi/Cairo：$SLAM_ENV_PYTHON" >&2
    echo '请为所选环境安装 GUI 依赖，或指定已具备依赖的 Python；Conda 不一定能导入系统 apt 安装的 gi。' >&2
    echo 'Ubuntu 系统 Python 的依赖：sudo apt install python3-venv python3-gi python3-gi-cairo gir1.2-gtk-3.0 fonts-noto-cjk' >&2
    exit 1
  }
fi
SLAM_INSTALL_TARGET="$SLAM_ROOT"
if [[ "$SLAM_MODE" == dev ]]; then SLAM_INSTALL_TARGET="$SLAM_ROOT[dev]"; fi
"$SLAM_ENV_PYTHON" -m pip install -e "$SLAM_INSTALL_TARGET"
echo '安装完成。离线检查：bash scripts/run.sh check；命令帮助：bash scripts/run.sh help'
