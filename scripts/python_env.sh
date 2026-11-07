#!/usr/bin/env bash
# Shared interpreter selection for setup/run/check. Source this file.
fairino_validate_python() {
  local candidate="$1"
  if [[ ! -f "$candidate" || ! -x "$candidate" ]]; then
    echo "Python 不存在或不可执行：$candidate" >&2
    return 2
  fi
  if ! "$candidate" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else "需要 Python 3.10 或更新版本")'; then
    echo "Python 无法运行或版本不符合要求：$candidate" >&2
    echo '请激活可用的 Conda 环境，或通过 FAIRINO_SLAM_PYTHON 指定解释器。' >&2
    return 2
  fi
}

fairino_select_python() {
  local project_root="$1"
  local candidate
  if [[ -n "${FAIRINO_SLAM_PYTHON:-}" ]]; then
    candidate=$(command -v -- "$FAIRINO_SLAM_PYTHON") || {
      echo "FAIRINO_SLAM_PYTHON 不可用：$FAIRINO_SLAM_PYTHON" >&2
      return 2
    }
  elif [[ -n "${CONDA_PREFIX:-}" ]]; then
    candidate="$CONDA_PREFIX/bin/python"
  elif [[ -e "$project_root/.venv" || -L "$project_root/.venv" ]]; then
    candidate="$project_root/.venv/bin/python"
    if [[ ! -f "$candidate" || ! -x "$candidate" ]]; then
      echo "工程 .venv 不完整或不可执行：$project_root/.venv" >&2
      echo '请激活 Conda，或备份移走该目录后重新安装；脚本不会覆盖它。' >&2
      return 2
    fi
  else
    candidate=$(command -v python3) || {
      echo '找不到 Python 3；请激活 Conda 或指定 FAIRINO_SLAM_PYTHON。' >&2
      return 2
    }
  fi
  fairino_validate_python "$candidate" || return "$?"
  printf '%s\n' "$candidate"
}
