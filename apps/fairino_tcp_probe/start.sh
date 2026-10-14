#!/usr/bin/env bash
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PY="${FAIRINO_PYTHON:-$HERE/../../.venv/bin/python}"
if [[ ! -x "$PY" ]]; then PY=/usr/bin/python3; fi
cd -- "$HERE"
exec "$PY" "tcp_keyboard.py" "$@"
