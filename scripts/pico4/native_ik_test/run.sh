#!/usr/bin/env bash
set -euo pipefail
TEST_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
TEST_PYTHON="$TEST_ROOT/.local/pico4/env/bin/python"
if [[ ! -x "$TEST_PYTHON" ]]; then
    echo "缺少 PICO 环境，请先运行：bash $TEST_ROOT/scripts/pico4/run.sh setup" >&2
    exit 1
fi
export PYTHONPATH="$TEST_ROOT/src:$TEST_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export XENSE_LOG_DIR="$TEST_ROOT/.local/pico4/logs"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
mkdir -p "$XENSE_LOG_DIR"
exec "$TEST_PYTHON" "$TEST_ROOT/scripts/pico4/native_ik_test/client.py" "$@"
