#!/usr/bin/env bash
set -euo pipefail
PICO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
PICO_RUNTIME="$PICO_ROOT/.local/pico4"
export PYTHONPATH="$PICO_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
export XENSE_LOG_DIR="$PICO_RUNTIME/logs"
mkdir -p "$XENSE_LOG_DIR"
if [[ "${1:-}" == setup ]]; then
    shift
    exec bash "$PICO_ROOT/scripts/pico4/setup.sh" "$@"
fi
if [[ ! -x "$PICO_RUNTIME/env/bin/python" ]]; then
    echo "请先运行：bash $PICO_ROOT/scripts/pico4/run.sh setup" >&2
    exit 1
fi
if [[ "${1:-}" == fairino ]]; then
    shift
    exec "$PICO_RUNTIME/env/bin/python" "$PICO_ROOT/scripts/pico4/fairino/vr_client.py" "$@"
fi
if [[ "${1:-}" == latency ]]; then
    shift
    exec "$PICO_RUNTIME/env/bin/python" "$PICO_ROOT/scripts/pico4/fairino/pico_latency.py" "$@"
fi
exec "$PICO_RUNTIME/env/bin/python" "$PICO_ROOT/scripts/pico4/pico.py" "$@"
