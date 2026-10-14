#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
PY="${FAIRINO_PYTHON:-$ROOT/.venv/bin/python}"
COMMAND=${1:-help}
if (($#)); then shift; fi
case "$COMMAND" in
  help|-h|--help)
    echo '离线诊断：bash run.sh diagnostics geometry|lidar|wrist|fault-example'
    echo 'geometry：模型中的雷达机械位置；lidar：内置双雷达样例配准；wrist：手腕模型；fault-example：开机故障历史样例。'
    exit 0 ;;
esac
if [[ ! -x "$PY" ]]; then echo '请先执行：bash setup.sh diagnostics' >&2; exit 1; fi
case "$COMMAND" in
  geometry)
    "$PY" "$ROOT/diagnostics/lidar_diagnostics/model_geometry/inspect_base.py"
    exec "$PY" "$ROOT/diagnostics/lidar_diagnostics/model_geometry/plot_lidar_centers.py" "$@" ;;
  lidar)
    "$PY" "$ROOT/diagnostics/lidar_diagnostics/alignment/compare_scans.py" "$@"
    "$PY" "$ROOT/diagnostics/lidar_diagnostics/alignment/fit_relative.py"
    exec "$PY" "$ROOT/diagnostics/lidar_diagnostics/alignment/validate_fit.py" ;;
  wrist) exec "$PY" "$ROOT/apps/fairino_tcp_probe/analysis/plot_wrist.py" "$@" ;;
  fault-example) exec "$PY" -m json.tool "$ROOT/data/diagnostics/startup-fault30-reset.json" ;;
  *) echo "未知离线诊断：$COMMAND" >&2; exit 2 ;;
esac
