#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../task_runtime.sh"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
case "$TASK_MODE" in
 capture|preflight)
  task_ros workspace
  exec python3 "$TASK_DIR/$TASK_MODE.py" "$@" ;;
 capture-audit) exec python3 "$TASK_DIR/capture_audit.py" "$@" ;;
 compare|audit|preview) exec python3 "$TASK_DIR/$TASK_MODE.py" "$@" ;;
 *) echo 'Usage: bash run.sh {preflight|capture|capture-audit|compare|audit|preview} [arguments]' ;;
esac
