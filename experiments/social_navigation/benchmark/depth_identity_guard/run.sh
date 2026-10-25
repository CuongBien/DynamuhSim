#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../task_runtime.sh"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
case "$TASK_MODE" in
 test) exec python3 "$TASK_DIR/test_guard.py" "$@" ;;
 seal|freeze|replay|evaluate|holdout|audit|preview) exec python3 "$TASK_DIR/$TASK_MODE.py" "$@" ;;
 preflight|capture44)
  task_ros workspace
  exec python3 "$TASK_DIR/$TASK_MODE.py" "$@" ;;
 capture-audit) exec python3 "$TASK_DIR/capture_audit.py" "$@" ;;
 diagnose) exec python3 "$TASK_DIR/diagnose_case.py" "$@" ;;
 *) echo 'Usage: bash run.sh {test|freeze|seal|replay|evaluate|preflight|capture44|capture-audit|holdout|audit|preview|diagnose} [arguments]' ;;
esac
