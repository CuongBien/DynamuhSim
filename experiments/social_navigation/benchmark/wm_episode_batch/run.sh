#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../task_runtime.sh"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
case "$TASK_MODE" in
 prepare|capture|export)
  task_ros workspace
  if [[ "$TASK_MODE" == export ]]; then exec python3 "$TASK_DIR/export.py" "$@";
  elif [[ "$TASK_MODE" == prepare ]]; then exec python3 "$TASK_DIR/capture.py" prepare;
  else exec python3 "$TASK_DIR/capture.py" cohort; fi ;;
 test|audit|preview) exec python3 "$TASK_DIR/$TASK_MODE.py" "$@" ;;
 *) echo 'Usage: bash run.sh {prepare|capture|export|test|audit|preview}' ;;
esac
