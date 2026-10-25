#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../task_runtime.sh"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
case "$TASK_MODE" in
 prepare) exec python3 "$TASK_DIR/prepare.py" "$@" ;;
 finalize) exec python3 "$TASK_DIR/finalize.py" "$@" ;;
 smoke) exec python3 -I "$TASK_DIR/smoke.py" "$@" ;;
 build|audit)
  task_ros workspace
  exec python3 "$TASK_DIR/$TASK_MODE.py" "$@" ;;
 *) echo 'Usage: bash run.sh {prepare|build|finalize|audit|smoke} [arguments]' ;;
esac
