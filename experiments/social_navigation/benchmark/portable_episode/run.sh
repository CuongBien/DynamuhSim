#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../task_runtime.sh"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
case "$TASK_MODE" in
 smoke) exec python3 -I "$TASK_DIR/smoke.py" "$@" ;;
 audit)
  task_ros
  exec python3 "$TASK_DIR/$TASK_MODE.py" "$@" ;;
 *) echo 'Usage: bash run.sh {audit|smoke} [arguments]' >&2; exit 2 ;;
esac
