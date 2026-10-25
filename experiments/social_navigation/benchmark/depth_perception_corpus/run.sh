#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../task_runtime.sh"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
case "$TASK_MODE" in
 build|capture)
  task_ros workspace
  if [[ "$TASK_MODE" == build ]]; then task_cmake "$TASK_DIR/generated/build" 1
  else exec python3 "$TASK_DIR/capture.py" "$@"; fi ;;
 test|export|audit|preview) exec python3 "$TASK_DIR/$TASK_MODE.py" "$@" ;;
 *) echo 'Usage: bash run.sh {build|capture|test|export|audit|preview}' ;;
esac
