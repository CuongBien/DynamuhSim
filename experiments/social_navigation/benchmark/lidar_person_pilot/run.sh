#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../task_runtime.sh"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export CUDA_CACHE_PATH="$TASK_DIR/generated/cuda_cache" XDG_CACHE_HOME="$TASK_DIR/generated/cache"
case "$TASK_MODE" in
 capture)
  task_ros workspace
  exec python3 "$TASK_DIR/capture.py" "$@" ;;
 infer|smoke) exec "$TASK_DIR/generated/venv/bin/python" "$TASK_DIR/$TASK_MODE.py" "$@" ;;
 test|evaluate|audit) exec python3 "$TASK_DIR/$TASK_MODE.py" "$@" ;;
 *) echo 'Usage: bash run.sh {capture|infer|smoke|test|evaluate|audit}' ;;
esac
