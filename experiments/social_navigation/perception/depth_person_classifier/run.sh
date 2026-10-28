#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../task_runtime.sh"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export CUBLAS_WORKSPACE_CONFIG=:4096:8
export CUDA_CACHE_PATH="$TASK_DIR/generated/cuda_cache" XDG_CACHE_HOME="$TASK_DIR/generated/cache"
TASK_PYTHON="$TASK_DIR/../lidar_person_pilot/generated/venv/bin/python"
case "$TASK_MODE" in
 train|evaluate|test) exec "$TASK_PYTHON" "$TASK_DIR/$TASK_MODE.py" "$@" ;;
 *) echo 'Usage: bash run.sh {train|evaluate|test}' ;;
esac
