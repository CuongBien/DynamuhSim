#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../task_runtime.sh"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export CUDA_CACHE_PATH="$TASK_DIR/generated/cuda_cache"
case "$TASK_MODE" in
 capture)
  task_ros workspace
  exec python3 "$TASK_DIR/capture.py" "$@" ;;
 infer|test|audit) TASK_SCRIPT="$TASK_MODE"; [[ "$TASK_MODE" == infer ]] && TASK_SCRIPT=surface_runner
  exec "$TASK_DIR/../lidar_person_pilot/generated/venv/bin/python" "$TASK_DIR/$TASK_SCRIPT.py" "$@" ;;
 *) echo 'Usage: bash run.sh {capture|infer|test|audit}' ;;
esac
