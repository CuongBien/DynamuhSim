#!/usr/bin/env bash
set -euo pipefail
TASK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_ROOT="$(cd "$TASK_DIR/../../../.." && pwd)"
TASK_MODE="${1:-help}"
if (($#)); then shift; fi
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export CUDA_CACHE_PATH="$TASK_DIR/generated/cuda_cache"
case "$TASK_MODE" in
 capture)
  set +u; source /opt/ros/jazzy/setup.bash; source "$TASK_ROOT/install/setup.bash"; set -u
  exec python3 "$TASK_DIR/capture.py" "$@" ;;
 infer|test|audit) TASK_SCRIPT="$TASK_MODE"; [[ "$TASK_MODE" == infer ]] && TASK_SCRIPT=surface_runner
  exec "$TASK_DIR/../lidar_person_pilot/generated/venv/bin/python" "$TASK_DIR/$TASK_SCRIPT.py" "$@" ;;
 *) echo 'Usage: bash run.sh {capture|infer|test|audit}' ;;
esac
