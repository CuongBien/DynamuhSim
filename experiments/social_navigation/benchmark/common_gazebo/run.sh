#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../task_runtime.sh"
case "$TASK_MODE" in
 build)
  task_ros
  task_cmake "$TASK_DIR/build" 2
  ;;
 live)
  task_ros workspace
  export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
  exec python3 "$TASK_DIR/verify_live.py" "$@"
  ;;
 analyze) exec "$TASK_DIR/../../.venv-socnavgym/bin/python" "$TASK_DIR/analyze.py" "$@" ;;
 *) echo 'Usage: bash run.sh {build|live|analyze} [arguments]' >&2; exit 2 ;;
esac
