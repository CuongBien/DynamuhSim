#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../task_runtime.sh"
task_ros workspace
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
case "$TASK_MODE" in
 live) exec python3 "$TASK_DIR/capture.py" "$@" ;;
 analyze) exec python3 "$TASK_DIR/analyze.py" "$@" ;;
 test) exec python3 "$TASK_DIR/test_geometry.py" "$@" ;;
 *) echo 'Usage: bash run.sh {live|analyze|test} [arguments]' ;;
esac
