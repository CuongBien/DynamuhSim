#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../task_runtime.sh"
task_ros workspace
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
case "$TASK_MODE" in
 audit) exec python3 "$TASK_DIR/audit.py" "$@" ;;
 test) exec python3 "$TASK_DIR/test_failure.py" "$@" ;;
 motion) exec python3 "$TASK_DIR/motion.py" "$@" ;;
 static) exec python3 "$TASK_DIR/static_control.py" "$@" ;;
 analyze) exec python3 "$TASK_DIR/analyze_motion.py" "$@" ;;
 *) echo 'Usage: bash run.sh {audit|test|analyze|static|motion} [arguments]' ;;
esac
