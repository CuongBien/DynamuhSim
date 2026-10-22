#!/usr/bin/env bash
set -euo pipefail
TASK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_ROOT="$(cd "$TASK_DIR/../../../.." && pwd)"
TASK_MODE="${1:-help}"
if (($#)); then shift; fi
set +u
source /opt/ros/jazzy/setup.bash
source "$TASK_ROOT/install/setup.bash"
set -u
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
case "$TASK_MODE" in
 audit) exec python3 "$TASK_DIR/audit.py" "$@" ;;
 test) exec python3 "$TASK_DIR/test_failure.py" "$@" ;;
 motion) exec python3 "$TASK_DIR/motion.py" "$@" ;;
 static) exec python3 "$TASK_DIR/static_control.py" "$@" ;;
 analyze) exec python3 "$TASK_DIR/analyze_motion.py" "$@" ;;
 *) echo 'Usage: bash run.sh {audit|test|analyze|static|motion} [arguments]' ;;
esac
