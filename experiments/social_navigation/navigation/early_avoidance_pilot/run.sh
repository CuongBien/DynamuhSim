#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../task_runtime.sh"
task_ros workspace
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
case "$TASK_MODE" in
 build) mkdir -p "$TASK_DIR/generated"; g++ -std=c++17 "$TASK_DIR/check_friction.cc" -o "$TASK_DIR/generated/check_friction" $(pkg-config --cflags --libs sdformat14) ;;
 test) exec python3 "$TASK_DIR/test_replay.py" "$@" ;;
 analyze) exec python3 "$TASK_DIR/analyze.py" "$@" ;;
 audit) exec python3 "$TASK_DIR/audit.py" "$@" ;;
 *) echo 'Usage: bash run.sh {build|test|analyze|audit} [arguments]' >&2; exit 2;;
esac
