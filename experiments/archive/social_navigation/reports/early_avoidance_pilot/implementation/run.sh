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
 live) exec python3 "$TASK_DIR/run_validated.py" "$@" ;;
 build) mkdir -p "$TASK_DIR/generated"; g++ -std=c++17 "$TASK_DIR/check_friction.cc" -o "$TASK_DIR/generated/check_friction" $(pkg-config --cflags --libs sdformat14) ;;
 test) exec python3 "$TASK_DIR/test_replay.py" "$@" ;;
 analyze) exec python3 "$TASK_DIR/analyze.py" "$@" ;;
 audit) exec python3 "$TASK_DIR/audit.py" "$@" ;;
 *) echo 'Usage: bash run.sh {build|live|test|analyze|audit} [arguments]' ;;
esac
