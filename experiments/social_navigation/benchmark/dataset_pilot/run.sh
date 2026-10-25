#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../task_runtime.sh"
task_ros workspace
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
case "$TASK_MODE" in
 live) exec python3 "$TASK_DIR/trial.py" "$@" ;;
 build) exec python3 "$TASK_DIR/build.py" "$@" ;;
 audit) exec python3 "$TASK_DIR/audit.py" "$@" ;;
 test) exec python3 "$TASK_DIR/test_contract.py" "$@" ;;
 *) echo 'Usage: bash run.sh {live|build|audit|test} [arguments]' ;;
esac
