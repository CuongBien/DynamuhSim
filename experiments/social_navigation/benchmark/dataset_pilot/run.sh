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
 live) exec python3 "$TASK_DIR/trial.py" "$@" ;;
 build) exec python3 "$TASK_DIR/build.py" "$@" ;;
 audit) exec python3 "$TASK_DIR/audit.py" "$@" ;;
 test) exec python3 "$TASK_DIR/test_contract.py" "$@" ;;
 *) echo 'Usage: bash run.sh {live|build|audit|test} [arguments]' ;;
esac
