#!/usr/bin/env bash
set -euo pipefail
TASK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_MODE="${1:-help}"
if (($#)); then shift; fi
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
case "$TASK_MODE" in
 freeze|replay|evaluate|audit) exec python3 "$TASK_DIR/$TASK_MODE.py" "$@" ;;
 prepare-repeat) exec python3 "$TASK_DIR/prepare_repeat.py" "$@" ;;
 test) exec python3 "$TASK_DIR/test_refinement.py" "$@" ;;
 *) echo 'Usage: bash run.sh {test|freeze|replay|evaluate|audit|prepare-repeat} [arguments]' ;;
esac
