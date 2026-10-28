#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../task_runtime.sh"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
case "$TASK_MODE" in
 test) exec python3 "$TASK_DIR/test_guard.py" "$@" ;;
 replay|evaluate|audit) exec python3 "$TASK_DIR/$TASK_MODE.py" "$@" ;;
 capture-audit) exec python3 "$TASK_DIR/capture_audit.py" "$@" ;;
 *) echo 'Usage: bash run.sh {test|replay|evaluate|capture-audit|audit} [arguments]' >&2; exit 2;;
esac
