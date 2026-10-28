#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../task_runtime.sh"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
case "$TASK_MODE" in
 replay|evaluate|audit) exec python3 "$TASK_DIR/$TASK_MODE.py" "$@" ;;
 test) exec python3 "$TASK_DIR/test_refinement.py" "$@" ;;
 *) echo 'Usage: bash run.sh {test|replay|evaluate|audit} [arguments]' >&2; exit 2;;
esac
