#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../task_runtime.sh"
task_ros workspace
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
case "$TASK_MODE" in
 build)
  task_cmake "$TASK_DIR/generated/build_jazzy" 1 ;;
 live) exec python3 "$TASK_DIR/capture.py" "$@" ;;
 analyze) exec python3 "$TASK_DIR/analyze.py" "$@" ;;
 audit) exec python3 "$TASK_DIR/audit.py" "$@" ;;
 assemble) exec python3 "$TASK_DIR/assemble.py" "$@" ;;
 test) exec python3 "$TASK_DIR/test_fixture.py" "$@" ;;
 *) echo 'Usage: bash run.sh {build|live|assemble|analyze|audit|test} [arguments]' ;;
esac
