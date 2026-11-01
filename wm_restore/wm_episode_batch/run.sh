#!/usr/bin/env bash
set -euo pipefail
TASK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_ROOT="$(cd "$TASK_DIR/../../../.." && pwd)"
TASK_MODE="${1:-help}"
if (($#)); then shift; fi
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
case "$TASK_MODE" in
 prepare|capture|export)
  set +u; source /opt/ros/jazzy/setup.bash; source "$TASK_ROOT/install/setup.bash"; set -u
  if [[ "$TASK_MODE" == export ]]; then exec python3 "$TASK_DIR/export.py" "$@";
  elif [[ "$TASK_MODE" == prepare ]]; then exec python3 "$TASK_DIR/capture.py" prepare;
  else exec python3 "$TASK_DIR/capture.py" cohort; fi ;;
 test|audit|preview) exec python3 "$TASK_DIR/$TASK_MODE.py" "$@" ;;
 *) echo 'Usage: bash run.sh {prepare|capture|export|test|audit|preview}' ;;
esac
