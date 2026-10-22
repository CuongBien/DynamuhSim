#!/usr/bin/env bash
set -euo pipefail
TASK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_MODE="${1:-help}"
if (($#)); then shift; fi
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
case "$TASK_MODE" in
 track) exec python3 "$TASK_DIR/track_episode.py" "$@" ;;
 labels) exec python3 "$TASK_DIR/build_labels.py" "$@" ;;
 test) exec python3 "$TASK_DIR/test_contract.py" "$@" ;;
 audit) exec python3 "$TASK_DIR/audit.py" "$@" ;;
 *) echo 'Usage: bash run.sh {track|labels|test|audit} [arguments]' ;;
esac
