#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../task_runtime.sh"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
case "$TASK_MODE" in
 audit) exec python3 "$TASK_ROOT/tools/audit_wm_dataset.py" "$@" ;;
 test|preview) exec python3 "$TASK_DIR/$TASK_MODE.py" "$@" ;;
 *) echo 'Usage: bash run.sh {test|audit --output NEW_DIRECTORY|preview}' >&2
    echo 'Dataset01 is closed. For new captures, see tools/capture_episode.py --help.' >&2
    exit 2 ;;
esac
