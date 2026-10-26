#!/usr/bin/env bash
set -euo pipefail
TASK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_MODE="${1:-}"
if (($#)); then shift; fi
TASK_IMAGE=dynamuhsim-complexitynav:round3
case "$TASK_MODE" in
  build)
    if ! docker image inspect dynamuhsim-crowdsimplus:round2 >/dev/null 2>&1; then
      bash "$TASK_DIR/../round2/run.sh" build
    fi
    exec docker build -t "$TASK_IMAGE" "$TASK_DIR"
    ;;
  *) echo 'Usage: bash run.sh build' >&2; exit 2;;
esac
