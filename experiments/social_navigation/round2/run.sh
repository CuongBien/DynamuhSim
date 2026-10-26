#!/usr/bin/env bash
set -euo pipefail
TASK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_MODE="${1:-}"
if (($#)); then shift; fi
TASK_IMAGE=dynamuhsim-crowdsimplus:round2
case "$TASK_MODE" in
  build)
    exec docker build -t "$TASK_IMAGE" "$TASK_DIR"
    ;;
  *) echo 'Usage: bash run.sh build' >&2; exit 2;;
esac
