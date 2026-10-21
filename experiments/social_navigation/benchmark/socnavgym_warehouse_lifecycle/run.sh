#!/usr/bin/env bash
set -euo pipefail
TASK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_ROOT="$(cd "$TASK_DIR/../../../.." && pwd)"
TASK_MODE="${1:-}"; shift || true
case "$TASK_MODE" in
  build)
    set +u; source /opt/ros/jazzy/setup.bash; set -u
    cmake -S "$TASK_DIR" -B "$TASK_DIR/build" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$TASK_DIR/build" -j2 ;;
  offline)
    exec "$TASK_DIR/../../.venv-socnavgym/bin/python" "$TASK_DIR/offline.py" "$@" ;;
  live|live-seed)
    python3 "$TASK_DIR/check_preflight.py"
    set +u; source /opt/ros/jazzy/setup.bash; source "$TASK_ROOT/install/setup.bash"; set -u
    if [[ "$TASK_MODE" == live-seed ]]; then exec python3 "$TASK_DIR/verify_live_seed.py" "$@"; fi
    exec python3 "$TASK_DIR/verify_live.py" "$@" ;;
  analyze)
    exec "$TASK_DIR/../../.venv-socnavgym/bin/python" "$TASK_DIR/analyze.py" "$@" ;;
  *) echo 'Usage: run.sh build|offline|live|live-seed|analyze [arguments]' >&2; exit 2 ;;
esac
