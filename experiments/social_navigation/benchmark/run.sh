#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u
case "${1:-}" in
  trial|matrix)
    MODE="$1"; shift
    if [[ "$MODE" == matrix ]]; then exec python3 "$HERE/matrix.py" "$@"; fi
    exec python3 "$HERE/trial.py" "$@" ;;
  *) echo 'Usage: run.sh trial --backend empty|rvo2|socnavgym --condition standing|counterflow --seed 42 [--gui] | matrix' >&2; exit 2 ;;
esac
