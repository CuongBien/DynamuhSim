#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u
case "${1:-}" in
  trial|matrix|geometry-matrix|avoidance-matrix|crowdsim-matrix|sfm-matrix|individual-goals-matrix)
    MODE="$1"; shift
    if [[ "$MODE" == matrix ]]; then exec python3 "$HERE/matrix.py" "$@"; fi
    if [[ "$MODE" == geometry-matrix ]]; then exec python3 "$HERE/geometry_matrix.py" "$@"; fi
    if [[ "$MODE" == avoidance-matrix ]]; then exec python3 "$HERE/avoidance_matrix.py" "$@"; fi
    if [[ "$MODE" == crowdsim-matrix ]]; then exec python3 "$HERE/crowdsim_matrix.py" "$@"; fi
    if [[ "$MODE" == sfm-matrix ]]; then exec python3 "$HERE/sfm_matrix.py" "$@"; fi
    if [[ "$MODE" == individual-goals-matrix ]]; then exec python3 "$HERE/individual_goals_matrix.py" "$@"; fi
    exec python3 "$HERE/trial.py" "$@" ;;
  *) echo 'Usage: run.sh trial --backend empty|rvo2|socnavgym|crowdsimplus [--human-policy orca_plus|sfm] [--goal-layout shared|individual] [--gui] [--report-dir DIR] | matrix|geometry-matrix|avoidance-matrix|crowdsim-matrix|sfm-matrix|individual-goals-matrix' >&2; exit 2 ;;
esac
