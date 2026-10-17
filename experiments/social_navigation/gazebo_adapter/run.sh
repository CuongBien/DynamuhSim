#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
case "${1:-demo}" in
  build)
    cmake -S "$HERE" -B "$HERE/build" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$HERE/build" -j2
    ;;
  demo|verify)
    MODE="${1:-demo}"
    if [[ $# -gt 0 ]]; then shift; fi
    if [[ ! -f "$HERE/build/libsocnavgym_adapter.so" ]]; then
      echo "Run $HERE/run.sh build first" >&2; exit 1
    fi
    set +u
    source /opt/ros/jazzy/setup.bash
    source "$ROOT/install/setup.bash"
    set -u
    if [[ "$MODE" == verify ]]; then
      exec python3 "$HERE/verify.py" "$@"
    fi
    exec python3 "$HERE/run.py" "$@"
    ;;
  *) echo 'Usage: run.sh build | demo [--mode walking|conversation] [--gui] [--rviz] [--nav] | verify [options]' >&2; exit 2 ;;
esac
