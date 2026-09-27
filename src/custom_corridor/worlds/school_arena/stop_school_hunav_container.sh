#!/usr/bin/env bash
set -euo pipefail

CONTAINER=${1:-hunavsim_gz_fortress}

# Match executable command lines without matching this cleanup shell itself.
docker exec "$CONTAINER" bash -lc '
  mapfile -t pids < <(pgrep -f \
    "[h]unav_agent_manager/hunav_loader|[h]unav_agent_manager/hunav_agent_manager|[h]unav_gz8_school_bridge.py" \
    || true)
  if ((${#pids[@]})); then
    kill -INT "${pids[@]}" 2>/dev/null || true
    sleep 1
  fi
  mapfile -t pids < <(pgrep -f \
    "[h]unav_agent_manager/hunav_loader|[h]unav_agent_manager/hunav_agent_manager|[h]unav_gz8_school_bridge.py" \
    || true)
  if ((${#pids[@]})); then
    kill -TERM "${pids[@]}" 2>/dev/null || true
  fi
'

echo "Stopped HuNav loader, manager and school bridge in $CONTAINER"
