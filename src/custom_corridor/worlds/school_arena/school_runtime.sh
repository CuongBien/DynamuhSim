#!/usr/bin/env bash
# Shared setup and process ownership for the School Arena launchers.
school_ros() {
  source /opt/ros/jazzy/setup.bash
  if [[ -n "${1:-}" ]]; then source "$1"; fi
  set -u
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_DOMAIN_ID=0
}

school_processes() {
  set +m  # setsid must keep the PID recorded by school_start.
  SCHOOL_PIDS=()
  trap school_cleanup EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM
}

school_start() {
  setsid "$@" 9>&- &
  SCHOOL_PIDS+=("$!")
}

school_wait() {
  # wait -n can miss a child that exited before the wait began.
  while :; do
    for pid in "${SCHOOL_PIDS[@]}"; do
      if ! kill -0 "$pid" 2>/dev/null; then wait "$pid"; return $?; fi
    done
    sleep 0.1
  done
}

school_cleanup() {
  local status=$? sig pid alive
  trap - EXIT
  trap '' INT TERM
  # Each child has a private session; never signal a process by its name.
  for sig in INT TERM KILL; do
    for pid in "${SCHOOL_PIDS[@]}"; do
      # A launcher can fail before setsid has created the child's group.
      kill -s "$sig" -- "-$pid" 2>/dev/null || kill -s "$sig" "$pid" 2>/dev/null || true
    done
    [[ "$sig" == KILL ]] && break
    for _ in {1..20}; do
      alive=0
      for pid in "${SCHOOL_PIDS[@]}"; do
        if kill -0 -- "-$pid" 2>/dev/null || kill -0 "$pid" 2>/dev/null; then alive=1; fi
      done
      ((alive)) || break
      sleep 0.1
    done
  done
  wait "${SCHOOL_PIDS[@]}" 2>/dev/null || true
  exit "$status"
}
