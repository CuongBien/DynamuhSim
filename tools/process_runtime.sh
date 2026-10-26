#!/usr/bin/env bash
# Sourced helpers for launchers that own private process sessions.
process_init() {
  set +m  # setsid must keep the PID recorded by process_start.
  PROCESS_PIDS=()
  PROCESS_SIGNALS=("$@")
  if (($# == 0)); then PROCESS_SIGNALS=(INT TERM KILL); fi
  trap process_cleanup EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM
}

process_start() {
  setsid "$@" 9>&- &
  PROCESS_PID=$!
  PROCESS_PIDS+=("$PROCESS_PID")
}

process_wait() {
  # wait -n can miss a child that exited before the wait began.
  while :; do
    for pid in "${PROCESS_PIDS[@]}"; do
      if ! kill -0 "$pid" 2>/dev/null; then wait "$pid"; return $?; fi
    done
    sleep 0.1
  done
}

process_stop() {
  local sig pid alive step
  # Each child has a private session; never signal a process by its name.
  for sig in "${PROCESS_SIGNALS[@]}"; do
    for pid in "${PROCESS_PIDS[@]}"; do
      # A launcher can fail before setsid has created the child's group.
      kill -s "$sig" -- "-$pid" 2>/dev/null || kill -s "$sig" "$pid" 2>/dev/null || true
    done
    [[ "$sig" == KILL ]] && break
    for ((step=0; step<${PROCESS_GRACE_STEPS:-20}; step++)); do
      alive=0
      for pid in "${PROCESS_PIDS[@]}"; do
        if kill -0 -- "-$pid" 2>/dev/null || kill -0 "$pid" 2>/dev/null; then alive=1; fi
      done
      ((alive)) || break
      sleep 0.1
    done
  done
  if ((${#PROCESS_PIDS[@]})); then wait "${PROCESS_PIDS[@]}" 2>/dev/null || true; fi
  PROCESS_PIDS=()
}

process_cleanup() {
  local status=$?
  trap - EXIT
  trap '' INT TERM
  process_stop
  exit "$status"
}
