#!/usr/bin/env bash
# Sourced helpers for launchers that own private process sessions.
process_init() {
  set +m  # setsid must keep the PID recorded by process_start.
  PROCESS_PIDS=()
  PROCESS_RECORDERS=()
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

# Recorders get the first stop signal together, then flush in reverse start order.
process_record() {
  local seconds="$1"; shift
  process_start "$@"
  PROCESS_RECORDERS=("$PROCESS_PID" "$seconds" "${PROCESS_RECORDERS[@]}")
}

process_logged() {
  local log="$1"; shift
  process_start bash -c 'log=$1; shift; "$@" | tee "$log"; exit "${PIPESTATUS[0]}"' process-log "$log" "$@"
  wait "$PROCESS_PID"
}

process_signal() {
  kill -s "$2" -- "-$1" 2>/dev/null || kill -s "$2" "$1" 2>/dev/null || true
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
  local sig pid alive step i deadline
  # Each child has a private session; never signal a process by its name.
  for sig in "${PROCESS_SIGNALS[@]}"; do
    for pid in "${PROCESS_PIDS[@]}"; do
      # A launcher can fail before setsid has created the child's group.
      process_signal "$pid" "$sig"
    done
    for ((i=0; i<${#PROCESS_RECORDERS[@]}; i+=2)); do
      pid=${PROCESS_RECORDERS[i]}
      deadline=$(( $(date +%s) + PROCESS_RECORDERS[i+1] ))
      while kill -0 "$pid" 2>/dev/null && [ "$(date +%s)" -lt "$deadline" ]; do sleep 0.5; done
      process_signal "$pid" KILL
      wait "$pid" 2>/dev/null || true
    done
    PROCESS_RECORDERS=()
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
