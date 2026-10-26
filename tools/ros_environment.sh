#!/usr/bin/env bash
# ROS detection and discovery settings shared by workspace entrypoints.
dynamuhsim_source_ros() {
  # Auto-detect ROS 2 distribution if not already set or sourced
  if [[ -z "${ROS_DISTRO:-}" ]]; then
      for distro in jazzy humble iron rolling; do
          if [[ -f "/opt/ros/$distro/setup.bash" ]]; then
              ROS_DISTRO="$distro"
              break
          fi
      done
  fi

  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
      if [[ "${1:-bash}" == zsh ]]; then
          source "/opt/ros/$ROS_DISTRO/setup.sh"
      else
          source "/opt/ros/$ROS_DISTRO/setup.bash"
      fi
  elif ! command -v ros2 &>/dev/null; then
      echo "[ERROR] No ROS 2 installation found in /opt/ros or PATH."
      return 1
  fi
}

dynamuhsim_discovery() {
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  export ROS_AUTOMATIC_DISCOVERY_RANGE="${DYNAMUHSIM_DISCOVERY_RANGE:-LOCALHOST}"
}
