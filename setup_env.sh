#!/usr/bin/env bash

# note: 
# cd vào folder github
# run: `source setup_env.sh` để setup enviroment cho ROS 2 Jazzy và DynamuhSim workspace

DYNAMUHSIM_ROOT="$(
    cd "$(dirname "${BASH_SOURCE[0]}")" &&
    pwd
)"

export DYNAMUHSIM_ROOT

# Avoid stale prefix variables when this script is sourced more than once.
unset AMENT_CURRENT_PREFIX AMENT_SHELL COLCON_CURRENT_PREFIX

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
    unset AMENT_CURRENT_PREFIX
    if [[ -n "${ZSH_VERSION:-}" ]]; then
        source "/opt/ros/$ROS_DISTRO/setup.sh"
    else
        source "/opt/ros/$ROS_DISTRO/setup.bash"
    fi
elif ! command -v ros2 &>/dev/null; then
    echo "[ERROR] No ROS 2 installation found in /opt/ros or PATH."
    return 1 2>/dev/null || exit 1
fi

if [[ ! -f "$DYNAMUHSIM_ROOT/install/setup.bash" ]]; then
    echo "[ERROR] DynamuhSim workspace is not built."
    echo
    echo "Run:"
    echo "  cd \"$DYNAMUHSIM_ROOT\""
    echo "  colcon build --symlink-install"
    return 1 2>/dev/null || exit 1
fi

# colcon's generated setup chain is not safe when the workspace path contains
# spaces. Source the local prefix with an explicit quoted prefix instead.
export COLCON_CURRENT_PREFIX="$DYNAMUHSIM_ROOT/install"
if [[ -n "${ZSH_VERSION:-}" ]]; then
    source "$DYNAMUHSIM_ROOT/install/local_setup.sh"
else
    source "$DYNAMUHSIM_ROOT/install/local_setup.bash"
fi
unset COLCON_CURRENT_PREFIX

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${DYNAMUHSIM_DISCOVERY_RANGE:-LOCALHOST}"

echo "[DynamuhSim environment]"
echo "ROOT      : $DYNAMUHSIM_ROOT"
echo "ROS_DISTRO: ${ROS_DISTRO:-unknown}"
echo "DISCOVERY : $ROS_AUTOMATIC_DISCOVERY_RANGE"