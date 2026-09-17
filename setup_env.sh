#!/usr/bin/env bash

# note: 
# cd vào folder github
# run: `source setup_env.sh` để setup enviroment cho ROS 2 Jazzy và DynamuhSim workspace

DYNAMUHSIM_ROOT="$(
    cd "$(dirname "${BASH_SOURCE[0]}")" &&
    pwd
)"

export DYNAMUHSIM_ROOT

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "[ERROR] ROS 2 Jazzy not found"
    return 1 2>/dev/null || exit 1
fi

source /opt/ros/jazzy/setup.bash

if [[ ! -f "$DYNAMUHSIM_ROOT/install/setup.bash" ]]; then
    echo "[ERROR] DynamuhSim workspace is not built."
    echo
    echo "Run:"
    echo "  cd \"$DYNAMUHSIM_ROOT\""
    echo "  colcon build --symlink-install"
    return 1 2>/dev/null || exit 1
fi

source "$DYNAMUHSIM_ROOT/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=42

echo "[DynamuhSim environment]"
echo "ROOT      : $DYNAMUHSIM_ROOT"
echo "ROS_DISTRO: $ROS_DISTRO"