#!/usr/bin/env bash

# note:
# cd vào folder github
# run: `source setup_env.sh` để setup enviroment cho ROS 2 Jazzy và DynamuhSim workspace

DYNAMUHSIM_ROOT="$(
    cd "$(dirname "${BASH_SOURCE[0]}")/.." &&
    pwd
)"

export DYNAMUHSIM_ROOT

# Avoid stale prefix variables when this script is sourced more than once.
unset AMENT_CURRENT_PREFIX AMENT_SHELL COLCON_CURRENT_PREFIX

source "$DYNAMUHSIM_ROOT/tools/ros_environment.sh"
dynamuhsim_source_ros "${ZSH_VERSION:+zsh}" || { return 1 2>/dev/null || exit 1; }

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

dynamuhsim_discovery

# Gazebo Sim 8 (Harmonic) Resource & System Plugin Paths
TB3_MODELS="/opt/ros/${ROS_DISTRO:-jazzy}/share/turtlebot3_gazebo/models"
CORRIDOR_MODELS="$DYNAMUHSIM_ROOT/install/custom_corridor/share/custom_corridor/models"
PLUGINS_LIB="$DYNAMUHSIM_ROOT/install/custom_corridor_plugins/lib"

export GZ_SIM_RESOURCE_PATH="$CORRIDOR_MODELS:$TB3_MODELS${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PLUGINS_LIB${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"

# Hardware-accelerated Direct3D 12 rendering on NVIDIA dGPU in WSL2
export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"

echo "[DynamuhSim environment]"
echo "ROOT      : $DYNAMUHSIM_ROOT"
echo "ROS_DISTRO: ${ROS_DISTRO:-unknown}"
echo "DISCOVERY : $ROS_AUTOMATIC_DISCOVERY_RANGE"
echo "GZ_MODELS : $CORRIDOR_MODELS"
echo "RENDERER  : D3D12 ($MESA_D3D12_DEFAULT_ADAPTER_NAME)"
