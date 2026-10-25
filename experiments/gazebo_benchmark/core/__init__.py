"""
Shared Core Modules for DynamuhSim Gazebo Harmonic Benchmarks.
- geometry: Footprint clearance and predictive wall safety filter.
- mppi_monitor: One ROS interaction monitor for Step1/Step2/Step3 profiles (import explicitly when ROS is available).
- policies: Robot policies P1, P2, P3 (a-d), and P4.
- sim_env: Gazebo environments, spawn/bridge commands, SDF patching and scenario helpers.
- trajectory: A3 sim-time trajectory collectors (import explicitly when ROS is available).
"""

from .geometry import (
    compute_rect_circle_distance,
    apply_footprint_wall_safety_filter,
    check_footprint_wall_contact,
)
from .policies import (
    BasePolicy,
    P1Policy,
    P2Policy,
    P3aPolicy,
    P3bPolicy,
    P3cPolicy,
    P3dPolicy,
    P4Policy,
    create_policy_instance,
)
from .sim_env import (
    cleanup_all,
    create_corridor_scenario_json,
    patch_corridor_world_sdf,
    build_sim_env,
)
