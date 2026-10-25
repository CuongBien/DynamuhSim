"""
Simulation Environment Helpers:
- Process cleanup for Gazebo Harmonic & Nav2
- Corridor scenario JSON generation
- SDF world file patching
- Environment variable configuration for GPU/D3D12 & ROS domain isolation
"""

import os
import re
import json
import time
import subprocess
from typing import Dict


def cleanup_all(sleep_sec: float = 2.0):
    """Terminates all lingering Gazebo, Nav2, and ROS 2 bridge processes."""
    subprocess.run(["pkill", "-9", "-f", "gz sim"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(
        [
            "killall", "-9", "map_server", "amcl", "planner_server", "controller_server",
            "bt_navigator", "behavior_server", "waypoint_follower", "lifecycle_manager",
            "parameter_bridge", "static_transform_publisher", "robot_state_publisher"
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    subprocess.run(["pkill", "-9", "-f", "nav2_"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "robot_state_publisher"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "corridor_tb3"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "parameter_bridge"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "ros2"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["ros2", "daemon", "stop"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if sleep_sec > 0:
        time.sleep(sleep_sec)


def create_corridor_scenario_json(
    scen_file: str, seed: int, persona: str,
    start_x: float = -7.0, start_y: float = 0.0, end_x: float = -15.0,
    v_human: float = None
) -> str:
    """Writes a single-human corridor scenario JSON file and returns its path."""
    if v_human is None:
        v_human = 1.05 if persona == "H1" else 1.15
    scen_data = {
        "episode_id": seed,
        "corridor_mode": True,
        "humans": [
            {
                "id": 1,
                "name": "human_01",
                "mode": "autonomous",
                "speed": v_human,
                "max_speed": v_human,
                "persona": persona,
                "loop": False,
                "wait_for_robot": True,
                "start": {"x": start_x, "y": start_y, "yaw": 3.14159265},
                "waypoints": [[start_x, start_y], [end_x, start_y]]
            }
        ]
    }
    os.makedirs(os.path.dirname(os.path.abspath(scen_file)), exist_ok=True)
    with open(scen_file, "w") as f:
        json.dump(scen_data, f, indent=2)
    return scen_file


def patch_corridor_world_sdf(
    base_sdf_path: str, out_sdf_path: str,
    seed: int, scen_file: str, rtf: float = 1.0, proxy_z: str = "0.875"
) -> str:
    """Patches corridor SDF world with RTF, seed, scenario path, and human starting pose."""
    with open(base_sdf_path, "r") as f:
        sdf = f.read()

    sdf = re.sub(
        r'<real_time_factor>.*?</real_time_factor>',
        f'<real_time_factor>{rtf}</real_time_factor>',
        sdf
    )
    sdf = re.sub(
        r'<randomize_politeness>.*?</randomize_politeness>',
        f'<randomize_politeness>false</randomize_politeness>\n      <seed>{seed}</seed>\n      <default_scenario_file>{scen_file}</default_scenario_file>',
        sdf
    )
    if f"<seed>{seed}</seed>" not in sdf:
        sdf = re.sub(r'<seed>\d+</seed>', f'<seed>{seed}</seed>', sdf)

    sdf = re.sub(
        r'<actor name=\"human_01_visual\">\s*<pose>.*?</pose>',
        '<actor name=\"human_01_visual\">\n      <pose>-7.0 0.0 0.0 0 0 3.14159</pose>',
        sdf
    )
    sdf = re.sub(
        r'<model name=\"human_01_proxy\">\s*<pose>.*?</pose>',
        f'<model name=\"human_01_proxy\">\n      <pose>-7.0 0.0 {proxy_z} 0 0 3.14159</pose>',
        sdf
    )

    os.makedirs(os.path.dirname(os.path.abspath(out_sdf_path)), exist_ok=True)
    with open(out_sdf_path, "w") as f:
        f.write(sdf)
    return out_sdf_path


def build_sim_env(root_dir: str, domain_id: int, partition: str) -> Dict[str, str]:
    """Builds the environment dictionary for launching Gazebo and Nav2."""
    corridor_models = os.path.join(root_dir, "install/custom_corridor/share/custom_corridor/models")
    plugin_share = os.path.join(root_dir, "install/custom_corridor_plugins/share/custom_corridor_plugins")
    plugin_lib = os.path.dirname(os.path.dirname(plugin_share)) + "/lib"

    env = os.environ.copy()
    env["GALLIUM_DRIVER"] = "d3d12"
    env["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"
    env["GZ_SIM_RESOURCE_PATH"] = f"{corridor_models}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{plugin_lib}:{os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"
    env["ROS_DOMAIN_ID"] = str(domain_id)
    env["GZ_PARTITION"] = partition
    return env
