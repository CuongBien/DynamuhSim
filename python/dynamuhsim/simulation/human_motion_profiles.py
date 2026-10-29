"""Explicit motion profile dispatch; omitted input preserves the legacy backend."""
import json
from pathlib import Path
from .human_groups import PROFILE as ORCA_PROFILE,enable_profile as enable_orca
from .group_intent import PROFILE as GOAL_PROFILE,GroupGoal


def profile_config(name):
    if name==ORCA_PROFILE["profile"]:return dict(ORCA_PROFILE)
    if name==GOAL_PROFILE["profile"]:return dict(GOAL_PROFILE)
    raise ValueError("Unsupported human motion profile")


def validate_profile(value):
    expected=profile_config(value.get("profile"))
    if value!=expected or (value["profile"]=="group_goal_v1" and value["group_avoidance"] is not False):
        raise ValueError("Undeclared human motion treatment")
    return expected


def enable_profile(kernel,value):
    config=validate_profile(value)
    if config["profile"]=="group_orca_v1":adapter=enable_orca(kernel,config)
    else:
        adapter=GroupGoal(kernel)
        kernel.env.compute_orca_interaction_velocities=adapter.velocities
    adapter.profile=config["profile"]
    return adapter


def load_profile(kernel,runtime):
    path=Path(runtime)/"human_motion.json"
    return enable_profile(kernel,json.loads(path.read_text())) if path.exists() else None
