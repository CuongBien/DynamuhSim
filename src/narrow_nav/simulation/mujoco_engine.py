"""Headless runtime assembled with the official MuJoCo adapter."""

from pathlib import Path

from .lifecycle import SimulationLifecycle
from .mujoco.adapter import MuJoCoAdapter
from .world_builder import MJCF_ROOT


class MuJoCoEngine(SimulationLifecycle):
    def __init__(self, *, mjcf_root: Path = MJCF_ROOT) -> None:
        super().__init__(MuJoCoAdapter(), mjcf_root=mjcf_root)
