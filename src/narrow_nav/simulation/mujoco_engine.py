"""Headless runtime assembled with the official MuJoCo adapter."""

from pathlib import Path

from narrow_nav.core.protocols import RegisteredHumanBackend

from .lifecycle import SimulationLifecycle
from .mujoco.adapter import MuJoCoAdapter
from .world_builder import MJCF_ROOT


class MuJoCoEngine(SimulationLifecycle):
    def __init__(
        self,
        *,
        mjcf_root: Path = MJCF_ROOT,
        human_backend: RegisteredHumanBackend | None = None,
    ) -> None:
        super().__init__(
            MuJoCoAdapter(human_motion_enabled=human_backend is not None),
            mjcf_root=mjcf_root,
            human_backend=human_backend,
        )
