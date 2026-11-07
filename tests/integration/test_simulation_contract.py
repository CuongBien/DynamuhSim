import os
import subprocess
import sys

from narrow_nav.core.observations import Observation
from narrow_nav.core.states import GroundTruthState
from narrow_nav.simulation import SimulationBackend
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine


def test_public_backend_contract_has_no_privileged_sensor_data(monkeypatch):
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
    backend: SimulationBackend = MuJoCoEngine()
    backend.load(
        compose_config(
            overrides=(
                "worlds=blind_corner",
                "scenarios=blind_corner",
                "simulation.observation_sensors=[lidar]",
            )
        )
    )
    reset = backend.reset()
    assert isinstance(reset, Observation)
    truth = backend.get_ground_truth()
    assert isinstance(truth, GroundTruthState) and truth.humans
    assert [(slot.sensor_id, slot.reason, slot.sample) for slot in reset.sensors] == [
        ("lidar", "capability_unavailable", None)
    ]
    assert backend.get_metadata().unavailable_sensors == (
        "depth",
        "imu",
        "lidar",
        "odometry",
    )
    backend.close()


def test_core_and_backend_protocol_import_without_native_mujoco():
    code = """
import sys
import narrow_nav.core.protocols
import narrow_nav.core.states
import narrow_nav.simulation.backend
assert 'mujoco' not in sys.modules
"""
    env = dict(os.environ)
    env.pop("PYTHONPATH", None)
    subprocess.run([sys.executable, "-c", code], env=env, check=True)
