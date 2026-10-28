"""Compatibility entry point for the shared warehouse launch description."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "python"))
from dynamuhsim.simulation.warehouse import generate_launch_description as warehouse_launch


def generate_launch_description():
    return warehouse_launch(Path(__file__))
