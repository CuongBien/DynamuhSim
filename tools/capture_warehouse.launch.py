"""Launch API for new versioned capture runs."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from dynamuhsim.simulation.capture_launch import generate_launch_description
