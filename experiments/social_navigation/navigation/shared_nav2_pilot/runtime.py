"""Stage contract over the shared live-worker command builder."""
import sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
COMMON = HERE.parent / "common_response"
SOCIAL = HERE.parent.parent
sys.path.insert(0, str(HERE.parents[3] / 'python'))
from dynamuhsim.runtime.workers import worker_command as build_command
MODELS = ['crowd_orca_plus', 'complex_geometry_mixed']
def worker_command(runtime, socket_dir, model, token):
    assert model in MODELS
    return build_command(HERE, runtime, socket_dir, model, 'crossing_near', token,
                         container_prefix='dynamuhsim-nav2-shared-')
