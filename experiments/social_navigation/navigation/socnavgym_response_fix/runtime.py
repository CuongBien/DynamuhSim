"""Stage contract over the shared live-worker command builder."""
import sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
COMMON = HERE.parent / "common_response"
SOCIAL = HERE.parent.parent
sys.path.insert(0, str(HERE.parents[3] / 'python'))
from dynamuhsim.runtime.workers import worker_command as build_command
def worker_command(runtime, socket_dir, model, case, token):
    assert model == 'soc_guarded_orca'
    return build_command(HERE, runtime, socket_dir, model, case, token)[:2]
