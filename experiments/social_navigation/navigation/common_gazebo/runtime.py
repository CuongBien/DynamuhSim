"""Stage contract over the shared live-worker command builder."""
import json, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
COMMON = HERE.parent / "common_response"
SOCIAL = HERE.parent.parent
sys.path.insert(0, str(HERE.parents[3] / 'python'))
from dynamuhsim.runtime.workers import worker_command as build_command
MODELS = ['soc_guarded_orca', 'crowd_orca_plus', 'complex_geometry_mixed']
def worker_command(runtime, socket_dir, model, case, token):
    kind = json.loads((HERE.parent / 'common_response/inputs/protocol.json').read_text())['models'][model]['runtime']
    return build_command(HERE, runtime, socket_dir, model, case, token,
                         container_prefix=None if kind == 'venv' else 'dynamuhsim-common-gz-')[:2]
