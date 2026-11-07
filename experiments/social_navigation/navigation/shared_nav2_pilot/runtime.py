"""Stage contract over the shared live-worker command builder."""
import sys
import hashlib
import json
from pathlib import Path
HERE = Path(__file__).resolve().parent
COMMON = HERE.parent / "common_response"
SOCIAL = HERE.parent.parent
sys.path.insert(0, str(HERE.parents[3] / 'python'))
from dynamuhsim.runtime.workers import worker_command as build_command
MODELS = ['crowd_orca_plus', 'complex_geometry_mixed']

def prepare_capture_kernel():
    """Restore the existing native kernel's six-human allocation, without a new policy."""
    source = HERE.parents[1] / 'crowdsimplus_adapter/kernel.py'
    destination = COMMON / 'generated/crowd_kernel_n6.py'
    text = source.read_text()
    base = text
    for old, new in [('self.targets=[0,0,0]', "self.targets=[0]*len(scene['initial'])"),
                     ('human_num=3', "human_num=len(scene['initial'])"),
                     ('self.arrivals=[0,0,0]', "self.arrivals=[0]*len(scene['initial'])")]:
        if text.count(old) != 1:
            raise RuntimeError('Native kernel population contract changed: ' + old)
        text = text.replace(old, new)
    expected = json.loads((COMMON / 'inputs/crowd_bookkeeping_patch.json').read_text())
    if hashlib.sha256(base.encode()).hexdigest() != expected['original_sha256'] or hashlib.sha256(text.encode()).hexdigest() != expected['generated_sha256']:
        raise RuntimeError('Native CrowdSimPlus source differs from its locked population-only patch')
    compile(text, str(destination), 'exec')
    destination.parent.mkdir(parents=True, exist_ok=True)
    if not destination.exists() or destination.read_text() != text:
        destination.write_text(text)
    return destination

def worker_command(runtime, socket_dir, model, token, *, image=None):
    assert model in MODELS
    command, name, pinned = build_command(HERE, runtime, socket_dir, model, 'crossing_near', token,
                                         container_prefix='dynamuhsim-nav2-shared-')
    if image is not None:
        if model != 'crowd_orca_plus' or not image.startswith('sha256:'):
            raise ValueError('Local image override requires a content-pinned CrowdSimPlus image')
        command[command.index(pinned)] = image
        command[2:2] = ['--pull', 'never']
        command[command.index(image):command.index(image)] = ['-e', 'DYNAMUHSIM_POLICY_AUDIT=/runtime/policy_audit.json']
        pinned = image
    return command, name, pinned
