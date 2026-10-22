"""Only this cohort's worker processes/containers are owned here."""
import json,os
from pathlib import Path
HERE=Path(__file__).resolve().parent;COMMON=HERE.parent/'common_response';SOCIAL=HERE.parent.parent
MODELS=['soc_guarded_orca','crowd_orca_plus','complex_geometry_mixed']
def worker_command(runtime,socket_dir,model,case,token):
    protocol=json.loads((COMMON/'inputs/protocol.json').read_text());kind=protocol['models'][model]['runtime'];container=None
    if kind=='venv':
        cmd=[str(SOCIAL/'.venv-socnavgym/bin/python'),str(HERE/'worker.py'),'--common',str(COMMON),'--runtime',str(runtime),'--socket',str(socket_dir/'worker.sock')]
    else:
        container='dynamuhsim-common-gz-'+token[:12];images=json.loads((COMMON/'inputs/images.json').read_text());image=images['dynamuhsim-crowdsimplus:round2' if kind=='crowd' else 'dynamuhsim-complexitynav:round3']
        cmd=['docker','run','--rm','--name',container,'--network','none','--cpus','2','--memory','4g','--user',f'{os.getuid()}:{os.getgid()}','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','MKL_NUM_THREADS=1','-e','DYNAMUHSIM_EXPERIMENT_SESSION='+token,'-v',f'{HERE}:/live:ro','-v',f'{COMMON}:/common:ro','-v',f'{HERE.parent/"complexitynav_geometry"}:/geometry:ro','-v',f'{runtime}:/runtime','-v',f'{socket_dir}:/socket',image,'python','/live/worker.py','--common','/common','--runtime','/runtime','--socket','/socket/worker.sock']
    return cmd+['--model',model,'--case',case],container
