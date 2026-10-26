"""Pinned isolated Python3.8 worker; ROS/Nav2 stays on host Python3.12."""
import json,os
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;COMMON=BENCH/'common_response'
MODELS=['crowd_orca_plus','complex_geometry_mixed']
def worker_command(runtime,socket_dir,model,token):
    assert model in MODELS
    images=json.loads((COMMON/'inputs/images.json').read_text());image=images['dynamuhsim-crowdsimplus:round2' if model=='crowd_orca_plus' else 'dynamuhsim-complexitynav:round3'];container='dynamuhsim-nav2-shared-'+token[:12]
    command=['docker','run','--rm','--name',container,'--network','none','--cpus','2','--memory','4g','--user',f'{os.getuid()}:{os.getgid()}','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','MKL_NUM_THREADS=1','-e','DYNAMUHSIM_EXPERIMENT_SESSION='+token,'-v',f'{HERE}:/live:ro','-v',f'{BENCH/"common_gazebo"}:/common_gazebo:ro','-v',f'{COMMON}:/common:ro','-v',f'{BENCH/"complexitynav_geometry"}:/geometry:ro','-v',f'{runtime}:/runtime','-v',f'{socket_dir}:/socket',image,'python','/live/worker.py','--common','/common','--runtime','/runtime','--socket','/socket/worker.sock','--model',model,'--case','crossing_near']
    return command,container,image
