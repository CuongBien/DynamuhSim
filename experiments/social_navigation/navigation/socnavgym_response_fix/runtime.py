"""Own only the isolated SocNavGym response worker; no container or ROS modification."""
from pathlib import Path
HERE=Path(__file__).resolve().parent;COMMON=HERE.parent/'common_response';SOCIAL=HERE.parent.parent
def worker_command(runtime,socket_dir,model,case,token):
    assert model=='soc_guarded_orca'
    return [str(SOCIAL/'.venv-socnavgym/bin/python'),str(HERE/'worker.py'),'--common',str(COMMON),'--runtime',str(runtime),'--socket',str(socket_dir/'worker.sock'),'--model',model,'--case',case],None
