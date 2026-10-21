#!/usr/bin/env python3
import argparse,json,os,socket,time
from pathlib import Path
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
os.environ['SDL_VIDEODRIVER']='dummy';os.environ['SDL_AUDIODRIVER']='dummy';os.sched_setaffinity(0,sorted(os.sched_getaffinity(0))[:2])
from kernel import Kernel
HERE=Path(__file__).resolve().parent;SOCIAL=HERE.parent.parent
p=argparse.ArgumentParser();p.add_argument('--seed',type=int,required=True);p.add_argument('--runtime',type=Path,required=True);p.add_argument('--socket',required=True);a=p.parse_args()
k=Kernel(HERE/'config.yaml',json.loads((SOCIAL/'warehouse/layout.json').read_text()),seed=a.seed);server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);server.bind(a.socket);server.listen(1)
(a.runtime/'ready.json').write_text(json.dumps(dict(humans=k.initial,obstacles=len(k.env.walls),origin=k.origin,goal_sampling_window=[-27.5,-20.5,-1.5,1.5],robot_radius=.25,wall_observation_shape=list(k.env.observation_space['walls'].shape))))
try:
    conn,_=server.accept()
    with conn,conn.makefile('rb') as stream,(a.runtime/'worker_trajectory.jsonl').open('w',buffering=1) as log:
        previous=None
        for line in stream:
            request=json.loads(line);started=time.monotonic()
            try:
                t=request['time']
                if previous is not None and abs(t-previous-.1)>1e-6:raise RuntimeError('Reset/non-contiguous clock requires a fresh session')
                if abs(request['dt']-.1)>1e-6:raise RuntimeError('Unexpected planning dt')
                before=k.humans();groups_before=k.groups();events=len(k.motion.events);k.advance(request['robot']);previous=t
                reply=dict(time=t,state_time=t+.1,step=k.steps,before=before,humans=k.humans(),groups_before=groups_before,groups=k.groups(),guard=k.motion.steps[-1],finish_events=k.motion.events[events:],compute_ms=1000*(time.monotonic()-started))
                log.write(json.dumps(dict(**reply,robot_input=request['robot']))+'\n')
            except Exception as ex:reply=dict(error=str(ex))
            conn.sendall((json.dumps(reply,allow_nan=False)+'\n').encode())
            if 'error' in reply:break
finally:server.close();k.close()
