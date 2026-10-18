#!/usr/bin/env python3
"""Unix socket host inside a network-isolated CrowdSimPlus container."""
import argparse,json,socket,time
from pathlib import Path
from kernel import CrowdKernel
p=argparse.ArgumentParser();p.add_argument('--runtime',type=Path,default=Path('/runtime'));p.add_argument('--socket',default='/socket/worker.sock');p.add_argument('--invisible-robot',action='store_true');p.add_argument('--human-policy',choices=['orca_plus','sfm'],default='orca_plus');a=p.parse_args()
k=CrowdKernel(json.loads((a.runtime/'scene.json').read_text()),json.loads((a.runtime/'layout.json').read_text()),not a.invisible_robot,a.human_policy)
(a.runtime/'crowdsim_metadata.json').write_text(json.dumps(k.metadata(),indent=2))
server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);server.bind(a.socket);server.listen(1)
(a.runtime/'worker_ready').write_text('ready')
try:
    connection,_=server.accept()
    with connection,connection.makefile('rb') as stream,(a.runtime/'native_trajectory.jsonl').open('w',buffering=1) as log:
        previous=None
        for line in stream:
            req=json.loads(line)
            try:
                if previous is not None and abs(req['time']-previous-.1)>1e-6:raise ValueError('Simulation clock discontinuity')
                active=(a.runtime/'start_gate').exists();begin=time.perf_counter()
                if req['advance'] and active:k.advance(req['robot'])
                previous=req['time']
                reply=dict(time=req['time'],step=k.steps,humans=k.humans(),groups=[],active=active,
                    framework='crowdsimplus',human_policy=k.human_policy,robot_visible=k.visible,actions=k.last_actions,compute_ms=1000*(time.perf_counter()-begin))
                log.write(json.dumps(dict(**reply,robot=req['robot']))+'\n')
            except Exception as ex:reply=dict(error=repr(ex))
            connection.sendall((json.dumps(reply,allow_nan=False)+'\n').encode())
            if 'error' in reply:break
finally:server.close()
