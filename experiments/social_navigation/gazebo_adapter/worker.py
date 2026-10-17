#!/usr/bin/env python3
"""Private Unix socket worker. One request advances exactly one 0.1 s native step."""
import argparse
import json
from pathlib import Path
import socket
import time
from kernel import GroupKernel

p = argparse.ArgumentParser()
p.add_argument('--runtime', type=Path, required=True)
p.add_argument('--socket', required=True)
p.add_argument('--mode', choices=['walking','conversation'], required=True)
p.add_argument('--seed', type=int, default=42)
a = p.parse_args()
layout = json.loads((Path(__file__).resolve().parent.parent/'warehouse/layout.json').read_text())
k = GroupKernel(a.runtime/'config.yaml', layout, a.mode, a.seed)
server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
server.bind(a.socket)
server.listen(1)
(a.runtime/'ready.json').write_text(json.dumps(dict(humans=k.initial,
    native_initial_observation_in_space=bool(k.initial_observation_in_space), obstacles=len(k.env.walls))))
try:
    connection, _ = server.accept()
    connection.settimeout(None)  # A paused simulator sends no requests.
    with connection, connection.makefile('rb') as stream, (a.runtime/'trajectory.jsonl').open('w', buffering=1) as log:
        previous = None
        for line in stream:
            request = json.loads(line)
            started = time.perf_counter()
            try:
                t = request['time']
                if previous is not None and abs(t-previous-.1) > 1e-6:
                    raise RuntimeError('Non-contiguous simulation time; restart the experiment after reset')
                if request['advance']:
                    if abs(request['dt']-.1) > 1e-6:
                        raise RuntimeError('Unexpected simulation timestep')
                    k.advance(request['robot'])
                elif previous is not None:
                    raise RuntimeError('Unexpected reinitialization')
                previous = t
                reply = dict(time=t, step=k.steps, frame_id="world", humans=k.humans(), mode=a.mode,
                             compute_ms=1000*(time.perf_counter()-started))
                reply['groups']=[dict(id=1,type=a.mode,members=[1,2,3],center=[sum(h['x'] for h in reply['humans'])/3,sum(h['y'] for h in reply['humans'])/3])]
                log.write(json.dumps(dict(**reply, robot=request['robot']))+'\n')
            except Exception as ex:
                reply = dict(error=str(ex))
            connection.sendall((json.dumps(reply, allow_nan=False)+'\n').encode())
            if 'error' in reply:
                break
finally:
    server.close()
    k.close()
