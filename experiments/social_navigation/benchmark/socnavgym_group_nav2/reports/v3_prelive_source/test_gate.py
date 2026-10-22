#!/usr/bin/env python3
import argparse,copy,hashlib,json,socket,subprocess,tempfile,time,os,signal
from pathlib import Path
HERE=Path(__file__).resolve().parent;SOCIAL=HERE.parent.parent;parser=argparse.ArgumentParser();parser.add_argument('--name',required=True);parser.add_argument('--offline',required=True);args=parser.parse_args();out=HERE/'reports'/args.name;out.mkdir();results=[]
for scenario in ['walking','standing']:
 for seed in [42,43]:
    name=scenario+str(seed);runtime=HERE/'generated'/(args.name+'_'+name);runtime.mkdir()
    with tempfile.TemporaryDirectory(prefix='groupgate_') as directory,(out/(name+'.log')).open('w') as log:
        sock=Path(directory)/'worker.sock';cmd=[str(SOCIAL/'.venv-socnavgym/bin/python'),str(HERE/'worker.py'),'--runtime',str(runtime),'--socket',str(sock),'--scenario',scenario,'--seed',str(seed)];proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+90
            while not (runtime/'ready.json').exists():
                if proc.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Group gate worker readiness')
                time.sleep(.1)
            ready=json.loads((runtime/'ready.json').read_text());saved=json.loads((HERE/'reports'/args.offline/(name+'_trajectory.json')).read_text());robot=dict(x=-28.,y=0.,yaw=0.,vx=0.,vy=0.,omega=0.)
            with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as conn:
                conn.settimeout(20);conn.connect(str(sock));stream=conn.makefile('rb')
                def exchange(t):
                    conn.sendall((json.dumps(dict(time=t,dt=.1,epoch=0,frame='world',robot=robot))+'\n').encode());return json.loads(stream.readline())
                for i in range(10):
                    row=exchange(50+i*.1);assert row['humans']==ready['humans'] and row['groups']==ready['groups'] and row['native_step']==0 and not row['active_phase']
                (runtime/'start_gate').write_text('51.0')
                for i in range(20):
                    row=exchange(51+i*.1);assert row['active_phase'] and row['native_step']==i+1 and row['humans']==saved[i+1]['humans'] and row['groups']==saved[i+1]['groups'],(name,i)
                bad=exchange(60.);assert bad['fatal'] and bad['restart_required'];assert stream.readline()==b''
            proc.wait(timeout=10);assert proc.returncode==0
            results.append(dict(scenario=scenario,seed=seed,frozen_ticks=10,exact_native_prefix=20,terminal_bad_clock=True))
        finally:
            if proc.poll() is None:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10)
validation=dict(status='PASS',cases=results)
(out/'validation.json').write_text(json.dumps(validation,indent=2)+'\n');print(json.dumps(validation,indent=2))
