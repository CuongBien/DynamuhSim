#!/usr/bin/env python3
import copy,json,socket,subprocess,tempfile,time,os,signal
from pathlib import Path
HERE=Path(__file__).resolve().parent;COMMON=HERE.parent/'common_response';SOCIAL=HERE.parent.parent
out=HERE/'reports/gate01';out.mkdir(exist_ok=False);runtime=HERE/'generated/gate01';runtime.mkdir(exist_ok=False)
with tempfile.TemporaryDirectory(prefix='navgate_') as directory,(out/'worker.log').open('w') as log:
    sock=Path(directory)/'worker.sock';cmd=[str(SOCIAL/'.venv-socnavgym/bin/python'),str(HERE/'worker.py'),'--common',str(COMMON),'--runtime',str(runtime),'--socket',str(sock),'--model','soc_guarded_orca','--case','crossing_near'];proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        deadline=time.monotonic()+90
        while not (runtime/'ready.json').exists():
            if proc.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Gate worker readiness')
            time.sleep(.1)
        ready=json.loads((runtime/'ready.json').read_text());initial=ready['humans'];case=json.loads((COMMON/'inputs/crossing_near.json').read_text());saved=json.loads((HERE.parent/'socnavgym_response_fix/reports/offline01/crossing_near_trajectory.json').read_text())
        with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as conn:
            conn.settimeout(20);conn.connect(str(sock));stream=conn.makefile('rb')
            def exchange(t,robot):conn.sendall((json.dumps(dict(time=t,dt=.1,epoch=0,frame='world',robot=robot))+'\n').encode());return json.loads(stream.readline())
            robot=dict(x=-28.,y=0.,yaw=0.,vx=0.,vy=0.,omega=0.)
            for i in range(10):
                row=exchange(50+i*.1,robot);assert row['humans']==initial and row['native_step']==0 and not row['active_phase']
            (runtime/'start_gate').write_text('51.0')
            for i in range(20):
                robot=copy.deepcopy(case['robot_trace'][i*2]);robot.pop('time');robot['omega']=0.;row=exchange(51+i*.1,robot);assert row['active_phase'] and row['native_step']==i+1
                for actual,expected in zip(row['humans'],saved[i+1]['humans']):
                    for key in expected:assert actual[key]==expected[key],(i,key)
            bad=exchange(60.,robot);assert bad['fatal'] and bad['restart_required'];assert stream.readline()==b''
        proc.wait(timeout=10);assert proc.returncode==0
    finally:
        if proc.poll() is None:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10)
validation=dict(status='PASS',frozen_warmup_transport_steps=10,native_step_during_warmup=0,exact_post_gate_offline_steps=20,terminal_bad_clock=True,command=cmd)
(out/'validation.json').write_text(json.dumps(validation,indent=2)+'\n');print(json.dumps(validation,indent=2))
