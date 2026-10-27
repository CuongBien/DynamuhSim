#!/usr/bin/env python3
import copy,json,socket,subprocess,tempfile,time,os,signal,uuid
from pathlib import Path
from runtime import worker_command,MODELS
HERE=Path(__file__).resolve().parent;COMMON=HERE.parent/'common_response'
out=HERE/'reports/gate01';out.mkdir(exist_ok=False);results=[]
for model in MODELS:
    runtime=HERE/'generated'/('gate_'+model);runtime.mkdir(exist_ok=False)
    dt=json.loads((HERE/'protocol.json').read_text())['native_dt'][model]
    with tempfile.TemporaryDirectory(prefix='navgate_') as directory,(out/(model+'.log')).open('w') as log:
        sock=Path(directory)/'worker.sock';cmd,container,image=worker_command(runtime.resolve(),Path(directory),model,uuid.uuid4().hex);proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+90
            while not (runtime/'ready.json').exists():
                if proc.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Gate worker readiness: '+model)
                time.sleep(.1)
            ready=json.loads((runtime/'ready.json').read_text());initial=ready['humans'];case=json.loads((COMMON/'inputs/crossing_near.json').read_text());saved=json.loads((COMMON/'reports/run01'/model/'crossing_near_trajectory.json').read_text())
            with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as conn:
                conn.settimeout(20);conn.connect(str(sock));stream=conn.makefile('rb')
                def exchange(t,robot):
                    conn.sendall((json.dumps(dict(time=t,dt=dt,epoch=0,frame='world',robot=robot))+'\n').encode());return json.loads(stream.readline())
                robot=dict(x=-28.,y=0.,yaw=0.,vx=0.,vy=0.,omega=0.)
                for i in range(10):
                    row=exchange(50+i*dt,robot);assert row['humans']==initial and row['native_step']==0 and not row['active_phase']
                (runtime/'start_gate').write_text(str(50+10*dt))
                for i in range(20):
                    robot=copy.deepcopy(case['robot_trace'][round(i*dt/.05)]);robot.pop('time');robot['omega']=0.;row=exchange(50+(10+i)*dt,robot);assert row['active_phase'] and row['native_step']==i+1
                    for actual,expected in zip(row['humans'],saved[i+1]['humans']):
                        for key in expected:assert actual[key]==expected[key],(model,i,key,actual[key],expected[key])
                bad=exchange(100.,robot);assert bad['fatal'] and bad['restart_required'];assert stream.readline()==b''
            proc.wait(timeout=10);assert proc.returncode==0
            results.append(dict(model=model,native_dt=dt,image=image,frozen_steps=10,exact_native_prefix_steps=20,terminal_clock_fault=True))
        finally:
            present=subprocess.check_output(['docker','ps','-aq','--filter','name=^/'+container+'$'],text=True).strip()
            if present:subprocess.run(['docker','stop','-t','3',container],check=True,capture_output=True,timeout=12)
            if proc.poll() is None:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10)
            assert not subprocess.check_output(['docker','ps','-aq','--filter','name=^/'+container+'$'],text=True).strip()
validation=dict(status='PASS',models=results)
(out/'validation.json').write_text(json.dumps(validation,indent=2)+'\n');print(json.dumps(validation,indent=2))
