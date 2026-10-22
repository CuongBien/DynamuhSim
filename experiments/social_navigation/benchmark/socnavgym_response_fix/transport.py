#!/usr/bin/env python3
"""Real isolated-worker IPC matches saved offline prefix and closes after bad clock."""
import argparse,copy,json,socket,subprocess,tempfile,time,uuid
from pathlib import Path
from runtime import HERE,COMMON,worker_command
MODELS=['soc_guarded_orca']
parser=argparse.ArgumentParser();parser.add_argument('--report-name',default='transport01');args=parser.parse_args();assert args.report_name.replace('_','').replace('-','').isalnum();out=HERE/'reports'/args.report_name;out.mkdir(exist_ok=False);results=[]
for model in MODELS:
    runtime=HERE/'generated'/('transport_'+model+'_'+uuid.uuid4().hex[:6]);runtime.mkdir(exist_ok=False)
    with tempfile.TemporaryDirectory(prefix='commonipc_') as sockdir,(out/(model+'.log')).open('w') as log:
        cmd,container=worker_command(runtime,Path(sockdir),model,'crossing_near',uuid.uuid4().hex)
        proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+90
            while not (runtime/'ready.json').exists():
                if proc.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Worker readiness failed: '+model)
                time.sleep(.1)
            ready=json.loads((runtime/'ready.json').read_text());dt=ready['native_dt'];case=json.loads((COMMON/'inputs/crossing_near.json').read_text());saved=json.loads((HERE/'reports/offline01/crossing_near_trajectory.json').read_text())
            with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as conn:
                conn.settimeout(20);conn.connect(str(Path(sockdir)/'worker.sock'));stream=conn.makefile('rb')
                def exchange(req):conn.sendall((json.dumps(req)+'\n').encode());return json.loads(stream.readline())
                for i in range(20):
                    robot=copy.deepcopy(case['robot_trace'][round(i*dt/.05)]);robot.pop('time');robot['omega']=0.;reply=exchange(dict(time=3+i*dt,dt=dt,epoch=0,frame='world',robot=robot))
                    assert 'error' not in reply,reply
                    for actual,expected in zip(reply['humans'],saved[i+1]['humans']):
                        for key in expected:assert actual[key]==expected[key],(model,i,key,actual[key],expected[key])
                reply=exchange(dict(time=3+22*dt,dt=dt,epoch=0,frame='world',robot=robot));assert reply['fatal'] and reply['restart_required'] and 'Noncontiguous' in reply['error'];assert stream.readline()==b''
            proc.wait(timeout=20);assert proc.returncode==0
            results.append(dict(model=model,status='PASS',exact_saved_steps=20,terminal_bad_clock=True,command=cmd))
        finally:
            if container and subprocess.check_output(['docker','ps','-aq','--filter','name=^/'+container+'$'],text=True).strip():subprocess.run(['docker','stop','-t','3',container],capture_output=True,timeout=12)
            if proc.poll() is None:proc.terminate();proc.wait(timeout=10)
(out/'validation.json').write_text(json.dumps(dict(status='PASS',models=results),indent=2)+'\n');print(json.dumps(results,indent=2))
