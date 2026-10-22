#!/usr/bin/env python3
"""Exercise the actual container worker and Unix socket before Gazebo."""
import copy,hashlib,json,os,socket,subprocess,tempfile,time,uuid
from pathlib import Path
HERE=Path(__file__).resolve().parent;GEOM=HERE.parent/'complexitynav_geometry';IMAGE='sha256:edfd591ea7551813419e511b341089ad956b6d3e4c1bd395a326035865ef1d5d'
out=HERE/'reports/transport01';out.mkdir(parents=True,exist_ok=False);results=[]
for case in ['duplicate_clock','stale_epoch','nonfinite_robot','nonfinite_dt','infeasible_robot']:
 runtime=HERE/'generated'/('ipc_'+case);runtime.mkdir(exist_ok=False);name='dynamuhsim-cnav-ipc-'+uuid.uuid4().hex[:10]
 with tempfile.TemporaryDirectory(prefix='cnav_ipc_') as temporary,(runtime/'worker.log').open('w') as log:
  cmd=['docker','run','--rm','--name',name,'--network','none','--cpus','2','--memory','4g','--user',f'{os.getuid()}:{os.getgid()}','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-v',f'{HERE}:/work:ro','-v',f'{GEOM}:/geometry:ro','-v',f'{runtime}:/runtime','-v',f'{temporary}:/socket',IMAGE,'python','/work/worker.py','--runtime','/runtime','--socket','/socket/worker.sock']
  proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT)
  try:
   deadline=time.monotonic()+30
   while not (runtime/'ready.json').exists():
    if proc.poll() is not None or time.monotonic()>deadline:raise RuntimeError('IPC worker startup failed')
    time.sleep(.05)
   client=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);client.settimeout(10);client.connect(temporary+'/worker.sock');stream=client.makefile('rb')
   def send(req):client.sendall((json.dumps(req)+'\n').encode());return json.loads(stream.readline())
   req=dict(time=.0025,dt=.25,epoch=0,frame='world',robot=dict(x=-27.,y=0.,vx=0.,vy=0.,yaw=0.))
   valid=[]
   if case=='duplicate_clock':
    valid.append(send(req));req['time']+=.25;valid.append(send(req));assert all('error' not in r for r in valid)
   elif case=='stale_epoch':req['epoch']=1
   elif case=='nonfinite_robot':req['robot']['vx']=float('nan')
   elif case=='nonfinite_dt':req['dt']=float('nan')
   elif case=='infeasible_robot':req['robot'].update(x=-25.,y=-1.)
   reply=send(req);assert reply.get('fatal') and reply.get('restart_required') and 'error' in reply
   assert stream.readline()==b'','Faulted worker accepted further requests'
   stream.close();client.close();proc.wait(timeout=10);assert proc.returncode==0
   rows=[json.loads(line) for line in (runtime/'worker_trajectory.jsonl').open()];assert 'error' in rows[-1]
   expected=valid[-1]['humans'] if valid else json.loads((runtime/'ready.json').read_text())['humans']
   assert rows[-1]['last_safe_endpoint']==expected
   (out/(case+'.json')).write_text(json.dumps(dict(valid=valid,error_reply=reply,terminal_log=rows[-1],worker_exit_code=proc.returncode),indent=2)+'\n')
   results.append(dict(case=case,status='PASS',valid_updates=len(valid),error=reply['error'],fatal_close=True,last_safe_endpoint_preserved=True))
  finally:
   if subprocess.check_output(['docker','ps','-aq','--filter','name=^/'+name+'$'],text=True).strip():subprocess.run(['docker','stop','-t','3',name],stdout=subprocess.DEVNULL)
   if proc.poll() is None:proc.wait(timeout=10)
(out/'validation.json').write_text(json.dumps(dict(status='PASS',worker_sha256=hashlib.sha256((HERE/'worker.py').read_bytes()).hexdigest(),cases=results),indent=2)+'\n')
print(json.dumps(results,indent=2))
