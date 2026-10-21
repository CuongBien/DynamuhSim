#!/usr/bin/env python3
"""Reproduce the accepted standalone contract in the existing pinned container."""
import argparse,hashlib,json,os,subprocess,time,uuid
from pathlib import Path
HERE=Path(__file__).resolve().parent
IMAGE='sha256:edfd591ea7551813419e511b341089ad956b6d3e4c1bd395a326035865ef1d5d'
p=argparse.ArgumentParser();p.add_argument('--report-name',required=True);p.add_argument('--timeout',type=float,default=120);a=p.parse_args()
if not a.report_name.replace('_','').replace('-','').isalnum():p.error('Use a simple new report name')
if (HERE/'reports'/a.report_name).exists():p.error('Report already exists; choose a new name')
(HERE/'generated').mkdir(exist_ok=True)
(HERE/'generated/layout.json').write_bytes((HERE/'../../warehouse/layout.json').resolve().read_bytes())
subprocess.run(['docker','image','inspect',IMAGE],stdout=subprocess.DEVNULL,check=True)
name='dynamuhsim-complexity-contract-'+uuid.uuid4().hex[:10]
cmd=['docker','run','--rm','--name',name,'--network','none','--cpus','2','--memory','4g','--user',f'{os.getuid()}:{os.getgid()}',
 '-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','MKL_NUM_THREADS=1','-v',f'{HERE}:/work',IMAGE,'python','/work/probe.py','--report-dir',f'/work/reports/{a.report_name}']
start=time.monotonic();status='ERROR';code=None
try:
 result=subprocess.run(cmd,text=True,capture_output=True,timeout=a.timeout);code=result.returncode
 (HERE/'generated'/(a.report_name+'.log')).write_text(result.stdout+result.stderr)
 status='COMPLETED' if code==0 else 'PROBE_ERROR'
 print(result.stdout);print(result.stderr)
finally:
 # Only this UUID-owned container can be stopped; no foreign runtime cleanup.
 listed=subprocess.check_output(['docker','ps','-aq','--filter','name=^/'+name+'$'],text=True).strip()
 if listed:subprocess.run(['docker','stop','-t','5',name],stdout=subprocess.DEVNULL,check=False)
 record=dict(status=status,command=cmd,exit_code=code,wall_seconds=time.monotonic()-start)
 (HERE/'reports'/(a.report_name+'_execution.json')).write_text(json.dumps(record,indent=2)+'\n')
if code:raise SystemExit(code)
