#!/usr/bin/env python3
"""Run the pinned geometry variant sequentially, with no ROS/system changes."""
import argparse,json,os,subprocess,time,uuid
from pathlib import Path
HERE=Path(__file__).resolve().parent
IMAGE='sha256:edfd591ea7551813419e511b341089ad956b6d3e4c1bd395a326035865ef1d5d'
p=argparse.ArgumentParser();p.add_argument('--report-name',required=True);p.add_argument('--timeout',type=float,default=240);a=p.parse_args()
if not a.report_name.replace('_','').replace('-','').isalnum():p.error('Use a simple new report name')
if (HERE/'reports'/a.report_name).exists():p.error('Existing report will not be overwritten')
(HERE/'generated').mkdir(exist_ok=True);(HERE/'reports').mkdir(exist_ok=True)
for source,name in [(HERE.parent/'complexitynav_contract/kernel.py','native_contract.py'),(HERE.parent/'socnavgym_warehouse_lifecycle/geometry.py','swept.py'),(HERE/'../../warehouse/layout.json','layout.json')]:
 (HERE/'generated'/name).write_bytes(source.resolve().read_bytes())
subprocess.run(['docker','image','inspect',IMAGE],stdout=subprocess.DEVNULL,check=True)
name='dynamuhsim-complexity-geometry-'+uuid.uuid4().hex[:10]
command=['docker','run','--rm','--name',name,'--network','none','--cpus','2','--memory','4g','--user',f'{os.getuid()}:{os.getgid()}',
 '-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','MKL_NUM_THREADS=1','-v',f'{HERE}:/work',IMAGE,'python','/work/probe.py','--report-dir',f'/work/reports/{a.report_name}']
start=time.monotonic();result=None
try:
 result=subprocess.run(command,text=True,capture_output=True,timeout=a.timeout)
 (HERE/'generated'/(a.report_name+'.log')).write_text(result.stdout+result.stderr)
 print(result.stdout);print(result.stderr)
finally:
 remaining=subprocess.check_output(['docker','ps','-aq','--filter','name=^/'+name+'$'],text=True).strip()
 if remaining:subprocess.run(['docker','stop','-t','5',name],stdout=subprocess.DEVNULL,check=False)
 (HERE/'reports'/(a.report_name+'_execution.json')).write_text(json.dumps(dict(command=command,returncode=result.returncode if result else None,wall_seconds=time.monotonic()-start),indent=2)+'\n')
if result and result.returncode:raise SystemExit(result.returncode)
