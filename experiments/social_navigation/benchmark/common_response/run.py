#!/usr/bin/env python3
"""Sequential locked-input cohort; each backend uses its existing isolated runtime."""
import argparse,hashlib,json,os,subprocess,time,uuid
from pathlib import Path
HERE=Path(__file__).resolve().parent;SOCIAL=HERE.parent.parent;REPO=SOCIAL.parent.parent
p=argparse.ArgumentParser();p.add_argument('--report-name',required=True);a=p.parse_args()
if not a.report_name.replace('_','').replace('-','').isalnum():p.error('Use a simple new name')
out=HERE/'reports'/a.report_name;out.mkdir(parents=True,exist_ok=False)
protocol=json.loads((HERE/'inputs/protocol.json').read_text());images=json.loads((HERE/'inputs/images.json').read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for name,h in protocol['input_sha256'].items():assert sha(HERE/name)==h,name
snap=out/'source_snapshot';snap.mkdir()
for file in [HERE/'model_worker.py',Path(__file__),HERE/'prepare.py',HERE/'generated/crowd_kernel_n6.py']:(snap/file.name).write_bytes(file.read_bytes())
source=[*snap.glob('*'),SOCIAL/'benchmark/socnavgym_warehouse_lifecycle/kernel.py',SOCIAL/'benchmark/socnavgym_warehouse_lifecycle/motion.py',SOCIAL/'benchmark/socnavgym_warehouse_lifecycle/geometry.py',SOCIAL/'benchmark/socnavgym_lifecycle_fix/continuous_groups.py',SOCIAL/'benchmark/complexitynav_geometry/kernel.py',SOCIAL/'benchmark/complexitynav_geometry/roadmap.py',SOCIAL/'benchmark/complexitynav_contract/kernel.py',SOCIAL/'warehouse/layout.json']
(out/'protocol.json').write_text(json.dumps(dict(**protocol,head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),source_sha256={str(p.relative_to(REPO)):sha(p) for p in source},images=images),indent=2)+'\n')
results=[]
for name,settings in protocol['models'].items():
    target=out/name;container=None;start=time.monotonic();cmd=[]
    if settings['runtime']=='venv':cmd=[str(SOCIAL/'.venv-socnavgym/bin/python'),str(HERE/'model_worker.py'),'--model',name,'--report-dir',str(target)]
    else:
        container='dynamuhsim-common-'+uuid.uuid4().hex[:10];image=images['dynamuhsim-crowdsimplus:round2' if settings['runtime']=='crowd' else 'dynamuhsim-complexitynav:round3']
        cmd=['docker','run','--rm','--name',container,'--network','none','--cpus','2','--memory','4g','--user',f'{os.getuid()}:{os.getgid()}','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','MKL_NUM_THREADS=1','-v',f'{HERE}:/work','-v',f'{SOCIAL/"benchmark/complexitynav_geometry"}:/geometry:ro',image,'python','/work/model_worker.py','--model',name,'--report-dir','/work/reports/'+a.report_name+'/'+name]
    status='ERROR';code=None
    try:
        result=subprocess.run(cmd,capture_output=True,text=True,timeout=300);code=result.returncode
        (out/(name+'.log')).write_text(result.stdout+result.stderr);status='FINISHED' if code==0 else 'PROCESS_ERROR'
    except subprocess.TimeoutExpired as exc:
        status='TIMEOUT';(out/(name+'.log')).write_bytes((exc.stdout or b'')+(exc.stderr or b''))
    finally:
        if container and subprocess.check_output(['docker','ps','-aq','--filter','name=^/'+container+'$'],text=True).strip():subprocess.run(['docker','stop','-t','3',container],capture_output=True)
    row=dict(model=name,status=status,exit_code=code,wall_seconds=time.monotonic()-start,command=cmd);results.append(row)
    (out/'execution.json').write_text(json.dumps(results,indent=2)+'\n');print(json.dumps(row),flush=True)
print('DONE: all6 models attempted sequentially; no outcome replaced')
