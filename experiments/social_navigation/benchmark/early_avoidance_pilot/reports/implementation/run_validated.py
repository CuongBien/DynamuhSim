#!/usr/bin/env python3
"""Public experiment runner: preserves original friction prefix, no oracle defaults."""
import argparse,difflib,hashlib,json,os,subprocess,sys
from pathlib import Path
from trial import prepare,HERE,PARENT,DATASET
def build_driver(name,profile):
    assert name.replace('_','').replace('-','').isalnum(),'Simple unique name required'
    out=HERE/'generated/runs'/name;assert not out.exists() and not (DATASET/'generated'/name).exists()
    driver=prepare(name);before=driver.read_text();after=before.replace('from motion_prepare import prepare_camera,add_filter','from namespace_prepare import prepare_camera,add_filter')
    assert before!=after
    if profile=='moderate':
        needle='from config_runtime import apply_runtime_config,changes,capture_live_parameters';assert after.count(needle)==1
        after=after.replace(needle,'from config_runtime import changes\nfrom moderate_config import apply_runtime_config,capture_live_parameters')
    compile(after,str(driver),'exec');driver.write_text(after)
    (driver.parent/'validated.diff').write_text(''.join(difflib.unified_diff(before.splitlines(True),after.splitlines(True),fromfile='base preparation',tofile='preserved native physics runtime')))
    (driver.parent/'actual_manifest.json').write_text(json.dumps(dict(profile=profile,oracle=False,driver_sha256=hashlib.sha256(after.encode()).hexdigest(),protocol=json.loads((HERE/'protocol.json').read_text())),indent=2)+'\n')
    return driver

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--name',required=True);p.add_argument('--profile',choices=['slow','moderate'],default='moderate');a=p.parse_args()
    driver=build_driver(a.name,a.profile);out=HERE/'generated/runs'/a.name
    env=os.environ.copy();env['PYTHONPATH']=os.pathsep.join([str(HERE),str(PARENT),str(DATASET),env.get('PYTHONPATH','')])
    with (driver.parent/'console.log').open('w') as log:
        result=subprocess.run([sys.executable,str(driver),'--scenario','standing','--seed','42','--report-dir',str(out)],env=env,stdout=log,stderr=subprocess.STDOUT)
    print(json.dumps(dict(returncode=result.returncode,output=str(out),oracle=False)),flush=True);raise SystemExit(result.returncode)
