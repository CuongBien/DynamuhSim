#!/usr/bin/env python3
import difflib,hashlib,json,os,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;PARENT=BENCH/'depth_proxy_lifecycle';DATASET=BENCH/'dataset_pilot'
if __name__=='__main__':
    name='avoidance_moderate01';out=HERE/'generated/runs'/name;destination=HERE/'generated/drivers'/name
    assert not out.exists() and not (DATASET/'generated'/name).exists();destination.mkdir(parents=True,exist_ok=False)
    before=(HERE/'generated/drivers/avoidance_fixed01/driver.py').read_text();needle='from config_runtime import apply_runtime_config,changes,capture_live_parameters';assert before.count(needle)==1
    after=before.replace(needle,'from config_runtime import changes\nfrom moderate_config import apply_runtime_config,capture_live_parameters')
    target=destination/'driver.py';target.write_text(after);compile(after,str(target),'exec')
    (destination/'moderate.diff').write_text(''.join(difflib.unified_diff(before.splitlines(True),after.splitlines(True),fromfile='slow corrected physics',tofile='moderate corrected physics')))
    (destination/'manifest.json').write_text(json.dumps(dict(parent_driver_sha256=hashlib.sha256(before.encode()).hexdigest(),driver_sha256=hashlib.sha256(after.encode()).hexdigest(),protocol=json.loads((HERE/'protocol.json').read_text()),source_sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in HERE.glob('*.py')}),indent=2)+'\n')
    env=os.environ.copy();env['PYTHONPATH']=os.pathsep.join([str(HERE),str(PARENT),str(DATASET),env.get('PYTHONPATH','')])
    with (destination/'console.log').open('w') as log:
        result=subprocess.run([sys.executable,str(target),'--scenario','standing','--seed','42','--report-dir',str(out)],env=env,stdout=log,stderr=subprocess.STDOUT)
    print(json.dumps(dict(returncode=result.returncode,scope='Correct native robot + moderate speed early avoidance',output=str(out))),flush=True)
    raise SystemExit(result.returncode)
