#!/usr/bin/env python3
"""Generate a bounded runtime copy; parent experiments and their evidence stay immutable."""
import argparse,difflib,hashlib,json,os,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;PARENT=BENCH/'depth_proxy_lifecycle';DATASET=BENCH/'dataset_pilot'

def prepare(name):
    destination=HERE/'generated/drivers'/name;destination.mkdir(parents=True,exist_ok=False)
    before=(PARENT/'reports/implementation/motion_driver.py').read_text();after=before
    edits={
      'from quiesce import stop_paused':'from quiesce import stop_paused\nfrom config_runtime import apply_runtime_config,changes,capture_live_parameters',
      "plan=json.loads((BASE/'protocol.json').read_text());":"plan=json.loads((BASE/'protocol.json').read_text());plan['sim_budget']=90.;",
      "    (runtime/'nav2_runtime.yaml').write_text":"    before_treatment=copy.deepcopy(config);apply_runtime_config(config);(out/'config_changes.json').write_text(json.dumps(changes(before_treatment,config),indent=2)+'\\n')\n    (runtime/'nav2_runtime.yaml').write_text",
      "    recording.event('ready'":"    result['live_parameters']=capture_live_parameters(node,wait,out)\n    recording.event('ready'",
      "result['experiment']='Depth-only proxy motion pilot, native camera height, no default changes';":"result['experiment']='Conservative Nav2 footprint/inflation/speed pilot; native sensors/standingv3 unchanged';",
    }
    for needle,replacement in edits.items():
        assert after.count(needle)==1,(needle,after.count(needle));after=after.replace(needle,replacement)
    target=destination/'driver.py';target.write_text(after);compile(after,str(target),'exec')
    (destination/'driver.diff').write_text(''.join(difflib.unified_diff(before.splitlines(True),after.splitlines(True),fromfile='parent frozen motion_driver.py',tofile='runtime early avoidance driver')))
    manifest=dict(parent_sha256=hashlib.sha256(before.encode()).hexdigest(),driver_sha256=hashlib.sha256(after.encode()).hexdigest(),source_sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in HERE.glob('*.py')},protocol=json.loads((HERE/'protocol.json').read_text()))
    (destination/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');return target

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--name',default='avoidance01');a=p.parse_args()
    assert a.name.replace('_','').isalnum(),'Simple unique run name required'
    out=HERE/'generated/runs'/a.name
    assert not out.exists() and not (DATASET/'generated'/a.name).exists(),'Preserve all prior evidence'
    driver=prepare(a.name);env=os.environ.copy();env['PYTHONPATH']=os.pathsep.join([str(HERE),str(PARENT),str(DATASET),env.get('PYTHONPATH','')])
    cmd=[sys.executable,str(driver),'--scenario','standing','--seed','42','--report-dir',str(out)]
    with (driver.parent/'console.log').open('w') as log:
        result=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
    print(json.dumps(dict(returncode=result.returncode,output=str(out),console=str(driver.parent/'console.log'))),flush=True)
    raise SystemExit(result.returncode)
