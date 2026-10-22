#!/usr/bin/env python3
"""One oracle robot-pose control after the failed original odometry trial."""
import difflib,json,os,subprocess,sys
from pathlib import Path
from trial import prepare,HERE,PARENT,DATASET
if __name__=='__main__':
    name='avoidance_truth01';out=HERE/'generated/runs'/name
    assert not out.exists() and not (DATASET/'generated'/name).exists()
    failed=json.loads((HERE/'reports/avoidance01/summary.json').read_text());assert failed['status']=='FAIL'
    target=prepare(name);before=target.read_text();after=before
    edits={
      'from motion_prepare import prepare_camera,add_filter':'from oracle_prepare import prepare_camera,add_filter',
      'apply_runtime_config(config);':'apply_runtime_config(config);config[\'amcl\'][\'ros__parameters\'][\'tf_broadcast\']=False;',
      "    rclpy.init();node=Node(":"    truth_log=(out/'truth_odometry.log').open('w');files.append(truth_log);processes.append(subprocess.Popen([sys.executable,str(Path(os.environ['EARLY_AVOIDANCE_DIR'])/'truth_odometry.py')],stdout=truth_log,stderr=subprocess.STDOUT,start_new_session=True,env=launch_env))\n    rclpy.init();node=Node(",
    }
    for needle,replacement in edits.items():assert after.count(needle)==1;after=after.replace(needle,replacement)
    compile(after,str(target),'exec');target.write_text(after)
    (target.parent/'oracle.diff').write_text(''.join(difflib.unified_diff(before.splitlines(True),after.splitlines(True),fromfile='same Nav2 variant',tofile='explicit robot-truth odometry control')))
    env=os.environ.copy();env['PYTHONPATH']=os.pathsep.join([str(HERE),str(PARENT),str(DATASET),env.get('PYTHONPATH','')]);env['EARLY_AVOIDANCE_DIR']=str(HERE)
    with (target.parent/'console.log').open('w') as log:
        result=subprocess.run([sys.executable,str(target),'--scenario','standing','--seed','42','--report-dir',str(out)],env=env,stdout=log,stderr=subprocess.STDOUT)
    print(json.dumps(dict(returncode=result.returncode,scope='Explicit robot-truth odometry control',output=str(out))),flush=True)
    raise SystemExit(result.returncode)
