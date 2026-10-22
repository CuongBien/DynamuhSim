#!/usr/bin/env python3
"""One held-out seed44 capture; sensor tracker stays offline and cannot control robot."""
import datetime,difflib,fcntl,json,os,subprocess,sys
from pathlib import Path
from freeze import verify,sha
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parents[2]
NAME='identity_seed44_01'


def run():
    fixed=verify(HERE/'generated/final_engine.json');pre=json.loads((HERE/'generated/preflight/preflight.json').read_text())
    assert pre['status']=='PASS' and pre['final_engine_sha256']==sha(HERE/'generated/final_engine.json')
    lock=(BENCH/'common_gazebo/generated/domain91.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);fcntl.flock(lock,fcntl.LOCK_UN);lock.close()
    out=HERE/'generated/runs'/NAME;runtime=HERE/'generated/runtime'/NAME
    assert not out.exists() and not runtime.exists();runtime.parent.mkdir(parents=True,exist_ok=True)
    directory=HERE/'generated/drivers'/NAME;directory.mkdir(parents=True,exist_ok=False)
    parent=BENCH/'early_avoidance_pilot/reports/drivers/avoidance_moderate01/driver.py';before=parent.read_text();after=before
    needle="runtime=HERE/'generated'/out.name;runtime.mkdir(exist_ok=False)";assert after.count(needle)==1;after=after.replace(needle,"runtime=Path("+repr(str(runtime.parent))+")/out.name;runtime.mkdir(exist_ok=False)")
    needle="p.add_argument('--seed',type=int,choices=[42,43],required=True)";assert after.count(needle)==1;after=after.replace(needle,"p.add_argument('--seed',type=int,choices=[44],required=True)")
    driver=directory/'driver.py';driver.write_text(after);compile(after,str(driver),'exec');(directory/'changes.diff').write_text(''.join(difflib.unified_diff(before.splitlines(True),after.splitlines(True),fromfile='validated parent driver',tofile='owned runtime and seed44 parser input only')))
    manifest=dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),seed=44,head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),driver_sha256=sha(driver),parent_driver_sha256=sha(parent),
        final_engine_sha256=sha(HERE/'generated/final_engine.json'),engine_sha256=fixed['engine_sha256'],capture_helper_sha256=sha(Path(__file__)),
        protocol=dict(attempts=1,active_sim_s=90,active_wall_s=540,scenario='standing',people=6,camera='mesh depth OFF + ON diagnostic',profile='Same moderate Nav2/native physics; no oracle; same layout; new seed44'),
        source_sha256={str(q.relative_to(REPO)):sha(q) for q in [*[BENCH/'early_avoidance_pilot'/n for n in ['namespace_prepare.py','moderate_config.py','config_runtime.py']],BENCH/'depth_proxy_lifecycle/motion_prepare.py',BENCH/'depth_proxy_lifecycle/quiesce.py',BENCH/'depth_proxy_lifecycle/pair_recording.py',BENCH/'gpu_runtime/runtime_guard.py',BENCH/'depth_proxy_ablation/generated/build_jazzy/libdepth_proxy_filter.so']})
    (directory/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    env=os.environ.copy();env.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1');env['PYTHONPATH']=os.pathsep.join([str(BENCH/'early_avoidance_pilot'),str(BENCH/'depth_proxy_lifecycle'),str(BENCH/'dataset_pilot'),env.get('PYTHONPATH','')])
    with (directory/'console.log').open('w') as log:result=subprocess.run([sys.executable,str(driver),'--scenario','standing','--seed','44','--report-dir',str(out)],stdout=log,stderr=subprocess.STDOUT,env=env)
    print(json.dumps(dict(returncode=result.returncode,seed=44,output=str(out),automatic_retries=False)));return result.returncode
if __name__=='__main__':raise SystemExit(run())
