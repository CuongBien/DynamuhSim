#!/usr/bin/env python3
"""One new seed recording with the preserved validated driver; no ROS source edits."""
import argparse,datetime,difflib,fcntl,json,os,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parents[2]
sys.path.insert(0,str(BENCH/'depth_track_refinement'))
from freeze import sha,verify


def prepare():
    protocol=json.loads((HERE/'protocol.json').read_text());name=protocol['run']
    driver_dir=HERE/'generated/drivers'/name;driver_dir.mkdir(parents=True,exist_ok=False)
    (HERE/'generated/runtime').mkdir(exist_ok=True)
    parent=BENCH/'early_avoidance_pilot/reports/drivers/avoidance_moderate01/driver.py'
    before=parent.read_text();needle="runtime=HERE/'generated'/out.name;runtime.mkdir(exist_ok=False)"
    replacement="runtime=Path("+repr(str(HERE/'generated/runtime'))+")/out.name;runtime.mkdir(exist_ok=False)"
    assert before.count(needle)==1;after=before.replace(needle,replacement)
    driver=driver_dir/'driver.py';driver.write_text(after);compile(after,str(driver),'exec')
    (driver_dir/'runtime_path_only.diff').write_text(''.join(difflib.unified_diff(before.splitlines(True),after.splitlines(True),fromfile='accepted moderate seed42 capture driver',tofile='new stage owned runtime location')))
    original=BENCH/'depth_track_refinement/reports/frozen01.json';fixed=verify(original)
    manifest=dict(schema='new_seed_capture_sources/1',created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        protocol=protocol,parent_driver_sha256=sha(parent),driver_sha256=sha(driver),parent_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        driver_change='Runtime output location only; seed43 passed as CLI argument',
        inference_frozen_sha256=sha(original),inference_engine_sha256=fixed['engine_sha256'],inference_config=fixed['config'],
        helper_sha256={str(p.relative_to(REPO)):sha(p) for p in [HERE/'capture.py',HERE/'protocol.json',*[BENCH/'early_avoidance_pilot'/n for n in ['namespace_prepare.py','moderate_config.py','config_runtime.py']],BENCH/'depth_proxy_lifecycle/motion_prepare.py',BENCH/'depth_proxy_lifecycle/quiesce.py',BENCH/'depth_proxy_lifecycle/pair_recording.py',BENCH/'gpu_runtime/runtime_guard.py',BENCH/'depth_proxy_ablation/generated/build_jazzy/libdepth_proxy_filter.so']})
    (driver_dir/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');return driver,manifest


def run():
    preflight=json.loads((HERE/'generated/preflight/preflight.json').read_text())
    assert preflight['status']=='PASS' and preflight['protocol_sha256']==sha(HERE/'protocol.json')
    assert preflight['native_friction']['recognized_gz_base_link_frames']==4 and preflight['gpu']['hardware_verified']
    # Probe ownership lock without taking over a different user's experiment.
    lock=(BENCH/'common_gazebo/generated/domain91.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);fcntl.flock(lock,fcntl.LOCK_UN);lock.close()
    driver,manifest=prepare();name=manifest['protocol']['run'];out=HERE/'generated/runs'/name
    assert not out.exists() and not (HERE/'generated/runtime'/name).exists()
    env=os.environ.copy();env.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
    env['PYTHONPATH']=os.pathsep.join([str(BENCH/'early_avoidance_pilot'),str(BENCH/'depth_proxy_lifecycle'),str(BENCH/'dataset_pilot'),env.get('PYTHONPATH','')])
    cmd=[sys.executable,str(driver),'--scenario','standing','--seed','43','--report-dir',str(out)]
    with (driver.parent/'console.log').open('w') as log:result=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
    print(json.dumps(dict(returncode=result.returncode,output=str(out),console=str(driver.parent/'console.log'),automatic_retries=False)),flush=True)
    return result.returncode
if __name__=='__main__':raise SystemExit(run())
