#!/usr/bin/env python3
import json,subprocess,sys
from pathlib import Path
from freeze import verify,sha
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parents[2]
sys.path[:0]=[str(BENCH/'early_avoidance_pilot'),str(BENCH/'depth_proxy_lifecycle'),str(BENCH/'gpu_runtime')]
from namespace_prepare import prepare_camera
from runtime_guard import gpu_preflight
if __name__=='__main__':
    verify(HERE/'generated/final_engine.json');out=HERE/'generated/preflight';out.mkdir(parents=True,exist_ok=False)
    prepare_camera(out,REPO);gpu=gpu_preflight(out);check=json.loads(subprocess.check_output([str(BENCH/'early_avoidance_pilot/generated/check_friction'),str(out/'mecanum_camera.sdf')],text=True))
    assert check['recognized_gz_base_link_frames']==4 and sha(out/'mecanum_camera.sdf')=='14c15412664eb37dd2f0fb2d61a79b09f98722f204f97d97212f0ec814ad3d9f'
    result=dict(status='PASS',gpu=gpu,native_friction=check,final_engine_sha256=sha(HERE/'generated/final_engine.json'))
    (out/'preflight.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(status='PASS',renderer=gpu['renderer'],friction=check)))
