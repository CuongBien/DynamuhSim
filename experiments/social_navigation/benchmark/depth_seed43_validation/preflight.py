#!/usr/bin/env python3
"""Validate GPU/native friction semantics before the single new capture."""
import json,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parents[2]
sys.path[:0]=[str(BENCH/'early_avoidance_pilot'),str(BENCH/'depth_proxy_lifecycle'),str(BENCH/'depth_track_refinement')]
from namespace_prepare import prepare_camera
sys.path.insert(0,str(REPO/'python'));from dynamuhsim.runtime.gpu import gpu_preflight
from freeze import verify,sha
if __name__=='__main__':
    out=HERE/'generated/preflight';out.mkdir(parents=True,exist_ok=False)
    camera=prepare_camera(out,REPO);gpu=gpu_preflight(out)
    expected='14c15412664eb37dd2f0fb2d61a79b09f98722f204f97d97212f0ec814ad3d9f'
    assert sha(out/'mecanum_camera.sdf')==expected
    check=subprocess.run([str(BENCH/'early_avoidance_pilot/generated/check_friction'),str(out/'mecanum_camera.sdf')],capture_output=True,text=True,check=True)
    friction=json.loads(check.stdout);assert friction['recognized_gz_base_link_frames']==4
    frozen=verify(BENCH/'depth_track_refinement/reports/frozen01.json')
    result=dict(status='PASS',gpu=gpu,native_friction=friction,camera_sha256=expected,
        protocol_sha256=sha(HERE/'protocol.json'),inference_engine_sha256=frozen['engine_sha256'],
        camera_metadata=camera,source_robot_unchanged=True)
    (out/'preflight.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(status='PASS',renderer=gpu['renderer'],friction=friction,camera_sha256=expected)))
