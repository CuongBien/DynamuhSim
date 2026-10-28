#!/usr/bin/env python3
"""Audit one new seed capture separately from geometry and semantic acceptance."""
import argparse,json,re,subprocess,sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parents[2]
sys.path.insert(0,str(BENCH/'depth_track_refinement'))
from freeze import sha,verify

import sys
sys.path.insert(0,str(REPO/'python'))
from dynamuhsim.analysis.evidence import verify_sources
ARTIFACT_ROOT=REPO
def sources(expected):return verify_sources(expected,repository=REPO,roots=[ARTIFACT_ROOT],revision='9833565fa546d55cb0e553f8388bf4a703c59583')


def rows(p):return [json.loads(l) for l in p.read_text().splitlines()]
def audit(output):
    output.mkdir(parents=True,exist_ok=False)
    root=HERE/'generated/runs/depth_seed43_01';runtime=HERE/'generated/runtime/depth_seed43_01'
    r=json.loads((root/'result.json').read_text());capture=json.loads((HERE/'generated/drivers/depth_seed43_01/manifest.json').read_text())
    fixed=verify(BENCH/'depth_track_refinement/reports/frozen01.json');protocol=json.loads((HERE/'protocol.json').read_text())
    assert capture['inference_engine_sha256']==fixed['engine_sha256'] and capture['inference_config']==fixed['config']
    assert capture['protocol']==protocol;sources(capture['helper_sha256'])
    assert sha(HERE/'generated/drivers/depth_seed43_01/driver.py')==capture['driver_sha256']
    source_unchanged=bool(sources(r['source_sha256']))
    ready=json.loads((runtime/'ready.json').read_text())
    friction=json.loads(subprocess.check_output([str(BENCH/'early_avoidance_pilot/generated/check_friction'),str(runtime/'mecanum_camera.sdf')],text=True))
    expected=BENCH/'portable_episode/generated/episode01/config_snapshot/runtime'
    camera_equal=sha(runtime/'mecanum_camera.sdf')==sha(expected/'mecanum_camera.sdf')
    nav_equal=sha(runtime/'nav2_runtime.yaml')==sha(expected/'nav2_runtime.yaml')
    live_equal=json.loads((root/'live_parameters.json').read_text())==json.loads((BENCH/'early_avoidance_pilot/reports/avoidance_moderate01/live_parameters.json').read_text())
    q=r.get('server_shutdown',{}).get('quiesce',{})
    events=rows(root/'events.jsonl');em={e['event']:e for e in events};tail=(em.get('observation_end',{}).get('stamp_ns',0)-em.get('action_end',{}).get('stamp_ns',0))/1e9
    log=(root/'launch.log').read_text(errors='replace');m=re.search(r'teardown pre=(\d+) post=(\d+)',log)
    actual_gpu=any('NVIDIA' in line for line in r['cleanup'].get('ogre_renderer_lines',[]))
    old=rows(BENCH/'dataset_pilot/generated/avoidance_moderate01/worker_trajectory.jsonl');new=rows(runtime/'worker_trajectory.jsonl')
    def bystep(seq):return {x['native_step']:x for x in seq if x.get('active_phase') and x.get('native_step',0)>0 and 'humans' in x}
    a,b=bystep(old),bystep(new);diff=[]
    for step in sorted(a.keys()&b.keys()):
        first={h['id']:h for h in a[step]['humans']};second={h['id']:h for h in b[step]['humans']}
        d=max(np.hypot(first[k]['x']-second[k]['x'],first[k]['y']-second[k]['y']) for k in first.keys()&second.keys())
        diff.append((step,float(d)))
    diverged=[s for s,d in diff if d>1e-6]
    pair=json.loads((root/'paired_depth/index.json').read_text());common=sorted(set(pair['on'])&set(pair['off']),key=int);active=0;changed=0
    for stamp in common:
        if pair['off'][stamp]['phase']!='active':continue
        active+=1;on=np.load(root/'paired_depth/on'/(stamp+'.npy'),allow_pickle=False);off=np.load(root/'paired_depth/off'/(stamp+'.npy'),allow_pickle=False)
        good=np.isfinite(on)&np.isfinite(off);delta=np.zeros_like(on);np.subtract(off,on,out=delta,where=good)
        if np.sum((good&(abs(delta)>.02))|(np.isfinite(on)!=np.isfinite(off)))>=100:changed+=1
    checks=dict(measured=r['status']=='MEASURED',seed43=r['seed']==ready['seed']==43,no_adapter_fault=not r['faults'],recorder_clean=r.get('recording',{}).get('returncode')==0,
        source_unchanged=source_unchanged,original_friction=friction['recognized_gz_base_link_frames']==4,camera_exact_mesh_profile=camera_equal,
        nav2_profile_exact=nav_equal and live_equal,actual_nvidia_renderer=actual_gpu,tail_at_least_3s=tail>=3,
        events_complete=[e['seq'] for e in events]==[1,2,3,4],render_callbacks_balanced=bool(m and m[1]==m[2] and int(m[1])>0),
        paired_active_depth=active>=50 and changed>0,clean_shutdown=r['gazebo_clean_exit'] and r['cleanup']['status']=='PASS' and not r['cleanup']['remaining'] and q.get('pause_acknowledged') and q.get('simulation_time_stable'))
    result=dict(status='PASS' if all(checks.values()) else 'FAIL',checks=checks,seed=43,protocol_sha256=sha(HERE/'protocol.json'),
        capture_manifest_sha256=sha(HERE/'generated/drivers/depth_seed43_01/manifest.json'),native_friction=friction,
        runtime_sha256={n:sha(runtime/n) for n in ['mecanum_camera.sdf','warehouse.sdf','nav2_runtime.yaml']},
        navigation=dict(status=r.get('navigation_status'),actual_goal_reached=r.get('actual_goal_reached'),active_sim_seconds=r.get('active_sim_seconds'),goal_xy_error_m=r.get('goal_xy_error_world_m'),goal_yaw_error_rad=r.get('goal_yaw_error_world_rad')),
        complete_label_tail_s=tail,renderer_lines=r['cleanup'].get('ogre_renderer_lines',[]),paired_frames=len(common),active_paired_frames=active,active_different_on_off_frames=changed,
        seed_diversity=dict(common_native_steps=len(diff),first_human_xy_difference_native_step=diverged[0] if diverged else None,max_human_xy_difference_m=max((d for s,d in diff),default=None),
            observed_trajectory_diverged=bool(diverged),scope='New capture/seed with same explicit initial groups; coupled robot inputs also differ, not isolated RNG causal attribution'),
        semantic_person_validated=False,association_accepted=False,training_ready=False,hu_nav_sim_used=False)
    (output/'capture_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    rawfiles={p.name:dict(bytes=p.stat().st_size,sha256=sha(p)) for p in (root/'raw_bag').iterdir() if p.is_file()}
    (output/'closed_raw_inventory.json').write_text(json.dumps(rawfiles,indent=2)+'\n');print(json.dumps(result,indent=2));return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--artifact-root',type=Path,default=REPO);a=p.parse_args();ARTIFACT_ROOT=a.artifact_root.resolve();HERE=ARTIFACT_ROOT/HERE.relative_to(REPO);BENCH=HERE.parent;audit(a.output.resolve())
