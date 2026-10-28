#!/usr/bin/env python3
import argparse,json,re,subprocess,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from freeze import verify,sha
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parents[2]

import sys
sys.path.insert(0,str(REPO/'python'))
from dynamuhsim.analysis.evidence import verify_sources
ARTIFACT_ROOT=REPO
def sources(expected):return verify_sources(expected,repository=REPO,roots=[ARTIFACT_ROOT],revision='9833565fa546d55cb0e553f8388bf4a703c59583')


def rows(p):return [json.loads(l) for l in p.read_text().splitlines()]
def canonical(e):return [e.tag,sorted(e.attrib.items()),(e.text or '').strip(),[canonical(c) for c in e]]

def audit(output):
    output.mkdir(parents=True,exist_ok=False);root=HERE/'generated/runs/identity_seed44_01';runtime=HERE/'generated/runtime/identity_seed44_01'
    r=json.loads((root/'result.json').read_text());manifest=json.loads((HERE/'generated/drivers/identity_seed44_01/manifest.json').read_text());fixed=verify(HERE/'generated/final_engine.json')
    assert manifest['engine_sha256']==fixed['engine_sha256'] and manifest['final_engine_sha256']==sha(HERE/'generated/final_engine.json')
    sources({str(HERE.relative_to(ARTIFACT_ROOT)/'capture44.py'):manifest['capture_helper_sha256'],**manifest['source_sha256']})
    unchanged=bool(sources(r['source_sha256']));ready=json.loads((runtime/'ready.json').read_text())
    check=json.loads(subprocess.check_output([str(BENCH/'early_avoidance_pilot/generated/check_friction'),str(runtime/'mecanum_camera.sdf')],text=True))
    expected=BENCH/'portable_episode/generated/episode01/config_snapshot/runtime';worlds=[ET.parse(p).getroot() for p in [expected/'warehouse.sdf',runtime/'warehouse.sdf']]
    for tree in worlds:
        for n in tree.iter('socket'):n.text='OWNED_SOCKET'
    parity=canonical(worlds[0])==canonical(worlds[1]);events=rows(root/'events.jsonl');em={r['event']:r['stamp_ns'] for r in events};tail=(em.get('observation_end',0)-em.get('action_end',0))/1e9
    q=r.get('server_shutdown',{}).get('quiesce',{});log=(root/'launch.log').read_text(errors='replace');balanced=re.search(r'teardown pre=(\d+) post=(\d+)',log)
    old={v['native_step']:v for v in rows(BENCH/'depth_seed43_validation/generated/runtime/depth_seed43_01/worker_trajectory.jsonl') if v.get('active_phase') and v.get('native_step',0)>0}
    current={v['native_step']:v for v in rows(runtime/'worker_trajectory.jsonl') if v.get('active_phase') and v.get('native_step',0)>0};differences=[]
    for step in sorted(old.keys()&current.keys()):
        a={h['id']:h for h in old[step]['humans']};b={h['id']:h for h in current[step]['humans']};d=max(np.hypot(a[k]['x']-b[k]['x'],a[k]['y']-b[k]['y']) for k in a.keys()&b.keys());differences.append((step,float(d)))
    distinct=[s for s,d in differences if d>1e-6]
    checks=dict(measured=r['status']=='MEASURED',seed44=r['seed']==ready['seed']==44,no_adapter_fault=not r['faults'],recorder_clean=r.get('recording',{}).get('returncode')==0,source_unchanged=unchanged,
        native_friction=check['recognized_gz_base_link_frames']==4,camera_exact=sha(runtime/'mecanum_camera.sdf')==sha(expected/'mecanum_camera.sdf'),nav2_exact=sha(runtime/'nav2_runtime.yaml')==sha(expected/'nav2_runtime.yaml'),world_exact_except_owned_socket=parity,
        actual_gpu_nvidia=any('NVIDIA' in s for s in r['cleanup'].get('ogre_renderer_lines',[])),complete_tail=tail>=3,events_complete=[e['seq'] for e in events]==[1,2,3,4],callbacks_balanced=bool(balanced and balanced[1]==balanced[2]),
        clean_shutdown=r['gazebo_clean_exit'] and r['cleanup']['status']=='PASS' and not r['cleanup']['remaining'] and q.get('pause_acknowledged') and q.get('simulation_time_stable'))
    result=dict(status='PASS' if all(checks.values()) else 'FAIL',checks=checks,seed=44,actual_goal_reached=r.get('actual_goal_reached'),navigation_status=r.get('navigation_status'),active_sim_s=r.get('active_sim_seconds'),goal_xy_error_m=r.get('goal_xy_error_world_m'),goal_yaw_error_rad=r.get('goal_yaw_error_world_rad'),label_tail_s=tail,
        renderer=r['cleanup'].get('ogre_renderer_lines'),first_human_xy_difference_vs_seed43_step=distinct[0] if distinct else None,max_xy_difference_vs_seed43_m=max((d for s,d in differences),default=None),
        diversity_caveat='Same initial layout; coupled robot inputs also differ, not isolated RNG effect',frozen_before_capture=True,training_ready=False)
    (output/'capture_audit.json').write_text(json.dumps(result,indent=2)+'\n');inventory={p.name:dict(bytes=p.stat().st_size,sha256=sha(p)) for p in (root/'raw_bag').iterdir() if p.is_file()};(output/'closed_raw_inventory.json').write_text(json.dumps(inventory,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--artifact-root',type=Path,default=REPO);a=p.parse_args();ARTIFACT_ROOT=a.artifact_root.resolve();HERE=ARTIFACT_ROOT/HERE.relative_to(REPO);BENCH=HERE.parent;audit(a.output.resolve())
