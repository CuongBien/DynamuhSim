#!/usr/bin/env python3
"""Publish compact review artifacts while retaining all original FAIL outcomes."""
import json,shutil,sys,xml.etree.ElementTree as ET
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent
sys.path.insert(0,str(BENCH/'depth_track_refinement'))
from freeze import sha,verify


def finalize():
    reports=HERE/'reports';root=HERE/'generated/runs/depth_seed43_01';runtime=HERE/'generated/runtime/depth_seed43_01';norm=root/'normalized'
    capture=json.loads((reports/'capture/capture_audit.json').read_text());raw=json.loads((reports/'raw_audit.json').read_text());audit=json.loads((reports/'audit01/audit.json').read_text());evaluation=json.loads((reports/'new_seed_evaluation.json').read_text())
    assert capture['status']==raw['status']==audit['status']=='PASS'
    assert not evaluation['comparison']['geometric_improvement'] and not capture['navigation']['actual_goal_reached']
    verify(HERE/'generated/frozen_seed43.json')
    copied={}
    def keep(source,relative):
        target=reports/relative;target.parent.mkdir(parents=True,exist_ok=True)
        if target.exists():assert sha(target)==sha(source)
        else:shutil.copyfile(source,target)
        copied[relative]=dict(bytes=target.stat().st_size,sha256=sha(target))
    for name in ['result.json','events.jsonl','live_parameters.json','config_changes.json','initial.png','middle.png','final.png','camera_preview_4x.gif']:
        if (root/name).exists():keep(root/name,'capture/'+name)
    for name in ['driver.py','manifest.json','runtime_path_only.diff']:
        keep(HERE/'generated/drivers/depth_seed43_01'/name,'driver/'+name)
    for name in ['ready.json','rendering.json','ogre2_session.log','quiesce_shutdown.json','nav2_runtime.yaml']:
        keep(runtime/name,'runtime/'+name)
    for name in ['qa.json','manifest.json','arrays.json','calibration.json','lidar_geometry.json','sample_example.json','contract.json']:
        keep(norm/name,'normalized/'+name)
    keep(HERE/'generated/frozen_seed43.json','frozen_seed43.json')
    keep(HERE/'generated/sensor01/replay.json','sensor_replay.json')
    keep(HERE/'generated/preflight/preflight.json','preflight.json')
    def canon(e):return [e.tag,sorted(e.attrib.items()),(e.text or '').strip(),[canon(c) for c in e]]
    roots=[ET.parse(p).getroot() for p in [BENCH/'portable_episode/generated/episode01/config_snapshot/runtime/warehouse.sdf',runtime/'warehouse.sdf']]
    for tree in roots:
        for n in tree.iter('socket'):n.text='OWNED_SOCKET'
    assert canon(roots[0])==canon(roots[1])
    acceptance=dict(schema='new_seed_acceptance/1',recorded_episodes=1,seed=43,split='internal_validation_only',depth_frames=raw['depth_frames_bitwise_equal'],windows=raw['samples'],
        capture_data_gate='PASS',raw_integrity='PASS',metric_integrity='PASS',navigation='FAIL_TIMEOUT',frozen_combined_geometric_gate='FAIL',
        shape_filter_evidence='Paired rows retained 100%, unpaired rows reduced; geometry only',reid_improvement_demonstrated=False,
        semantic_person_validated=False,association_accepted=False,persona_validated=False,training_ready=False,
        deployment='Previous baseline remains; no default ROS/Nav2/tracker changes',
        next='Use identity pairing-change case as a regression to improve causal active matching; a future tuned tracker needs a new untouched validation seed',
        holdout='HuNavSim untouched')
    (reports/'acceptance.json').write_text(json.dumps(acceptance,indent=2)+'\n')
    (reports/'review_artifacts.json').write_text(json.dumps(copied,indent=2)+'\n')
    print(json.dumps(acceptance,indent=2))
if __name__=='__main__':finalize()
