#!/usr/bin/env python3
"""Reproducibility and independent per-pair checks; no semantic acceptance implied."""
import argparse,json
from pathlib import Path
from collections import defaultdict
import numpy as np
from freeze import verify,sha
from snapshots.sensor import project,odom_at,candidates
from refinement import enrich,DormantTracker


def run(output):
    here=Path(__file__).resolve().parent;gen=here/'generated';output.mkdir(parents=True,exist_ok=False)
    first=verify(gen/'frozen01.json');extra=verify(gen/'frozen02.json')
    assert first['engine_sha256']==extra['engine_sha256'] and first['config']==extra['config']
    assert extra['extension']['parent_frozen_sha256']==sha(gen/'frozen01.json')
    for n in ['sensor.py','association.py']:assert sha(here/'snapshots'/n)==sha(here.parent/'perception_target_pilot'/n)
    summaries={};all_frames=0;all_pairs=0;replay_rows=0;counts={}
    for key,runname,evalname in [('development','development01','development_evaluation01'),('validation','validation01','validation_evaluation01'),('mesh_repeat','mesh_repeat01','mesh_repeat_evaluation01')]:
        frozen=first if key!='mesh_repeat' else extra;source=frozen['datasets'][key];norm=Path(source['normalized']);sensor=gen/runname;evaluation=gen/evalname
        assert all(sha(norm/n)==v for n,v in source['input_sha256'].items())
        assert sha(norm/'human_states.jsonl')==source['scene_sha256'] and sha(norm/'manifest.json')==source['manifest_sha256']
        result=json.loads((evaluation/'evaluation.json').read_text());summaries[key]=result['comparison'];counts[key]={}
        scene=[json.loads(l) for l in (norm/'human_states.jsonl').read_text().splitlines()];times=np.array([r['stamp_ns'] for r in scene]);ids=sorted(h['id'] for h in scene[0]['humans'])
        robot=np.array([[r['robot'][k] for k in ['x','y','yaw']] for r in scene]);robot[:,2]=np.unwrap(robot[:,2])
        humans=np.array([[[h['x'],h['y']] for h in sorted(r['humans'],key=lambda h:h['id'])] for r in scene])
        for variant in first['config']['comparison_variants']:
            frames=[json.loads(l) for l in (sensor/(variant+'.jsonl')).read_text().splitlines()];pair_count=0;grouped=defaultdict(set);distances=[]
            for row in frames:
                assert row['available_ns']>=row['stamp_ns']
                if row['odom_valid']:assert row['odom_stamp_ns']<=row['stamp_ns'] and row['odom_available_ns']<=row['stamp_ns']
                assert len({t['track_id'] for t in row['tracks']})==len(row['tracks'])
                assert all(t['semantic_class']=='unknown_geometry_candidate' for t in row['tracks'])
            for line in (evaluation/(variant+'_pairs_LABEL_ONLY.jsonl')).read_text().splitlines():
                pair=json.loads(line);assert not pair['association_accepted']
                if not pair['geometric_pair_valid']:continue
                j=ids.index(pair['gt_human_id']);t=pair['stamp_ns'];r=np.array([np.interp(t,times,robot[:,c]) for c in range(3)])
                h=np.array([np.interp(t,times,humans[:,j,c]) for c in range(2)]);dx,dy=h-r[:2]
                local=np.array([np.cos(r[2])*dx+np.sin(r[2])*dy,-np.sin(r[2])*dx+np.cos(r[2])*dy])
                track=next(a for a in frames[pair['depth_index']]['tracks'] if a['track_id']==pair['track_id'])
                distance=float(np.linalg.norm(local-track['xy_base']));assert distance<=.6 and abs(distance-pair['distance_m'])<1e-10
                assert pair['gt_human_id'] not in grouped[pair['depth_index']];grouped[pair['depth_index']].add(pair['gt_human_id']);pair_count+=1;distances.append(distance)
            expected=result['variants'][variant];assert pair_count==expected['paired_rows']
            assert abs(np.median(distances)-expected['distance_median_m'])<1e-10
            assert not expected['association_accepted'] and not expected['training_ready']
            counts[key][variant]=pair_count;all_pairs+=pair_count
            if variant=='baseline' and key=='development':
                original=[json.loads(l) for l in (here.parent/'perception_target_pilot/generated/sensor01/sensor_tracks.jsonl').read_text().splitlines()]
                assert frames==original,'Development baseline no longer reproduces the prior experiment'
            # A real prefix replay with future odometry removed at every step.
            if variant=='combined':
                obs=np.load(norm/'observations.npz',allow_pickle=False);depth=np.load(norm/'depth.npy',mmap_mode='r',allow_pickle=False);cal=json.loads((norm/'calibration.json').read_text());tr=np.array(cal['optical_to_base_link']);tracker=DormantTracker()
                for i in range(min(100,len(frames))):
                    t=obs['depth__stamp_ns'][i];valid=(obs['odom__stamp_ns']<=t)&(obs['odom__available_ns']<=t)
                    od=odom_at(t,obs['odom__stamp_ns'][valid],obs['odom__available_ns'][valid],obs['odom__values'][valid])
                    det,_=enrich(project(depth[i],cal['k'],tr)) if od else ([],[])
                    actual=tracker.update(int(t),det,od['pose']) if od else []
                    assert actual==frames[i]['tracks'];replay_rows+=1
                all_frames+=len(frames)
        assert result['comparison']['semantic_or_training_acceptance'] is False
    # Validate one ablation gate independently, retaining the failed validation.
    assert summaries['development']['geometric_improvement'] and summaries['mesh_repeat']['geometric_improvement']
    assert not summaries['validation']['geometric_improvement']
    out=dict(status='PASS',scope='Integrity/causality/comparison reproducibility, not semantic validation',source_episodes=3,distinct_depth_frames=all_frames,
        frozen_engine_unchanged_between_datasets=True,development_baseline_exactly_reproduces_previous_run=True,independent_pair_distances_checked=all_pairs,
        real_prefix_frames_replayed_without_future_odom=replay_rows,paired_rows_by_variant=counts,
        initial_proxy_validation='FAIL retained',matched_mesh_repeat='Diagnostic improvement only, prior navigation TIMEOUT retained',
        deployment_decision='Keep previous baseline; new geometry/tracker is experiment only',training_ready=False)
    inventories={}
    for root in [gen/'development01',gen/'validation01',gen/'mesh_repeat01',gen/'development_evaluation01',gen/'validation_evaluation01',gen/'mesh_repeat_evaluation01']:
        inventories[root.name]={p.name:dict(bytes=p.stat().st_size,sha256=sha(p)) for p in root.iterdir() if p.is_file()}
    (output/'inventories.json').write_text(json.dumps(inventories,indent=2)+'\n');(output/'audit.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.output.resolve())
