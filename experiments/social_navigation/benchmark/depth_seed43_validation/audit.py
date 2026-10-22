#!/usr/bin/env python3
"""Independent per-pair, causal prefix, raw/source and frozen-gate verification."""
import argparse,json,sys
from collections import defaultdict
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent
sys.path.insert(0,str(BENCH/'depth_track_refinement'))
from freeze import verify,sha
from refinement import enrich,DormantTracker
from snapshots.sensor import project,odom_at


def audit(output):
    output.mkdir(parents=True,exist_ok=False)
    root=HERE/'generated/runs/depth_seed43_01';norm=root/'normalized';sensor=HERE/'generated/sensor01';labels=HERE/'generated/evaluation01'
    fixed=verify(HERE/'generated/frozen_seed43.json');source=fixed['datasets']['seed43']
    capture=json.loads((HERE/'generated/drivers/depth_seed43_01/manifest.json').read_text())
    assert capture['inference_engine_sha256']==fixed['engine_sha256'] and capture['inference_config']==fixed['config']
    assert all(sha(norm/n)==v for n,v in source['input_sha256'].items()) and sha(norm/'human_states.jsonl')==source['scene_sha256']
    inventory=json.loads((HERE/'reports/capture/closed_raw_inventory.json').read_text())
    assert all(sha(root/'raw_bag'/n)==v['sha256'] for n,v in inventory.items())
    raw=json.loads((HERE/'reports/raw_audit.json').read_text());obs=np.load(norm/'observations.npz',allow_pickle=False);samples=np.load(norm/'samples.npz',allow_pickle=False);y=np.load(norm/'labels.npz',allow_pickle=False)
    assert raw['depth_frames_bitwise_equal']==len(obs['depth__stamp_ns']) and raw['status']=='PASS'
    assert not y['persona_valid'].any() and (y['persona_index']==-1).all()
    assert not any(word in key for key in samples.files for word in ['human','persona','world','future','backend'])
    result=json.loads((labels/'evaluation.json').read_text());assert result['frozen_sha256']==sha(HERE/'generated/frozen_seed43.json')
    scene=[json.loads(l) for l in (norm/'human_states.jsonl').read_text().splitlines()];times=np.array([r['stamp_ns'] for r in scene]);ids=sorted(h['id'] for h in scene[0]['humans'])
    robot=np.array([[r['robot'][k] for k in ['x','y','yaw']] for r in scene]);robot[:,2]=np.unwrap(robot[:,2]);humans=np.array([[[h['x'],h['y']] for h in sorted(r['humans'],key=lambda h:h['id'])] for r in scene])
    checked=0;reacquisitions=0;by_variant={};combined=None
    for variant in fixed['config']['comparison_variants']:
        frames=[json.loads(l) for l in (sensor/(variant+'.jsonl')).read_text().splitlines()];assert len(frames)==len(obs['depth__stamp_ns'])
        confirmed=0
        for i,row in enumerate(frames):
            assert row['stamp_ns']==int(obs['depth__stamp_ns'][i]) and row['available_ns']==int(obs['depth__available_ns'][i])
            if row['odom_valid']:assert row['odom_stamp_ns']<=row['stamp_ns'] and row['odom_available_ns']<=row['stamp_ns']
            assert len({t['track_id'] for t in row['tracks']})==len(row['tracks'])
            for t in row['tracks']:
                assert t['semantic_class']=='unknown_geometry_candidate';confirmed+=int(t['confirmed'])
                if t.get('reacquired_after_gap'):
                    assert 500_000_000<t['gap_since_last_measurement_ns']<=1_500_000_000;reacquisitions+=1
        grouped=defaultdict(set);per_human=defaultdict(set);distances=[]
        for line in (labels/(variant+'_pairs_LABEL_ONLY.jsonl')).read_text().splitlines():
            pair=json.loads(line);assert not pair['association_accepted']
            if not pair['geometric_pair_valid']:continue
            i=pair['depth_index'];stamp=pair['stamp_ns'];j=ids.index(pair['gt_human_id']);r=np.array([np.interp(stamp,times,robot[:,c]) for c in range(3)]);h=np.array([np.interp(stamp,times,humans[:,j,c]) for c in range(2)]);dx,dy=h-r[:2]
            xy=np.array([np.cos(r[2])*dx+np.sin(r[2])*dy,-np.sin(r[2])*dx+np.cos(r[2])*dy]);track=next(t for t in frames[i]['tracks'] if t['track_id']==pair['track_id'])
            distance=float(np.linalg.norm(xy-track['xy_base']));assert distance<=.60 and abs(distance-pair['distance_m'])<1e-10
            assert pair['gt_human_id'] not in grouped[i];grouped[i].add(pair['gt_human_id']);per_human[pair['gt_human_id']].add(pair['track_id']);distances.append(distance)
        expected=result['variants'][variant];fragmentation=sum(max(0,len(per_human[j])-1) for j in ids)
        assert expected['paired_rows']==len(distances) and expected['confirmed_candidate_rows']==confirmed and expected['fragmentation_proxy']==fragmentation
        assert abs(np.median(distances)-expected['distance_median_m'])<1e-10
        assert not expected['association_accepted'] and not expected['training_ready'];checked+=len(distances);by_variant[variant]=len(distances)
        if variant=='combined':combined=frames
    depth=np.load(norm/'depth.npy',mmap_mode='r');cal=json.loads((norm/'calibration.json').read_text());tr=np.array(cal['optical_to_base_link']);tracker=DormantTracker()
    for i in range(min(100,len(combined))):
        stamp=obs['depth__stamp_ns'][i];valid=(obs['odom__stamp_ns']<=stamp)&(obs['odom__available_ns']<=stamp)
        od=odom_at(stamp,obs['odom__stamp_ns'][valid],obs['odom__available_ns'][valid],obs['odom__values'][valid]);det,_=enrich(project(depth[i],cal['k'],tr)) if od else ([],[])
        tracks=tracker.update(int(stamp),det,od['pose']) if od else [];assert tracks==combined[i]['tracks']
    base=result['variants']['baseline'];new=result['variants']['combined'];gate=fixed['config']['geometric_comparison_gate']
    decision=bool(new['paired_rows']>=gate['min_paired_row_retention']*base['paired_rows'] and new['unpaired_rows']<base['unpaired_rows'] and new['fragmentation_proxy']<base['fragmentation_proxy'] and new['paired_identity_changes_within_500ms']<=base['paired_identity_changes_within_500ms'] and new['dormant_reacquisitions_wrong_known_identity']==0)
    assert decision==result['comparison']['geometric_improvement']
    report=dict(status='PASS',seed=43,source_raw_unchanged=True,inference_frozen_before_capture=True,
        depth_frames_bitwise_equal=raw['depth_frames_bitwise_equal'],windows=len(samples['anchor_ns']),independent_pair_distances_checked=checked,paired_rows_by_variant=by_variant,
        causal_prefix_frames_replayed=min(100,len(combined)),dormant_reacquisition_gaps_checked=reacquisitions,independent_geometric_gate=decision,
        training_ready=False,semantic_person_validated=False,association_accepted=False,scope='Data/timing/metric integrity only; geometric gate result reported separately from semantic readiness')
    files={str(p.relative_to(HERE/'generated')):dict(bytes=p.stat().st_size,sha256=sha(p)) for folder in [sensor,labels] for p in folder.iterdir() if p.is_file()}
    (output/'derived_inventory.json').write_text(json.dumps(files,indent=2)+'\n');(output/'audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();audit(a.output.resolve())
