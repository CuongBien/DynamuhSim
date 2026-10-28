#!/usr/bin/env python3
"""Independent geometry, retained proposals, source snapshots and causal-prefix audit."""
import argparse,json
from pathlib import Path
from collections import defaultdict
import numpy as np
from freeze import verify,sha
from guard_tracker import IdentityGuard
from refinement import enrich
from snapshots.sensor import project,odom_at
HERE=Path(__file__).resolve().parent


def rows(p):return [json.loads(l) for l in p.read_text().splitlines()]
def audit(output):
    output.mkdir(parents=True,exist_ok=False);fixed=verify(HERE/'generated/final_with_seed44.json');results={};checked=0;prefix=0;unknown_total=0
    for key,source in fixed['datasets'].items():
        norm=Path(source['normalized']);tag='44' if key=='seed44' else '05';sensor=HERE/'generated'/(key+'_'+tag);evaluation=HERE/'generated'/(key+'_eval'+tag)
        assert all(sha(norm/n)==h for n,h in source['input_sha256'].items()) and sha(norm/'human_states.jsonl')==source['scene_sha256']
        expected_frozen=HERE/'generated'/('final_with_seed44.json' if key=='seed44' else 'candidate05.json')
        assert json.loads((sensor/'replay.json').read_text())['frozen_sha256']==sha(expected_frozen)
        result=json.loads((evaluation/'evaluation.json').read_text());assert result['frozen_sha256']==sha(expected_frozen);gate=json.loads((evaluation/'identity_gate.json').read_text());obs=np.load(norm/'observations.npz',allow_pickle=False)
        scene=rows(norm/'human_states.jsonl');times=np.array([r['stamp_ns'] for r in scene]);ids=sorted(h['id'] for h in scene[0]['humans']);robot=np.array([[r['robot'][k] for k in ['x','y','yaw']] for r in scene]);robot[:,2]=np.unwrap(robot[:,2]);humans=np.array([[[h['x'],h['y']] for h in sorted(r['humans'],key=lambda h:h['id'])] for r in scene])
        frames_by_name={name:rows(sensor/(name+'.jsonl')) for name in fixed['config']['comparison_variants']}
        original_paths={'mesh42':HERE.parent/'depth_track_refinement/generated/development01/combined.jsonl','repeat42':HERE.parent/'depth_track_refinement/generated/mesh_repeat01/combined.jsonl','regression43':HERE.parent/'depth_seed43_validation/generated/sensor01/combined.jsonl'}
        if key in original_paths:assert frames_by_name['parent']==rows(original_paths[key]),'Parent comparison changed'
        for p,n in zip(frames_by_name['parent'],frames_by_name['combined']):
            assert len(p['tracks'])==len(n['tracks'])
            for j,(a,c) in enumerate(zip(p['tracks'],n['tracks'])):
                assert a['xy_base']==c['xy_base'] and c['proposal_index']==j
                if c['track_id']<0:
                    assert not c['confirmed'] and not c['identity_observed'] and not c['velocity_valid'] and c['association_ambiguous'];unknown_total+=1
        for variant,frames in frames_by_name.items():
            assert len(frames)==len(obs['depth__stamp_ns']);confirmed=0;grouped=defaultdict(set);per_human=defaultdict(set);distances=[]
            for i,row in enumerate(frames):
                assert row['stamp_ns']==obs['depth__stamp_ns'][i] and row['available_ns']>=row['stamp_ns']
                if row['odom_valid']:assert row['odom_stamp_ns']<=row['stamp_ns'] and row['odom_available_ns']<=row['stamp_ns']
                positive=[t['track_id'] for t in row['tracks'] if t['track_id']>=0];assert len(positive)==len(set(positive))
                confirmed+=sum(t['confirmed'] for t in row['tracks']);assert all(t['semantic_class']=='unknown_geometry_candidate' for t in row['tracks'])
            for pair in rows(evaluation/(variant+'_pairs_LABEL_ONLY.jsonl')):
                assert not pair['association_accepted']
                if not pair['geometric_pair_valid']:continue
                i=pair['depth_index'];t=pair['stamp_ns'];j=ids.index(pair['gt_human_id']);r=np.array([np.interp(t,times,robot[:,c]) for c in range(3)]);h=np.array([np.interp(t,times,humans[:,j,c]) for c in range(2)]);dx,dy=h-r[:2];local=np.array([np.cos(r[2])*dx+np.sin(r[2])*dy,-np.sin(r[2])*dx+np.cos(r[2])*dy])
                track=next(x for x in frames[i]['tracks'] if x['track_id']==pair['track_id']);error=float(np.linalg.norm(local-track['xy_base']));assert error<=.6 and abs(error-pair['distance_m'])<1e-10
                assert pair['gt_human_id'] not in grouped[i];grouped[i].add(pair['gt_human_id']);per_human[pair['gt_human_id']].add(pair['track_id']);distances.append(error)
            expected=result['variants'][variant];assert confirmed==expected['confirmed_candidate_rows'] and len(distances)==expected['paired_rows']
            assert sum(max(0,len(per_human[j])-1) for j in ids)==expected['fragmentation_proxy'];checked+=len(distances)
        # Prefix includes the seed43 counterexample; odometry after each query is excluded.
        cal=json.loads((norm/'calibration.json').read_text());tr=np.array(cal['optical_to_base_link']);depth=np.load(norm/'depth.npy',mmap_mode='r');tracker=IdentityGuard();n=min(200,len(depth))
        for i in range(n):
            t=obs['depth__stamp_ns'][i];valid=(obs['odom__stamp_ns']<=t)&(obs['odom__available_ns']<=t)
            od=odom_at(t,obs['odom__stamp_ns'][valid],obs['odom__available_ns'][valid],obs['odom__values'][valid]);det,_=enrich(project(depth[i],cal['k'],tr)) if od else ([],[]);tracks=tracker.update(int(t),det,od['pose']) if od else [];assert tracks==frames_by_name['combined'][i]['tracks']
        prefix+=n;old=result['variants']['parent'];new=result['variants']['combined'];spec=fixed['config']['new_identity_gate']
        independent=bool(new['paired_rows']>=spec['min_paired_retention_vs_parent']*old['paired_rows'] and new['paired_identity_changes_within_500ms']==0 and new['fragmentation_proxy']<=old['fragmentation_proxy'] and new['dormant_reacquisitions_wrong_known_identity']==0)
        assert independent==gate['identity_guard_geometric_gate'];results[key]=dict(role=source['role'],identity_gate=independent,legacy_full_gate=gate['legacy_full_gate'],paired_retention=new['paired_rows']/old['paired_rows'],fragmentation=new['fragmentation_proxy'],pairing_changes=new['paired_identity_changes_within_500ms'])
    # Every archived candidate's inference files retain the SHA in its contemporaneous manifest.
    archived=0
    for d in sorted((HERE/'reports/candidates').iterdir()):
        frozen=json.loads((d/'frozen.json').read_text())
        for n,h in frozen['engine_sha256'].items():assert sha(d/'engine'/n)==h;archived+=1
    raw=json.loads((HERE/'reports/seed44/raw_audit.json').read_text());root=HERE/'generated/runs/identity_seed44_01';inventory=json.loads((HERE/'reports/seed44/closed_raw_inventory.json').read_text());assert all(sha(root/'raw_bag'/n)==v['sha256'] for n,v in inventory.items())
    report=dict(status='PASS',scope='Integrity and geometric association validation only; not semantic person/visibility or train acceptance',datasets=results,independent_pair_distances_checked=checked,causal_prefix_frames_replayed=prefix,
        archived_engine_files_sha_verified=archived,all_sensor_proposals_retained=True,ambiguous_proposals_withheld_from_identity=unknown_total,parent_streams_exactly_reproduced_on_regressions=True,
        new_seed_raw_depth_bitwise_equal=raw['depth_frames_bitwise_equal'],frozen_before_seed44=True,algorithm_changed_after_holdout=False,training_ready=False,association_accepted=False)
    (output/'audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();audit(a.output.resolve())
