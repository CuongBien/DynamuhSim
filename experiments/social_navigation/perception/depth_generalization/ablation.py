#!/usr/bin/env python3
"""Fixed prior ablation; reuses frozen inference/evaluation without editing either."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import time
import numpy as np

HERE=Path(__file__).resolve().parent
GUARD=HERE.parent/'depth_identity_guard'
CLASSIFIER=HERE.parent/'depth_person_classifier'
sys.path.insert(0,str(GUARD))
sys.path.insert(0,str(CLASSIFIER))
from compare_recordings import sha, dump, old_proposals, evaluate_episode
from guard_tracker import IdentityGuard
from snapshots.sensor import project, odom_at
from snapshots.association import associate
from generalized_detector import GeneralizedPersonDetector, adaptive_3d_clustering
from frozen_classifier import Classifier


def choose_without_prior(detections, threshold):
    return [dict(d,score=d['base_score'],is_person=True) for d in detections
            if d['inference_valid'] and np.isfinite(d['base_score']) and threshold<=d['base_score']<=1.0]


def check_reference(actual, expected):
    if actual['stamp_ns']!=expected['stamp_ns'] or actual['odom_valid']!=expected['odom_valid']:
        raise ValueError('Reference timestamp/validity mismatch')
    if len(actual['tracks'])!=len(expected['tracks']):raise ValueError('Reference track count mismatch')
    for a,b in zip(actual['tracks'],expected['tracks']):
        for key in ['track_id','proposal_index','point_count','confirmed','hits','association_ambiguous','reacquired_after_gap']:
            if a[key]!=b[key]:raise ValueError('Reference track field mismatch: '+key)
        for key in ['xy_base','xy_odom','velocity_odom','extent_m']:
            if not np.allclose(a[key],b[key],rtol=0,atol=1e-10):raise ValueError('Reference geometry mismatch: '+key)
        if 'person_score' in b and not np.isclose(a['person_score'],b['person_score'],rtol=0,atol=1e-12):
            raise ValueError('Reference score mismatch')


def infer(norm,reference,output,protocol):
    with np.load(norm/'observations.npz',allow_pickle=False) as data:obs=dict(data)
    depth=np.load(norm/'depth.npy',mmap_mode='r',allow_pickle=False)
    calibration=json.loads((norm/'calibration.json').read_text())
    transform=np.array(calibration['optical_to_base_link'])
    detector=GeneralizedPersonDetector(min_person_score=.45)
    classifier=Classifier(CLASSIFIER/'artifacts')
    trackers={name:IdentityGuard() for name in protocol['variants']}
    expected={name:[json.loads(line) for line in (reference/(name+'.jsonl')).read_text().splitlines()]
              for name in ['legacy_2d_classifier','generalized_3d_classifier']}
    handles={name:(output/(name+'.jsonl')).open('x') for name in protocol['variants']}
    stages=(output/'proposal_stages_SENSOR_ONLY.jsonl').open('x')
    try:
        for index,stamp in enumerate(obs['depth__stamp_ns']):
            stamp=int(stamp);start=time.perf_counter_ns()
            od=odom_at(stamp,obs['odom__stamp_ns'],obs['odom__available_ns'],obs['odom__values'],protocol['odom_max_age_ns'])
            before=[];after=[];choices={name:[] for name in trackers}
            if od:
                points=project(depth[index],calibration['k'],transform,protocol['stride'])
                before=[np.median(q[:,:2],axis=0).tolist() for q in adaptive_3d_clustering(points)]
                candidates=detector.detect_candidates(points)
                after=[c['xy'].tolist() for c in candidates]
                decisions=detector.detect(points)
                if len(candidates)!=len(decisions):raise ValueError('Candidate order mismatch')
                old=[]
                for candidate in old_proposals(points):
                    score=classifier.predict(dict(points_xyz=candidate['raw_points'],scan_valid=False,scan_ranges=[]))['score']
                    if score>=.5:old.append(dict(candidate,score=score,inference_valid=True))
                choices=dict(legacy_2d_classifier=old,
                             generalized_3d_classifier=[d for d in decisions if d['is_person']],
                             three_d_no_prior_045=choose_without_prior(decisions,.45),
                             three_d_no_prior_050=choose_without_prior(decisions,.5))
            elapsed=(time.perf_counter_ns()-start)/1e6
            for name,stream in handles.items():
                tracks=trackers[name].update(stamp,choices[name],od['pose']) if od else []
                for t,d in zip(tracks,choices[name]):
                    t.update(person_score=float(d['score']),inference_valid=bool(d['inference_valid']),training_authorized=False,semantic_validated=False)
                row=dict(depth_index=index,stamp_ns=stamp,available_ns=int(obs['depth__available_ns'][index]),
                         odom_valid=od is not None,odom_stamp_ns=od['stamp_ns'] if od else -1,
                         odom_available_ns=od['available_ns'] if od else -1,tracks=tracks,
                         latency_ms=elapsed,latency_scope='Shared instrumented frame, not per-variant timing')
                if name in expected:check_reference(row,expected[name][index])
                stream.write(json.dumps(row,allow_nan=False)+'\n')
            stages.write(json.dumps(dict(depth_index=index,stamp_ns=stamp,odom_valid=od is not None,
                                          pre_planar_3d_xy=before,post_planar_3d_xy=after),allow_nan=False)+'\n')
            if index%150==0:print(norm.name,index,'/',len(depth),flush=True)
    finally:
        for stream in handles.values():stream.close()
        stages.close()
    return dict(frames=len(depth),reference_rows_checked=2*len(depth),sha256={p.name:sha(p) for p in output.glob('*.jsonl')})


def matched_supported(xy,local,support,protocol):
    pair,_=associate(xy,local,protocol['pair_gate_m'],protocol['pair_ambiguity_margin_m'])
    return {int(i) for i in pair if i>=0 and support[i]}


def stage_counts(output,protocol):
    proposals={r['depth_index']:r for r in map(json.loads,(output/'proposal_stages_SENSOR_ONLY.jsonl').read_text().splitlines())}
    labels=[json.loads(l) for l in (output/'evaluation_LABEL_ONLY.jsonl').read_text().splitlines()]
    variants={name:[json.loads(l) for l in (output/(name+'.jsonl')).read_text().splitlines()] for name in protocol['variants']}
    totals=Counter();per_frame=[]
    for row in labels:
        index=row['depth_index'];local=row['gt_xy_base_LABEL_ONLY'];support=row['depth_supported_proxy'];source=proposals[index]
        universe={i for i,yes in enumerate(support) if yes}
        sets=dict(supported=universe,
                  proposal_3d=matched_supported(source['pre_planar_3d_xy'],local,support,protocol),
                  planar_kept=matched_supported(source['post_planar_3d_xy'],local,support,protocol))
        for name in protocol['variants']:
            sets[name]=matched_supported([t['xy_base'] for t in variants[name][index]['tracks']],local,support,protocol)
        totals.update({k:len(v) for k,v in sets.items()})
        transitions=[('proposal_geometry','supported','proposal_3d'),('planar_filter','proposal_3d','planar_kept'),
                     ('classifier_045','planar_kept','three_d_no_prior_045'),
                     ('multiplicative_prior','three_d_no_prior_045','generalized_3d_classifier')]
        for tag,a,b in transitions:
            totals[tag+'_lost']+=len(sets[a]-sets[b]);totals[tag+'_gained']+=len(sets[b]-sets[a])
        per_frame.append(dict(depth_index=index,stamp_ns=row['stamp_ns'],gt_ids=row['gt_ids'],
                              matched_supported_sets_LABEL_ONLY={name:sorted(v) for name,v in sets.items()}))
    with (output/'stage_losses_LABEL_ONLY.jsonl').open('x') as stream:
        for row in per_frame:stream.write(json.dumps(row)+'\n')
    return dict(totals,scope='One-to-one geometric association with cylinder-support visibility proxy. Stage matching may lose AND gain pairs; not semantic error attribution.')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--normalized-root',type=Path,required=True)
    parser.add_argument('--reference',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    protocol=json.loads((HERE/'protocol.json').read_text())
    normalized=args.normalized_root.resolve();reference=args.reference.resolve()
    reference_plan=json.loads((reference/'plan.json').read_text())
    ref_seal=json.loads((reference/'sensor_seal.json').read_text())
    for ep in protocol['episodes']:
        for name,h in reference_plan['input_sha256'][ep].items():
            if sha(normalized/ep/name)!=h:raise ValueError('Normalized input differs from reference')
        for name,h in ref_seal[ep]['sensor_sha256'].items():
            if sha(reference/ep/(name+'.jsonl'))!=h:raise ValueError('Reference sensor seal changed')
    sources=[HERE/'ablation.py',HERE/'protocol.json']
    for path,h in reference_plan['source_sha256'].items():
        relative=path.split('/DynamuhSim/',1)[1]
        current=HERE.parents[3]/relative
        if sha(current)!=h:raise ValueError('Inherited frozen source changed: '+relative)
        sources.append(current)
    plan=dict(schema='depth_ablation_freeze/1',protocol=protocol,normalized_root=str(normalized),
              input_sha256=reference_plan['input_sha256'],source_sha256={str(p):sha(p) for p in sources},
              reference_comparison_sha256=sha(reference/'comparison.json'),reference_sensor_seal_sha256=sha(reference/'sensor_seal.json'))
    dump(output/'plan.json',plan)
    seals={}
    for ep in protocol['episodes']:
        dest=output/ep;dest.mkdir();seals[ep]=infer(normalized/ep,reference/ep,dest,protocol)
    dump(output/'sensor_seal.json',seals)
    results={}
    for ep in protocol['episodes']:
        results[ep]=evaluate_episode(normalized/ep,output/ep,protocol)
        for stats in results[ep]['variants'].values():
            stats.pop('latency_p50_ms',None);stats.pop('latency_p95_ms',None)
        results[ep]['timing_policy']='Instrumented shared preprocessing, no comparative timing claim'
        results[ep]['stage_attribution']=stage_counts(output/ep,protocol)
        baseline=results[ep]['variants']['legacy_2d_classifier'];gates={}
        for name in ['three_d_no_prior_045','three_d_no_prior_050']:
            v=results[ep]['variants'][name]
            gates[name]=dict(paired_retention=v['paired']>=.95*baseline['paired'],
                             unpaired_not_increased=v['unpaired']<=baseline['unpaired'],
                             id_switches_not_increased=v['id_switches_500ms']<=baseline['id_switches_500ms'],
                             fragmentation_not_increased=v['fragmentation_proxy']<=baseline['fragmentation_proxy'])
            gates[name]['pass']=all(gates[name].values())
        results[ep]['local_gates']=gates
        results[ep].pop('promotion_conditions',None)
        results[ep].pop('geometric_nonregression_pass',None)
    keep_3d=all(r['local_gates']['three_d_no_prior_050']['pass'] for r in results.values())
    summary=dict(schema='depth_ablation_result/1',episodes=results,
                 reference_reproduced_rows=sum(s['reference_rows_checked'] for s in seals.values()),
                 candidate_proposal_provider='3D_no_prior_050' if keep_3d else 'legacy_2D_high_recall_geometry',
                 classifier_candidate='small_pointnet_style_head',
                 generalization_test_status='NOT_RUN_REQUIRES_FRESH_DOMAINS',training_authorized=False,
                 sensor_sealed_before_gt=True,promotion_to_default=False,training_performed=False)
    if any(sha(p)!=h for p,h in plan['source_sha256'].items()):raise ValueError('Source changed during run')
    for ep,files in plan['input_sha256'].items():
        if any(sha(normalized/ep/p)!=h for p,h in files.items()):raise ValueError('Input changed during run')
    dump(output/'result.json',summary)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
