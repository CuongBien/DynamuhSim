#!/usr/bin/env python3
"""Bounded original-vs-incoming weight replay on OPENED pilot data; no fit or new test access."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np

HERE = Path(__file__).resolve().parent
CLASSIFIER = HERE.parent / 'depth_person_classifier'
GUARD = HERE.parent / 'depth_identity_guard'
sys.path[:0] = [str(HERE), str(GUARD), str(CLASSIFIER)]
from compare_recordings import sha, dump, old_proposals, evaluate_episode, project, odom_at
from ablation import check_reference
from guard_tracker import IdentityGuard
from generalized_detector import GeneralizedPersonDetector
from frozen_classifier import Classifier

VARIANTS = ['legacy_2d_classifier', 'incoming_2d_050', 'incoming_2d_085', 'generalized_3d_classifier']

def geometry_fingerprint(path):
    arrays = [node.text.split() for node in ET.parse(path).getroot().iter() if node.tag.endswith('float_array')]
    return hashlib.sha256(json.dumps(arrays, separators=(',', ':')).encode()).hexdigest()

def morphology_leaks(rows):
    seen = {}; leaks = []
    for row in rows:
        fp = row['geometry_arrays_sha256']
        previous = seen.setdefault(fp, set())
        if previous and row['split'] not in previous:
            leaks.append(row['case_id'])
        previous.add(row['split'])
    return leaks

def audit(root):
    tree = ast.parse((HERE / 'build_mini_benchmark.py').read_text())
    specs = next(ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign) and
                 any(isinstance(t, ast.Name) and t.id == 'CASES_SPEC' for t in n.targets))
    rows = []
    for c in specs:
        if not c['has_people']: continue
        path = root / c['mesh_file']
        rows.append(dict(case_id=c['case_id'], split=c['split'], declared_family=c['avatar_families'],
                         mesh=c['mesh_file'], geometry_arrays_sha256=geometry_fingerprint(path),
                         scene_hash_length=len(c['scene_geometry_sha256']), height=c['people'][0]['height']))
    missing = [name for name in ['frozen_candidate_weights.pt', 'model_freeze.json',
               'training_curation_receipt.json', 'test_evaluation_receipt.json'] if not (HERE/'generated'/name).exists()]
    return dict(scope='Source + metadata inspection; no new held-out predictions generated', human_sources=rows,
                cross_split_morphology_leaks=morphology_leaks(rows),
                missing_local_candidate_artifacts=missing,
                benchmark_backend='procedural point-splat depth renderer, not a Gazebo capture',
                generalization_accepted=False, default_promotion=False)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--normalized-root', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = HERE.parents[3]
    output = args.output.resolve(); output.mkdir(parents=True, exist_ok=False)
    protocol = json.loads((HERE/'protocol.json').read_text()); protocol['variants'] = VARIANTS
    protocol['variant_policy'] = dict(legacy_2d_classifier='original weights; 2D .50',
        incoming_2d_050='incoming weights; same 2D proposals .50 control',
        incoming_2d_085='incoming weights; same 2D proposals .85 author threshold',
        generalized_3d_classifier='incoming weights; 3D + prior .45 as incoming default code')
    protocol['timing_policy'] = 'Shared instrumentation; no comparative latency claim'
    folders = dict(baseline=CLASSIFIER/'artifacts', incoming=CLASSIFIER/'artifacts_experimental_2ffd7c75')
    source_paths = [Path(__file__), HERE/'protocol.json', GUARD/'compare_recordings.py', GUARD/'guard_tracker.py',
                    CLASSIFIER/'generalized_detector.py', CLASSIFIER/'frozen_classifier.py']
    for folder in folders.values(): source_paths += [folder/n for n in ['features.py','inference.py','selected_model.npz','selected_frozen.json']]
    inputs = {ep:{n:sha(args.normalized_root/ep/n) for n in ['depth.npy','observations.npz','calibration.json','human_states.jsonl']} for ep in protocol['episodes']}
    reference_plan = json.loads((args.reference/'plan.json').read_text())
    reference_seals = json.loads((args.reference/'sensor_seal.json').read_text())
    for ep in protocol['episodes']:
        assert inputs[ep] == reference_plan['input_sha256'][ep], 'Pilot changed'
        expected_path = args.reference/ep/'legacy_2d_classifier.jsonl'
        assert sha(expected_path) == reference_seals[ep]['sensor_sha256']['legacy_2d_classifier']
    plan = dict(protocol=protocol, normalized_root=str(args.normalized_root.resolve()),
                reference=str(args.reference.resolve()), source_sha256={str(p):sha(p) for p in source_paths}, input_sha256=inputs,
                source_audit=audit(root), model_dirs={k:str(v) for k,v in folders.items()})
    dump(output/'plan.json',plan)
    legacy = Classifier(folders['baseline']); incoming = Classifier(folders['incoming'])
    detector = GeneralizedPersonDetector(artifacts_dir=folders['incoming'], min_person_score=.45)
    seals = {}; checked = 0
    for ep in protocol['episodes']:
        norm=args.normalized_root/ep; dest=output/ep;dest.mkdir()
        with np.load(norm/'observations.npz',allow_pickle=False) as data: obs=dict(data)
        depth=np.load(norm/'depth.npy',mmap_mode='r',allow_pickle=False)
        cal=json.loads((norm/'calibration.json').read_text()); transform=np.array(cal['optical_to_base_link'])
        reference=[json.loads(l) for l in (args.reference/ep/'legacy_2d_classifier.jsonl').read_text().splitlines()]
        trackers={name:IdentityGuard() for name in VARIANTS}
        handles={name:(dest/(name+'.jsonl')).open('x') for name in VARIANTS}
        try:
            for index,stamp in enumerate(obs['depth__stamp_ns']):
                stamp=int(stamp);od=odom_at(stamp,obs['odom__stamp_ns'],obs['odom__available_ns'],obs['odom__values'],protocol['odom_max_age_ns'])
                choices={name:[] for name in VARIANTS}
                if od:
                    points=project(depth[index],cal['k'],transform,protocol['stride'])
                    for candidate in old_proposals(points):
                        sample=dict(points_xyz=candidate['raw_points'],scan_valid=False,scan_ranges=[])
                        oldscore=legacy.predict(sample)['score'];newscore=incoming.predict(sample)['score']
                        for name,score,tau in [('legacy_2d_classifier',oldscore,.5),('incoming_2d_050',newscore,.5),('incoming_2d_085',newscore,.85)]:
                            if score>=tau:choices[name].append(dict(candidate,score=score,inference_valid=True))
                    choices['generalized_3d_classifier']=[d for d in detector.detect(points) if d['is_person']]
                for name in VARIANTS:
                    tracks=trackers[name].update(stamp,choices[name],od['pose']) if od else []
                    for t,d in zip(tracks,choices[name]):t.update(person_score=float(d['score']),inference_valid=True,semantic_validated=False,training_authorized=False)
                    row=dict(depth_index=index,stamp_ns=stamp,available_ns=int(obs['depth__available_ns'][index]),odom_valid=od is not None,
                             odom_stamp_ns=od['stamp_ns'] if od else -1,odom_available_ns=od['available_ns'] if od else -1,tracks=tracks,
                             latency_ms=0.,latency_scope='not measured per variant')
                    if name=='legacy_2d_classifier':check_reference(row,reference[index]);checked+=1
                    handles[name].write(json.dumps(row,allow_nan=False)+'\n')
                if index%150==0:print(ep,index,'/',len(depth),flush=True)
        finally:
            for handle in handles.values():handle.close()
        seals[ep]={name:sha(dest/(name+'.jsonl')) for name in VARIANTS}
    # Seal every sensor-only output before loading GT for either episode.
    dump(output/'sensor_seal.json',seals)
    results={}
    for ep in protocol['episodes']:
        q=evaluate_episode(args.normalized_root/ep,output/ep,protocol)
        q.pop('promotion_conditions',None);q.pop('geometric_nonregression_pass',None)
        baseline=q['variants']['legacy_2d_classifier']
        q['local_nonregression']={}
        for name,stat in q['variants'].items():
            stat.pop('latency_p50_ms',None);stat.pop('latency_p95_ms',None)
            if name=='legacy_2d_classifier':continue
            checks=dict(paired_retention=stat['paired']>=.95*baseline['paired'],unpaired_not_increased=stat['unpaired']<=baseline['unpaired'],
                        id_switches_not_increased=stat['id_switches_500ms']<=baseline['id_switches_500ms'],fragmentation_not_increased=stat['fragmentation_proxy']<=baseline['fragmentation_proxy'])
            q['local_nonregression'][name]=dict(checks,passed=all(checks.values()))
        results[ep]=q
    for ep,inputs_ep in inputs.items():
        for name,h in inputs_ep.items():assert sha(args.normalized_root/ep/name)==h
    for name,h in plan['source_sha256'].items():assert sha(Path(name))==h
    for ep,files in seals.items():
        for name,h in files.items():assert sha(output/ep/(name+'.jsonl'))==h
    summary=dict(incoming_commit='2ffd7c75218778f328d692724f382f623aab7139',baseline_commit='879aaf41229250de8eb4288d3105f12322f15dda',
                 episodes=results,baseline_reference_rows_checked=checked,sensor_sealed_before_gt=True,input_source_seal_integrity=True,
                 semantic_quality_accepted=False,generalization_accepted=False,promotion_to_default=False,new_training_performed=False)
    dump(output/'result.json',summary)
    print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
