#!/usr/bin/env python3
"""Bounded old/new replay. Sensor outputs are sealed before any GT is parsed."""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from scipy.ndimage import binary_dilation, label
from integrated_pipeline import EndToEndPerceptionPipeline
from guard_tracker import IdentityGuard
from refinement import enrich
from snapshots.sensor import project, odom_at, to_local
from snapshots.association import associate
from frozen_classifier import Classifier

HERE = Path(__file__).resolve().parent
CLASSIFIER = HERE.parent / 'depth_person_classifier'


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''): h.update(block)
    return h.hexdigest()


def dump(path, obj):
    with Path(path).open('x') as stream:
        json.dump(obj, stream, indent=2, allow_nan=False)
        stream.write('\n')


def old_proposals(points):
    kept, _ = enrich(points)
    p = points[(points[:, 0]>=0)&(points[:, 0]<5.3)&(abs(points[:, 1])<3.2)&(points[:, 2]>=.04)&(points[:, 2]<=1.8)]
    if not kept: return []
    cells = np.floor((p[:, :2]+[0, 3.2])/.08).astype(int)
    grid = np.zeros((68, 82), bool)
    grid[cells[:, 0], cells[:, 1]] = True
    components, _ = label(binary_dilation(grid, iterations=1), structure=np.ones((3, 3)))
    memberships = components[cells[:, 0], cells[:, 1]]
    clusters = [p[memberships==i] for i in np.unique(memberships)]
    centers = np.array([np.median(q[:, :2], axis=0) for q in clusters])
    return [dict(d, raw_points=clusters[int(np.argmin(np.linalg.norm(centers-d['xy'], axis=1)))]) for d in kept]


def infer_episode(norm, output, protocol):
    """Inputs here are SENSOR_ONLY: do not read scenes, labels or GT."""
    with np.load(norm/'observations.npz', allow_pickle=False) as arrays: obs = dict(arrays)
    depth = np.load(norm/'depth.npy', mmap_mode='r', allow_pickle=False)
    calibration = json.loads((norm/'calibration.json').read_text())
    transform = np.array(calibration['optical_to_base_link'])
    pipeline = EndToEndPerceptionPipeline(min_person_score=protocol['generalized_3d_classifier_threshold'])
    old = {name: IdentityGuard() for name in protocol['variants'] if name!='generalized_3d_classifier'}
    classifier = Classifier(CLASSIFIER/'artifacts')
    handles = {name: (output/(name+'.jsonl')).open('x') for name in protocol['variants']}
    try:
        for index, stamp in enumerate(obs['depth__stamp_ns']):
            stamp = int(stamp)
            od = odom_at(stamp, obs['odom__stamp_ns'], obs['odom__available_ns'], obs['odom__values'], protocol['odom_max_age_ns'])
            start = time.perf_counter_ns()
            points = project(depth[index], calibration['k'], transform, protocol['stride']) if od else np.empty((0, 3))
            projection_ms = (time.perf_counter_ns()-start)/1e6
            for name, stream in handles.items():
                start = time.perf_counter_ns()
                diagnostics = {}
                if od is None: tracks = []
                elif name=='generalized_3d_classifier':
                    tracks = pipeline.process_point_cloud(stamp, points, od['pose'])
                    diagnostics = pipeline.last_diagnostics
                else:
                    detections = old_proposals(points)
                    if name=='legacy_2d_classifier':
                        valid = []
                        for d in detections:
                            score = classifier.predict(dict(points_xyz=d['raw_points'], scan_valid=False, scan_ranges=[]))['score']
                            if score>=protocol['legacy_2d_classifier_threshold']: valid.append(d)
                        detections = valid
                    tracks = old[name].update(stamp, detections, od['pose'])
                elapsed = projection_ms+(time.perf_counter_ns()-start)/1e6
                row = dict(depth_index=index, stamp_ns=stamp, available_ns=int(obs['depth__available_ns'][index]),
                           odom_valid=od is not None, odom_stamp_ns=od['stamp_ns'] if od else -1,
                           odom_available_ns=od['available_ns'] if od else -1, tracks=tracks,
                           latency_ms=elapsed, diagnostics=diagnostics)
                stream.write(json.dumps(row, allow_nan=False)+'\n')
            if index%150==0: print(norm.name, index, '/', len(depth), flush=True)
    finally:
        for stream in handles.values(): stream.close()
    return dict(frames=len(depth), sensor_sha256={name: sha(output/(name+'.jsonl')) for name in handles})


def scene_at(scene, stamp, bracket_ns):
    times = np.array([s['stamp_ns'] for s in scene])
    hi = int(np.searchsorted(times, stamp))
    if hi==len(times) or (hi==0 and times[hi]!=stamp): return None
    lo = max(0, hi-1)
    if times[hi]!=stamp and times[hi]-times[lo]>bracket_ns: return None
    u = 1. if times[hi]==times[lo] else (stamp-times[lo])/(times[hi]-times[lo])
    a, b = scene[lo], scene[hi]
    robot = np.array([a['robot'][k] for k in ('x', 'y', 'yaw')], float)
    other = np.array([b['robot'][k] for k in ('x', 'y', 'yaw')], float)
    delta = other-robot
    delta[2] = (delta[2]+np.pi)%(2*np.pi)-np.pi
    robot += u*delta
    ids = sorted(h['id'] for h in a['humans'])
    ah = {h['id']: h for h in a['humans']}; bh = {h['id']: h for h in b['humans']}
    people = np.array([[(1-u)*ah[i][k]+u*bh[i][k] for k in ('x','y')] for i in ids])
    return ids, to_local(people, robot)


def evaluate_episode(norm, sensor, protocol):
    # Label access starts only after both sensor replays are sealed by main().
    scene = [json.loads(line) for line in (norm/'human_states.jsonl').read_text().splitlines()]
    calibration = json.loads((norm/'calibration.json').read_text())
    depth = np.load(norm/'depth.npy', mmap_mode='r', allow_pickle=False)
    frames = {name: [json.loads(l) for l in (sensor/(name+'.jsonl')).read_text().splitlines()] for name in protocol['variants']}
    stats = {name: dict(evaluated_frames=0, skipped_odom_or_gt_frames=0, proposals=0, paired=0, unpaired=0,
                       supported_gt_proxy=0, matched_supported_gt_proxy=0, id_switches_500ms=0) for name in frames}
    seen = {name: defaultdict(set) for name in frames}; last = {name: {} for name in frames}
    distances = {name: [] for name in frames}; latencies = {name: [] for name in frames}; label_rows = []
    for index in range(len(depth)):
        reference = frames[protocol['variants'][0]][index]
        stamp = reference['stamp_ns']
        gt = scene_at(scene, stamp, protocol['gt_max_bracket_ns'])
        if gt is None or not reference['odom_valid']:
            for stat in stats.values(): stat['skipped_odom_or_gt_frames'] += 1
            continue
        ids, local = gt
        points = project(depth[index], calibration['k'], np.array(calibration['optical_to_base_link']), protocol['stride'])
        proxy = protocol['sensor_supported_gt_proxy']
        support = []
        for xy in local:
            mask = (np.linalg.norm(points[:, :2]-xy, axis=1)<=proxy['xy_radius_m']) & (points[:, 2]>=proxy['z_range_m'][0]) & (points[:, 2]<=proxy['z_range_m'][1])
            support.append(int(mask.sum())>=proxy['min_depth_points'])
        for name, rows in frames.items():
            row = rows[index]
            if row['stamp_ns']!=stamp: raise ValueError('Unpaired sensor frame order')
            tracks = row['tracks']; stat = stats[name]
            paired, error = associate([v['xy_base'] for v in tracks], local, protocol['pair_gate_m'], protocol['pair_ambiguity_margin_m'])
            stat['evaluated_frames'] += 1; stat['proposals'] += len(tracks)
            stat['paired'] += int((paired>=0).sum()); stat['unpaired'] += int((paired<0).sum())
            stat['supported_gt_proxy'] += sum(support)
            stat['matched_supported_gt_proxy'] += sum(support[j] for j in paired if j>=0)
            latencies[name].append(row['latency_ms'])
            for track, j, distance in zip(tracks, paired, error):
                if j<0: continue
                distances[name].append(float(distance))
                ident, person = track['track_id'], ids[j]
                if ident>0:
                    previous = last[name].get(person)
                    if previous and stamp-previous[0]<=500_000_000 and previous[1]!=ident: stat['id_switches_500ms'] += 1
                    last[name][person] = stamp, ident
                    seen[name][person].add(ident)
            if name=='generalized_3d_classifier':
                label_rows.append(dict(depth_index=index, stamp_ns=stamp, gt_ids=ids, gt_xy_base_LABEL_ONLY=local.tolist(),
                                       depth_supported_proxy=support, geometric_matches=paired.tolist()))
    for name, stat in stats.items():
        stat['pairing_fraction'] = stat['paired']/stat['proposals'] if stat['proposals'] else None
        stat['supported_gt_proxy_coverage'] = stat['matched_supported_gt_proxy']/stat['supported_gt_proxy'] if stat['supported_gt_proxy'] else None
        stat['fragmentation_proxy'] = sum(max(0, len(v)-1) for v in seen[name].values())
        stat['latency_p50_ms'] = float(np.median(latencies[name])) if latencies[name] else None
        stat['latency_p95_ms'] = float(np.percentile(latencies[name], 95)) if latencies[name] else None
        stat['distance_p95_m'] = float(np.percentile(distances[name], 95)) if distances[name] else None
    with (sensor/'evaluation_LABEL_ONLY.jsonl').open('x') as stream:
        for row in label_rows: stream.write(json.dumps(row)+'\n')
    old, new = stats['legacy_2d_classifier'], stats['generalized_3d_classifier']
    gate = protocol['promotion_gate']
    conditions = dict(paired_retention=new['paired']>=gate['paired_retention_min']*old['paired'] and old['paired']>0,
                      unpaired_not_increased=new['unpaired']<=old['unpaired'],
                      identity_switches_not_increased=new['id_switches_500ms']<=old['id_switches_500ms'],
                      latency_budget=new['latency_p95_ms'] is not None and new['latency_p95_ms']<=gate['latency_p95_max_ms'])
    return dict(variants=stats, promotion_conditions=conditions, geometric_nonregression_pass=all(conditions.values()),
                semantic_validated=False, training_authorized=False, metric_scope=protocol['metric_limit'])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--normalized-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve(); output.mkdir(parents=True, exist_ok=False)
    root = args.normalized_root.resolve()
    protocol = json.loads((HERE/'comparison_protocol.json').read_text())
    sources = [HERE/n for n in ('compare_recordings.py','comparison_protocol.json','integrated_pipeline.py','ros_support.py','guard_tracker.py','config.json','parent/refinement.py','parent/config.json','parent/snapshots/sensor.py','parent/snapshots/association.py')]
    sources += [CLASSIFIER/n for n in ('generalized_detector.py','frozen_classifier.py','artifacts/features.py','artifacts/inference.py','artifacts/selected_model.npz','artifacts/selected_frozen.json')]
    inputs = {}
    for ep in protocol['episodes']:
        inputs[ep] = {name: sha(root/ep/name) for name in ('observations.npz','depth.npy','calibration.json','human_states.jsonl')}
    plan = dict(protocol=protocol, source_sha256={str(p):sha(p) for p in sources}, input_sha256=inputs)
    dump(output/'plan.json', plan)
    seals = {}
    for ep in protocol['episodes']:
        dest = output/ep; dest.mkdir()
        seals[ep] = infer_episode(root/ep, dest, protocol)
    dump(output/'sensor_seal.json', seals)
    results = {}
    for ep in protocol['episodes']:
        if any(sha(root/ep/name)!=h for name,h in inputs[ep].items()): raise ValueError('Input changed during replay')
        results[ep] = evaluate_episode(root/ep, output/ep, protocol)
    if any(sha(p)!=h for p,h in plan['source_sha256'].items()): raise ValueError('Source changed during replay')
    summary = dict(schema='depth_detector_replay_comparison/1', episodes=results,
                   sensor_outputs_sealed_before_gt=True, plan_sha256=sha(output/'plan.json'),
                   promotion_to_default=False, training_authorized=False)
    dump(output/'comparison.json', summary)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__': main()
