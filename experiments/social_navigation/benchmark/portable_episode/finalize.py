#!/usr/bin/env python3
"""Add explicit task eligibility and relative-path metadata to the already-built episode."""
import argparse,hashlib,json,shutil
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent

def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1<<20),b''):h.update(b)
    return h.hexdigest()
def dump(p,v):p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def finalize(root):
    assert not (root/'bundle.json').exists();qa=json.loads((root/'normalized/qa.json').read_text());assert qa['status']=='PASS'
    x=np.load(root/'normalized/samples.npz',allow_pickle=False);y=np.load(root/'normalized/labels.npz',allow_pickle=False);n=len(x['anchor_ns']);assert n>0
    masks=dict(trajectory_gt_available=y['human_valid'],sensor_track_association_valid=np.zeros((n,6),dtype=bool),
        m1_track_aligned_supervision_valid=np.zeros((n,30,6),dtype=bool),persona_valid=y['persona_valid'],
        future_bev_target_valid=np.zeros(n,dtype=bool),current_depth_bev_available=x['depth_valid'][:,-1],episode_training_eligible=np.zeros(n,dtype=bool))
    np.savez(root/'supervision_masks.npz',**masks)
    roles=dict(schema='social_label_roles/1',observations='normalized/samples.npz; no human GT, persona, simulator ID or future feature',labels='normalized/labels.npz',
        trajectory_gt_available='Enough actual simulator scene positions to interpolate3s LABELS; not visibility or detections',
        sensor_track_association_valid='False: sensor tracker/association not validated; not a negative claim about visibility',
        m1_track_aligned_supervision_valid='False until input track IDs can be associated to simulator label IDs',
        persona_valid='False with persona_index=-1; standing is not automatically H4',
        future_bev_target_valid='False: only current observed depth BEV exists; target definition pending',
        current_depth_bev_available='Current depth-derived observation, not future labels or a safe planning costmap',
        episode_training_eligible='False: this export is data-format QA, not permission to train',
        future_actions='Executed Nav2 commands in LABELS at t..t+2.9s, conditions for candidate-action branch; not measured future observations',
        split='pilot_only; overlapping windows stay in same episode/seed/scene split; HuNavSim held out',
        grouping='Native group IDs/modes are labels only; lifecycle may disperse; IDs persistent within this episode only')
    dump(root/'label_roles.json',roles)
    for name in ['portable_loader.py','smoke.py']:shutil.copyfile(HERE/name,root/'tools'/name)
    info=dict(schema='social_bundle/1',episode_id='socnavgym_standing42_earlyavoid_01',split='pilot_only',training_ready=False,
        paths=dict(samples='normalized/samples.npz',labels='normalized/labels.npz',depth='normalized/depth.npy',bev='normalized/bev_observed.npz',calibration='normalized/calibration.json',task_masks='supervision_masks.npz',raw_bag='raw_bag',config_snapshot='config_snapshot',provenance='provenance.json',label_roles='label_roles.json'),
        sample_count=n,units='SI, int64 simulation ns',data_portability='NumPy/JSON reader needs neither ROS nor simulator',simulation_replay_portability='Historical SDF/launch paths require asset/environment remapping; not a standalone simulator distribution')
    dump(root/'bundle.json',info)
    files={str(f.relative_to(root)):dict(bytes=f.stat().st_size,sha256=sha(f)) for f in root.rglob('*') if f.is_file()}
    dump(root/'bundle_manifest.json',dict(schema='social_bundle_manifest/1',hash='SHA256',files=files))
    print(json.dumps(dict(status='FINALIZED',samples=n,bundle_files=len(files),bytes=sum(v['bytes'] for v in files.values()),training_ready=False),indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);a=p.parse_args();finalize(a.bundle.resolve())
