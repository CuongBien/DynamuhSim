"""Checks tying final packaged helpers and timestamps back to captured artifacts."""
import hashlib,json
import numpy as np

def verify(root):
    provenance=json.loads((root/'provenance.json').read_text());manifest=json.loads((root/'config_snapshot/driver/manifest.json').read_text());sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
    for name in ['namespace_prepare.py','moderate_config.py','config_runtime.py']:
        assert sha(root/'config_snapshot/helpers'/name)==manifest['source_sha256'][name]
    assert sha(root/'config_snapshot/driver/driver.py')==manifest['driver_sha256']
    for name in ['worker.py','reactive_motion.py','standing_projection.py','fixture.py','config.yaml']:
        key='experiments/social_navigation/benchmark/socnavgym_group_nav2/'+name
        assert sha(root/'config_snapshot/human_worker'/name)==provenance['capture_sources'][key]
    x=np.load(root/'normalized/samples.npz',allow_pickle=False);bad=~x['goal_valid'];ages=x['anchor_ns'][bad]-x['goal_stamp_ns'][bad]
    assert np.all(x['goal_xy_from_amcl'][bad]==0) and np.all(ages>1_000_000_000)
    return dict(status='PASS',helper_snapshots_match_captured_driver_manifest=True,driver_sha256_matches=True,human_worker_snapshots_match_capture=True,
        invalid_goal_count=int(bad.sum()),invalid_goal_vectors_zero=True,invalid_goal_min_age_s=float(ages.min()/1e9) if len(ages) else None,
        depth_age_at_anchor_min_s=float(np.min((x['anchor_ns']-x['depth_stamp_ns'][:,-1])/1e9)),
        depth_age_at_anchor_max_s=float(np.max((x['anchor_ns']-x['depth_stamp_ns'][:,-1])/1e9)),
        bev_temporal_frame='base_link at depth measurement; no ego-motion compensation to anchor')
