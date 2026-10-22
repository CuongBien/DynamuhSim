#!/usr/bin/env python3
"""Run with python -I: no ROS, simulator, project import, GPU or environment path needed."""
import json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import numpy as np
from portable_loader import Episode
if __name__=='__main__':
    episode=Episode(sys.argv[1]);n=len(episode);assert n>0
    for index in [0,n//2,n-1]:
        example=episode.observation(index);x=example['observations'];y=episode.supervision(index);m=episode.task_masks(index)
        assert 'labels' not in example and x['action'].shape==(20,3) and example['depth_history'].shape==(20,240,424)
        assert y['human_xy_future_m'].shape==(30,6,2) and y['executed_action_future'].shape==(30,3)
        assert np.all(x['history_query_ns']<=x['anchor_ns']) and np.all(y['future_stamp_ns']>x['anchor_ns'])
        assert not y['persona_valid'].any() and (y['persona_index']==-1).all()
        assert not m['sensor_track_association_valid'].any() and not m['future_bev_target_valid'] and not m['episode_training_eligible']
        assert set(np.unique(example['current_bev'])).issubset({-1,0,1})
    assert not any(name.startswith(('rclpy','rosbag','gz.')) for name in sys.modules)
    print(json.dumps(dict(status='PASS',samples=n,depth_frames=len(episode.depth),default_payload_has_no_labels=True,ros_or_simulator_imported=False,training_ready=episode.metadata['training_ready']),indent=2))
