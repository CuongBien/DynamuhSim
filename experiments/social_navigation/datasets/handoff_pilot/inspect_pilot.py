"""Inspect the portable pilot using only Python and NumPy; no raw/ROS access."""
import argparse,json,sys
from pathlib import Path

def main():
 p=argparse.ArgumentParser();p.add_argument('--dataset',type=Path,required=True);a=p.parse_args();root=a.dataset.resolve();sys.path.insert(0,str(root/'tools'));from loader import Dataset
 ds=Dataset(root);assert ds.metadata['fit_allowed'] is False;reports=[]
 for meta in ds.metadata['episodes']:
  e=ds.episode(meta['episode_id']);i=len(e)//2;x=e.observation(i);condition=e.action_condition(i);y=e.supervision(i);m=e.task_masks(i)
  assert x['depth_history'].shape==(20,240,424) and condition['executed_action_future'].shape==(30,3) and y['future_bev']['occupancy'].shape==(3,64,64)
  assert not bool(m['training_authorized']);reports.append(dict(episode=meta['episode_id'],split=meta['split'],windows=len(e),example_index=i,depth_history=list(x['depth_history'].shape),scan_history=list(x['history']['scan_ranges_m'].shape),imu_history=list(x['history']['imu'].shape),odom_history=list(x['history']['odom_local_pose_and_twist'].shape),action_condition=list(condition['executed_action_future'].shape),future_bev=list(y['future_bev']['occupancy'].shape),wm_target_valid=bool(m['wm_observed_bev_train_valid']),training_authorized=False))
 assert 'rclpy' not in sys.modules and 'rosbag2_py' not in sys.modules
 print(json.dumps(dict(status='PASS',ROS_imported=False,episodes=reports),indent=2))

if __name__=='__main__':main()
