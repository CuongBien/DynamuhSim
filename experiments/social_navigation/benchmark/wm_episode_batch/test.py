import json,tempfile,unittest,sys
from pathlib import Path
import numpy as np
from loader import Dataset
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'tools/legacy'))
from dataset_core import causal_indices
sys.path.insert(0,str(HERE.parent/'perception_target_pilot'))
from geometry import warp,raster,interpolate,nearest_frame
from sensor import project
class WMData(unittest.TestCase):
 def fixture(self,root):
  (root/'dataset.json').write_text(json.dumps(dict(episodes=[dict(episode_id='a',split='train'),dict(episode_id='b',split='test')])));ep=root/'episodes/a';(ep/'inputs').mkdir(parents=True);(ep/'labels').mkdir();x=dict(anchor_ns=np.arange(4,dtype=np.int64),depth_index=np.zeros((4,20),np.int32));np.savez(ep/'inputs/samples_SENSOR_ONLY.npz',**x);np.save(ep/'inputs/depth_SENSOR_ONLY.npy',np.ones((2,2,3),np.float32));np.savez(ep/'inputs/bev_anchor_estimated_SENSOR_ONLY.npz',occupancy=np.zeros((4,64,64),np.int8),frame_valid=np.ones(4,bool));(ep/'inputs/calibration_SENSOR_ONLY.json').write_text('{}');np.savez(ep/'labels/future_trajectories_actions_LABEL_ONLY.npz',human_ids=np.arange(6),executed_action_future=np.ones((4,30,3)),executed_action_query_ns=np.ones((4,30),np.int64),executed_action_stamp_ns=np.zeros((4,30),np.int64),executed_action_available_ns=np.zeros((4,30),np.int64));np.savez(ep/'labels/future_observed_bev_LABEL_ONLY.npz',offset_ns=np.array([500000000,1000000000,3000000000]),occupancy=np.ones((4,3,64,64),np.int8));np.savez(ep/'labels/task_masks_LABEL_ONLY.npz',wm_observed_bev_train_valid=np.ones(4,bool),persona_valid=np.zeros((4,6),bool));return Dataset(root)
 def test_future_and_late_delivery_excluded(self):
  stamps=np.array([0,10,20]);available=np.array([0,25,20]);i,v=causal_indices(stamps,available,np.array([15]),100);self.assertEqual(i.item(),0);self.assertTrue(v.item());stamps[-1]=2000;i2,v2=causal_indices(stamps,available,np.array([15]),100);np.testing.assert_array_equal(i,i2)
 def test_depth_unknown_never_free(self):
  a=np.array([[np.nan,np.inf]]);points=project(a,[1,0,0,0,1,0,0,0,1],np.eye(4),stride=1);self.assertEqual(len(points),0);self.assertTrue((raster(points,np.zeros(3))==-1).all())
 def test_se2_warp_and_roundtrip(self):
  points=np.array([[2.,0.,.4],[3.,1.,.2]]);source=np.array([0.,0.,0.]);anchor=np.array([1.,0.,np.pi/2]);w=warp(points,source,anchor);np.testing.assert_allclose(w[0],[0.,-1.,.4],atol=1e-12);np.testing.assert_allclose(warp(w,anchor,source),points,atol=1e-12)
 def test_target_interpolation_gap_mask(self):
  _,valid=interpolate(np.array([0,200000000]),np.array([[0.,0.,0.],[1.,0.,0.]]),np.array([100000000]),max_gap_ns=100000000);self.assertFalse(valid.item())
 def test_nearest_target_tolerance(self):
  _,valid=nearest_frame(np.array([0,100000000]),np.array([300000000]),50000000);self.assertFalse(valid.item())
 def test_labels_not_loaded_by_observation(self):
  with tempfile.TemporaryDirectory(dir=HERE/'generated') as d:
   root=Path(d);dataset=self.fixture(root);e=dataset.episode('a');a=e.observation(3)
   for f in (root/'episodes/a/labels').iterdir():f.unlink()
   b=dataset.episode('a').observation(3);np.testing.assert_array_equal(a['depth_history'],b['depth_history']);np.testing.assert_array_equal(a['bev_anchor_estimated'],b['bev_anchor_estimated']);self.assertNotIn('human',str(b['history'].keys()));self.assertIsNone(e.y)
 def test_global_horizon_vector_and_explicit_action_condition(self):
  with tempfile.TemporaryDirectory(dir=HERE/'generated') as d:
   e=self.fixture(Path(d)).episode('a');obs=e.observation(3);self.assertNotIn('future',str(obs.keys()));y=e.supervision(3);self.assertEqual(y['future_bev']['offset_ns'].shape,(3,));self.assertEqual(e.action_condition(3)['executed_action_future'].shape,(30,3));self.assertFalse(e.task_masks(3)['persona_valid'].any())
 def test_test_denied_before_files(self):
  with tempfile.TemporaryDirectory(dir=HERE/'generated') as d:
   ds=self.fixture(Path(d))
   with self.assertRaises(PermissionError):ds.episode('b')
if __name__=='__main__':unittest.main()
