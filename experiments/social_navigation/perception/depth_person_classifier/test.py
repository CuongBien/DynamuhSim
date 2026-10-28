import json,tempfile,unittest
from pathlib import Path
import numpy as np
import torch
from features import extract,DEPTH_NAMES,LIDAR_NAMES
from data import load_split,training_weights
from model import Model,portable_arrays
from inference import normalized,numpy_logits
from metrics import binary,measure,gate,rank
class Classifier(unittest.TestCase):
 def sample(self):
  rng=np.random.default_rng(19);points=rng.normal(0,.08,(50,3))+[2.,.1,.8];return dict(points_xyz=points,scan_ranges=np.full(360,np.nan),scan_valid=False)
 def geometry(self):return dict(angle_min=0.,angle_increment=2*np.pi/359,range_min=.12,range_max=3.5,rays=360,frame_id='base_scan')
 def test_feature_shape_finite(self):
  d,l,m=extract(self.sample(),self.geometry());self.assertEqual(len(DEPTH_NAMES),30);self.assertEqual(len(LIDAR_NAMES),8);self.assertTrue(np.isfinite(d).all());self.assertFalse(m);np.testing.assert_array_equal(l,np.zeros(8))
 def test_depth_no_absolute_xy(self):
  s=self.sample();d=extract(s,self.geometry())[0];s['points_xyz']=s['points_xyz']+[7,-3,0];np.testing.assert_allclose(d,extract(s,self.geometry())[0],atol=1e-6,rtol=0)
 def test_permutation_invariant(self):
  s=self.sample();d=extract(s,self.geometry())[0];s['points_xyz']=s['points_xyz'][::-1];np.testing.assert_allclose(d,extract(s,self.geometry())[0],atol=1e-6,rtol=0)
 def test_labels_cannot_affect_features(self):
  s=self.sample();a=extract(s,self.geometry());s.update(class_index=1,person_id=999,future_pose=[40,80],H5=True);b=extract(s,self.geometry())
  for x,y in zip(a,b):np.testing.assert_array_equal(x,y)
 def test_missing_scan_not_interpreted(self):
  s=self.sample();a=extract(s,self.geometry());s['scan_ranges'][:]=2.;b=extract(s,self.geometry())
  for x,y in zip(a,b):np.testing.assert_array_equal(x,y)
 def test_native_local_scan_patch(self):
  s=self.sample();s['points_xyz'][:,1]=0.;s['scan_ranges'][:]=2.;s['scan_valid']=True;d,l,m=extract(s,self.geometry());self.assertTrue(m);self.assertTrue(np.isfinite(l).all());self.assertGreater(l[0],np.log1p(3))
 def test_weights_balance_scene_frame_class(self):
  specs=[('a',0,[0,0,1]),('a',1,[0,1]),('b',0,[0,0,0,1])];rows=[];y=[]
  for sid,frame,labels in specs:
   for label in labels:rows.append(dict(scene_id=sid,frame_index=frame));y.append(label)
  data=dict(rows=rows,y=np.array(y),label_valid=np.ones(len(y),bool));w=training_weights(data)
  for sid in ['a','b']:self.assertAlmostEqual(float(w[[r['scene_id']==sid for r in rows]].sum()),.5,places=6)
  for sid,frame,_ in specs:
   keep=np.array([r['scene_id']==sid and r['frame_index']==frame for r in rows]);self.assertAlmostEqual(float(w[keep&(data['y']==0)].sum()),float(w[keep&(data['y']==1)].sum()),places=6)
 def test_metrics_and_invalid_mask(self):
  a=binary([0,0,1,1],[.1,.7,.8,.2],.5);self.assertEqual([a[k] for k in ['tp','fn','fp','tn']],[1,1,1,1]);data=dict(y=np.array([0,1,-1]),label_valid=np.array([1,1,0],bool),rows=[dict(scene_id='a')]*3);m=measure(data,np.array([.1,.9,.99]),.5);self.assertEqual(m['overall']['samples'],2);self.assertEqual(m['overall']['balanced_accuracy'],1.)
 def test_test_split_denied_before_files(self):
  with self.assertRaises(PermissionError):load_split('test',Path('/not-a-real-path'))
 def test_cuda_numpy_parity_and_missing_residual(self):
  self.assertTrue(torch.cuda.is_available());torch.manual_seed(17);torch.backends.cuda.matmul.allow_tf32=False;rng=np.random.default_rng(19);data=dict(depth=rng.normal(size=(11,30)).astype(np.float32),lidar=rng.normal(size=(11,8)).astype(np.float32),lidar_mask=np.array([True]*8+[False]*3));stats=dict(depth_mean=np.zeros(30,np.float32),depth_std=np.ones(30,np.float32),lidar_mean=np.zeros(8,np.float32),lidar_std=np.ones(8,np.float32))
  for hidden in [0,16]:
   for modality in ['depth','fusion']:
    model=Model(hidden,modality).cuda();model.eval();arrays=portable_arrays(model,stats)
    with torch.inference_mode():gpu=model(*[torch.from_numpy(v).cuda() for v in normalized(data,stats)]).cpu().numpy()
    np.testing.assert_allclose(gpu,numpy_logits(data,arrays,hidden,modality),atol=1e-6,rtol=1e-6)
    data2={**data,'lidar':data['lidar'].copy()};data2['lidar'][~data['lidar_mask']]=1e6;np.testing.assert_array_equal(numpy_logits(data,arrays,hidden,modality)[-3:],numpy_logits(data2,arrays,hidden,modality)[-3:])
if __name__=='__main__':unittest.main()
