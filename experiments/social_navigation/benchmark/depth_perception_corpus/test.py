import importlib.util,json,tempfile,unittest,sys,shutil
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;spec=importlib.util.spec_from_file_location('corpus_export',HERE/'export.py');ex=importlib.util.module_from_spec(spec);spec.loader.exec_module(ex)
from loader import load_input,load_label,INPUT_KEYS
CFG=json.loads((HERE/'protocol.json').read_text())['label_rules']
class Corpus(unittest.TestCase):
 def test_pure_negative(self):self.assertEqual(ex.classify(np.zeros(12,int),CFG)[0]['class_index'],0)
 def test_person_with_small_background(self):self.assertEqual(ex.classify(np.r_[np.ones(19),0],CFG)[0]['class_index'],1)
 def test_mixed_persons_masked(self):self.assertFalse(ex.classify(np.r_[np.ones(20),[2]],CFG)[0]['label_valid'])
 def test_unknown_never_negative(self):self.assertEqual(ex.classify(np.r_[np.zeros(20),[-1]],CFG)[0]['class_index'],-1)
 def test_small_person_fragment_never_negative(self):self.assertEqual(ex.classify(np.r_[np.zeros(15),np.ones(3)],CFG)[0]['class_index'],-1)
 def test_null_unexplained_is_unknown(self):
  a=np.array([[2.]]);owner,qa=ex.label_pixels(a,[a]*6,[np.full((1,1),np.inf)]*6,np.array([[4.]]));self.assertEqual(owner.item(),-1);self.assertEqual(qa['null_effect_without_owner'],1)
 def test_splits_and_static_dedup(self):
  scenes=json.loads((HERE/'scenes_LABEL_ONLY.json').read_text());self.assertEqual(len(scenes),7);self.assertEqual(len({v['scene_id'] for v in scenes}),7)
  for family in ['A','B','C']:self.assertEqual(len({v['split'] for v in scenes if v['family']==family}),1)
  self.assertEqual(sum(len(v['selected_depth_indices']) for v in scenes),19)
  for v in scenes:self.assertEqual(v['selected_depth_indices'],list(range(5)) if v['moving'] else [2])
 def test_motion_geometry_clearance(self):
  scenes=json.loads((HERE/'scenes_LABEL_ONLY.json').read_text());minimum=100.
  for scene in scenes:
   for f in np.linspace(0,1,101):
    humans=np.array([np.array(h['start'])*(1-f)+np.array(h['end'])*f for h in scene['humans']])
    for i in range(6):
     minimum=min(minimum,float(np.linalg.norm(humans[i]-[-28,0])-.36-.2))
     for j in range(i):minimum=min(minimum,float(np.linalg.norm(humans[i]-humans[j])-.72))
     for o in scene['objects']:
      delta=humans[i]-o['pose'][:2]
      if o['shape']=='box':
       c,z=np.cos(o['pose'][5]),np.sin(o['pose'][5]);local=delta@np.array([[c,-z],[z,c]]);distance=np.linalg.norm(np.maximum(abs(local)-np.array(o['size'][:2])/2,0))-.36
      else:distance=np.linalg.norm(delta)-o['radius']-.36
      minimum=min(minimum,float(distance))
  self.assertGreater(minimum-.008,.02)
 def test_portable_smoke_and_no_label_input(self):
  source=HERE.parent/'lidar_person_pilot/generated/runs/full01'
  with tempfile.TemporaryDirectory(dir=HERE/'generated') as folder:
   root=Path(folder);(root/'inputs').mkdir();(root/'labels').mkdir();(root/'inputs/calibration_SENSOR_ONLY.json').write_bytes((HERE.parent/'lidar_person_pilot/calibration_SENSOR_ONLY.json').read_bytes())
   scene=dict(scene_id='smoke',split='train',family='development',moving=False,selected_depth_indices=[2]);summary=ex.build_scene(source,scene,root,CFG);self.assertEqual(summary['person_samples'],4);self.assertEqual(summary['environment_samples'],2);self.assertEqual(summary['ambiguous_samples'],0);(root/'manifest.json').write_text(json.dumps(dict(scenes=[summary])))
   sample=load_input(root,'smoke',0);self.assertEqual(set(sample),INPUT_KEYS);label=load_label(root,'smoke',0);self.assertIn('class_index',label);self.assertNotIn('class_index',sample)
   # Rewrite labels with contradictory values: sensor-only sample must remain identical.
   target=root/'labels/smoke/proposal_labels_LABEL_ONLY.npz';a=np.load(target);values={k:a[k].copy() for k in a.files};values['class_index'][:]=1-values['class_index'];np.savez_compressed(target,**values);other=load_input(root,'smoke',0)
   for k,v in sample.items():
    if isinstance(v,np.ndarray):np.testing.assert_array_equal(v,other[k])
    else:self.assertEqual(v,other[k])
   # Removing all future scans cannot alter this sample.
   p=root/'inputs/smoke/lidar_SENSOR_ONLY.npz';a=np.load(p);keep=a['stamp_ns']<=sample['depth_stamp_ns'];np.savez_compressed(p,**{k:a[k][keep] for k in a.files});prefix=load_input(root,'smoke',0);np.testing.assert_array_equal(sample['scan_ranges'],prefix['scan_ranges'])
   summary['split']='test';(root/'manifest.json').write_text(json.dumps(dict(scenes=[summary])))
   with self.assertRaises(PermissionError):load_input(root,'smoke',0)
   self.assertEqual(set(load_input(root,'smoke',0,allow_test=True)),INPUT_KEYS)
if __name__=='__main__':unittest.main()
