import unittest,tempfile,json,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from labels import ownership,revealed,same_depth
from fixture import create,HERE
class Labels(unittest.TestCase):
 def test_visible(self):
  full=np.array([[2.,3.,np.inf]]);drop=np.array([[4.,3.,np.inf]]);alone=np.array([[2.,np.inf,np.inf]])
  labels,qa=ownership(full,[drop],[alone]);np.testing.assert_array_equal(labels,[[1,0,0]]);self.assertEqual(qa['uncertain_pixels'],0)
 def test_occluded_not_visible(self):
  labels,_=ownership(np.array([[2.]]),[np.array([[2.]])],[np.array([[3.]])]);self.assertEqual(labels.item(),0)
 def test_no_unmask_nearer_object(self):
  self.assertFalse(revealed(np.array([[3.]]),np.array([[2.]]),.001).item())
 def test_disagreement_stays_unknown(self):
  labels,qa=ownership(np.array([[2.]]),[np.array([[4.]])],[np.array([[2.1]])]);self.assertEqual(labels.item(),-1);self.assertEqual(qa['contradictions'],1)
 def test_two_owners_stay_unknown(self):
  labels,qa=ownership(np.array([[2.]]),[np.array([[4.]])]*2,[np.array([[2.]])]*2);self.assertEqual(labels.item(),-1);self.assertEqual(qa['overlap_pixels'],1)
 def test_inf_not_silhouette(self):self.assertFalse(same_depth(np.array([[np.inf]]),np.array([[np.inf]]),.001).item())
 def test_fixture_is_static_and_camera_only(self):
  with tempfile.TemporaryDirectory(dir=HERE/'generated') as d:
   p=create(Path(d));world=ET.parse(p).getroot().find('world');robot=world.find("model[@name='mecanum_robot']")
   self.assertEqual(robot.findtext('static'),'true');self.assertFalse(robot.findall('plugin'));self.assertEqual(len(world.findall('actor')),6)
   sensors=robot.findall('.//sensor');self.assertEqual(len(sensors),15)
   self.assertTrue(all(v.get('type')=='depth_camera' for v in sensors));self.assertFalse(any('Adapter' in v.get('name','') for v in world.findall('plugin')))
   self.assertIn('gz:expressed_in',p.read_text());self.assertNotIn('ns0:expressed_in',p.read_text())
if __name__=='__main__':unittest.main()
