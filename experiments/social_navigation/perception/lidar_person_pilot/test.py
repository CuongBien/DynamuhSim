import unittest,tempfile,json,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from sensor_adapter import prepare_scan,corroborate
from fixture import create,HERE,REPO,BASE
class Contract(unittest.TestCase):
 def geometry(self):return dict(angle_min=0.,angle_increment=2*np.pi/359,rays=360)
 def test_exact_rays_without_interpolation(self):
  raw=np.arange(1,361,dtype=np.float32);out,phi,index,meta=prepare_scan(raw,self.geometry());np.testing.assert_array_equal(out,raw[index]);self.assertEqual(len(out),359);self.assertNotIn(359,index);self.assertTrue(meta['duplicate_endpoint_removed']);self.assertFalse(meta['interpolated']);self.assertTrue(np.all(np.diff(phi)>0))
 def test_bearing_geometry_preserved(self):
  raw=np.full(360,2.,dtype=np.float32);scan,phi,index,_=prepare_scan(raw,self.geometry());old_angle=index*2*np.pi/359
  np.testing.assert_allclose(np.c_[scan*np.cos(phi),scan*np.sin(phi)],np.c_[raw[index]*np.cos(old_angle),raw[index]*np.sin(old_angle)],atol=5e-7,rtol=0)
 def test_official_invalid_replacement(self):
  raw=np.ones(360,dtype=np.float32);raw[[1,2,3]]=[0,np.inf,np.nan];a,_,_,m=prepare_scan(raw,self.geometry());self.assertEqual(int((a==np.float32(29.99)).sum()),3);self.assertEqual(m['no_return_replaced'],3)
 def test_negative_range_rejected(self):
  a=np.ones(360);a[0]=-1
  with self.assertRaises(AssertionError):prepare_scan(a,self.geometry())
 def test_no_detector_does_not_mean_nonperson(self):
  r=corroborate([[1,0]],[],[]);self.assertEqual(r[0]['semantic_status'],'unknown');self.assertFalse(r[0]['association_accepted'])
 def test_ambiguous_person_kept_unknown(self):
  rows=corroborate([[1,0],[1,.1]],[[1,.05]],[.9]);self.assertTrue(all(v['semantic_status']=='unknown' for v in rows))
 def test_only_positive_mutual_pair(self):
  rows=corroborate([[1,0],[2,0]],[[1.1,0]],[.9]);self.assertEqual(rows[0]['semantic_status'],'person_supported_experimental');self.assertEqual(rows[1]['semantic_status'],'unknown')
 def test_original_scan_definition_and_null(self):
  original=(REPO/'src/custom_corridor/models/mecanum_robot/model.sdf').read_text().replace('<sdf version="1.8">','<sdf version="1.8" xmlns:gz="urn:gz">');expected=ET.fromstring(original).find("model/link[@name='base_scan']/sensor");expected.find('topic').text='/person/scan'
  def canonical(e):return (e.tag,sorted(e.attrib.items()),(e.text or '').strip(),[canonical(c) for c in e])
  for condition in ['full','null']:
   with tempfile.TemporaryDirectory(dir=HERE/'generated') as d:
    world=ET.parse(create(Path(d),condition)).getroot().find('world');scan=world.find("model[@name='mecanum_robot']/link[@name='base_scan']/sensor");self.assertEqual(canonical(scan),canonical(expected));self.assertEqual(len(world.findall('actor')),6 if condition=='full' else 0);self.assertEqual(len(world.findall("model/link/sensor[@type='depth_camera']")),15)
 def test_gate_requires_positive_coverage(self):
  p=json.loads((HERE/'protocol.json').read_text());self.assertEqual(p['confidence_threshold'],.5);self.assertGreater(p['validation']['min_supported_visible_people'],0);self.assertFalse(p['training_ready'])
if __name__=='__main__':unittest.main()
