"""Static bottleneck geometry/map agree and ordinary six-person recording is unchanged."""
import json,sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.simulation.static_gap import fixture,map_array,write_assets,configure_world
from dynamuhsim.runtime.capture_recording import Recording,TOPICS
class StaticGapTests(unittest.TestCase):
 def test_map_opening_matches_sdf_width_within_one_cell(self):
  for case in ['wide','narrow']:
   spec=fixture(case);grid=map_array(spec);meta=spec['map'];col=int((-24-meta['origin'][0])/meta['resolution']);free=np.where(grid[:,col]==254)[0];y=meta['origin'][1]+(free+.5)*meta['resolution'];y=y[abs(y)<3.35]
   self.assertLessEqual(abs(len(y)*meta['resolution']-spec['gap_width_m']),meta['resolution']+1e-8)
   self.assertTrue(np.all(abs(y)<spec['gap_width_m']/2))
   top,bottom=spec['boxes'][-2:];self.assertAlmostEqual(top['center'][1]-top['size'][1]/2,spec['gap_width_m']/2);self.assertAlmostEqual(bottom['center'][1]+bottom['size'][1]/2,-spec['gap_width_m']/2)
   self.assertAlmostEqual(top['center'][1]+top['size'][1]/2,3.35)
 def test_runtime_world_has_only_fixture_geometry_and_camera(self):
  tree=ET.ElementTree(ET.fromstring('<sdf><world name="social_warehouse"><model name="old"/><actor name="old_human"/><model name="common_observer"><pose>0 0 0 0 0 0</pose></model></world></sdf>'))
  spec=fixture('narrow');configure_world(tree,spec);world=tree.getroot().find('world');self.assertFalse(world.findall('actor'));self.assertEqual(len(world.findall('model')),8)
  for b in spec['boxes']:
   m=world.find("model[@name='%s']"%b['name']);self.assertEqual(m.findtext('static'),'true');self.assertEqual(m.findtext('link/collision/geometry/box/size'),m.findtext('link/visual/geometry/box/size'))
  with tempfile.TemporaryDirectory() as d:
   write_assets(d,spec);raw=(Path(d)/'gap.pgm').read_bytes().split(b'\n',3)[3];np.testing.assert_array_equal(np.frombuffer(raw,dtype='u1').reshape(160,280),np.flipud(map_array(spec)))
 def test_default_recorder_still_requires_human_ground_truth_subscription(self):
  recorder=Recording.__new__(Recording);recorder.process=NS(poll=lambda:None);recorder.observed={t:{'count':5} for t in ['/camera/depth/image_rect_raw','/camera/camera_info']};recorder.node=NS(get_subscriptions_info_by_topic=lambda t:[] if t=='/common/ground_truth' else [NS(node_name='dataset_pilot_recorder')])
  self.assertFalse(recorder.ready());self.assertTrue(recorder.ready(required_topics=[t for t in TOPICS[:12] if t!='/common/ground_truth']))
 def test_zero_human_policy_is_explicit_and_native_costmaps_required(self):
  default=json.loads((ROOT/'configs/audit/capture_native_geometry.json').read_text());gap=json.loads((ROOT/'configs/audit/static_gap.json').read_text())
  self.assertNotIn('scene_human_count',default);self.assertEqual(gap['scene_human_count'],0);self.assertEqual(gap['capture_schema'],'dynamuhsim_static_gap_capture/1');self.assertIn('/global_costmap/costmap_raw',gap['required_topics'])
if __name__=='__main__':unittest.main()
