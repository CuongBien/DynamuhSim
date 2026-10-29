"""Bounded heading proposals prevent immediate backward walking during regrouping."""
import copy,math,sys,unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.simulation.group_cohesion_heading import proposals,verify_trace,PROFILE
from dynamuhsim.simulation.human_motion_profiles import validate_profile
class HeadingTests(unittest.TestCase):
 def test_reverse_regrouping_brakes_translation_and_bounds_yaw(self):
  p=[[-2,0],[0,0],[2,0]];v=[[.6,0]]*3;out,d=proposals(p,v,.8,[.8,0],[math.pi,0,0],[0,0,0],.1,1.57)
  self.assertAlmostEqual(np.linalg.norm(out[0]),0.,places=12);self.assertAlmostEqual(np.linalg.norm(out[2]),0.,places=12);self.assertAlmostEqual(abs(math.atan2(math.sin(d['target_yaws'][0]-math.pi),math.cos(d['target_yaws'][0]-math.pi))),.157);self.assertAlmostEqual(d['target_yaws'][2],.157);self.assertTrue(np.all(np.linalg.norm(out,axis=1)<=.8+1e-12))
 def test_compact_native_proposals_remain_unchanged(self):
  p=[[.72,0],[-.36,.623538],[-.36,-.623538]];v=[[.6,0]]*3;out,d=proposals(p,v,.8,[.8,0],[0,0,0],[.1,.2,.3],.1,1.57);np.testing.assert_allclose(out,v);self.assertEqual(d['target_yaws'],[.1,.2,.3]);self.assertEqual(d['active_adjustment'],[False]*3)
 def test_declared_controller_and_trace_cannot_be_silently_changed(self):
  self.assertEqual(validate_profile(PROFILE),PROFILE);bad=copy.deepcopy(PROFILE);bad['heading_controller']['low_speed_deadband_mps']=.5
  with self.assertRaises(ValueError):validate_profile(bad)
  p=[[-2,0],[0,0],[2,0]];v=[[.6,0]]*3;out,d=proposals(p,v,.8,[.8,0],[0,0,0],[0,0,0],.1,1.57);row=dict(positions_before=p,native_velocity=v,speed_cap_mps=.8,shared_velocity=[.8,0],old_yaws=[0,0,0],native_yaws=[0,0,0],dt_s=.1,rotation_rate_radps=1.57,preferred_velocity=out.tolist(),**d);rows=[dict(active_phase=True,group_cohesion=[row])];self.assertTrue(verify_trace(rows)[0]);row['target_yaws'][2]=0.;self.assertFalse(verify_trace(rows)[0])
if __name__=='__main__':unittest.main()
