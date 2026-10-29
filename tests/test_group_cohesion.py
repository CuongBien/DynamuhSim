"""Cohesion acts only on proposals and remains independently reconstructable."""
import copy,sys,unittest
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.simulation.group_cohesion import proposals,GroupCohesion,PROFILE,verify_cohesion_trace
from dynamuhsim.simulation.human_motion_profiles import validate_profile
class CohesionTests(unittest.TestCase):
 def test_compact_group_keeps_velocity_and_spread_gates_leaders(self):
  p=[[.72,0],[-.36,.623538],[-.36,-.623538]];v=[[.6,0]]*3;out,d=proposals(p,v,.8);np.testing.assert_allclose(out,v);self.assertEqual(d['forward_gate'],1.)
  out,d=proposals([[-2,0],[0,0],[2,0]],v,.8);self.assertEqual(d['forward_gate'],0.);self.assertGreater(out[0,0],0);self.assertLess(out[2,0],0);self.assertTrue(np.all(np.linalg.norm(out,axis=1)<=.8+1e-10))
 def test_native_runs_once_and_guard_callback_is_unchanged(self):
  hs=[NS(id=i+1,x=x,y=y,orientation=.1,width=.72) for i,(x,y) in enumerate([[.72,0],[-.36,.623538],[-.36,-.623538]])];starts={h.id:(h.x,h.y,h.orientation) for h in hs};calls=[]
  def native():
   calls.append(1)
   for h in hs:h.x+=.05
  guard=object();motion=NS(start=starts,original={'handle_collision_and_update':native},velocities={});e=NS(TIMESTEP=.1,MAX_ADVANCE_HUMAN=.8,moving_interactions=[NS(humans=hs)],handle_collision_and_update=guard);k=NS(env=e,motion=motion);model=GroupCohesion(k);motion.original['handle_collision_and_update']();self.assertEqual(calls,[1]);self.assertIs(e.handle_collision_and_update,guard)
  for h in hs:self.assertAlmostEqual(h.x,starts[h.id][0]+.05);self.assertEqual(h.orientation,.1)
  rows=[dict(active_phase=True,group_cohesion=copy.deepcopy(model.cohesion_last))];self.assertTrue(verify_cohesion_trace(rows)[0]);rows[0]['group_cohesion'][0]['preferred_velocity'][0][0]=0.;self.assertFalse(verify_cohesion_trace(rows)[0])
 def test_parameter_mutation_is_not_silent_tuning(self):
  self.assertEqual(validate_profile(PROFILE),PROFILE);bad=copy.deepcopy(PROFILE);bad['cohesion']['attraction_gain_per_s']=0.
  with self.assertRaises(ValueError):validate_profile(bad)
if __name__=='__main__':unittest.main()
