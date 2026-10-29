"""Ablation changes only upper group proposal, with declared/frozen diagnostics."""
import copy,json,random,sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.simulation.group_intent import PROFILE,GroupGoal,preferred_velocity,verify_goal_intents
from dynamuhsim.simulation.human_motion_profiles import profile_config,validate_profile,enable_profile,load_profile
from dynamuhsim.simulation.human_groups import PROFILE as ORIGINAL_PROFILE

class IntentTests(unittest.TestCase):
 def test_native_goal_pref_rule_speed_zero_goal_and_finite_inputs(self):
  v=preferred_velocity((0,0),(3,4),.8);self.assertAlmostEqual(np.linalg.norm(v),.8,places=6);self.assertAlmostEqual(v[0]/v[1],3/4,places=6);self.assertEqual(preferred_velocity((1,2),(1,2),.8),(0.,0.))
  with self.assertRaises(ValueError):preferred_velocity((0,0),(float('nan'),1),.8)
  with self.assertRaises(ValueError):preferred_velocity((0,0),(1,1),-1)

 def test_per_person_callbacks_and_rng_states_are_untouched(self):
  group=NS(x=0.,y=0.,goal_x=2.,goal_y=0.,humans=[NS(id=1,x=.72,y=0,width=.72)])
  callbacks=[object(),object(),object()];e=NS(moving_interactions=[group],MAX_ADVANCE_HUMAN=.8,compute_orca_interaction_velocities=object(),compute_orca_velocities=callbacks[0],handle_collision_and_update=callbacks[1]);k=NS(env=e,motion=NS(velocities={1:(.2,0.)},integrate=callbacks[2]));np_before=copy.deepcopy(np.random.get_state());py_before=random.getstate();adapter=enable_profile(k,PROFILE);vel=e.compute_orca_interaction_velocities()
  np.testing.assert_allclose(vel,[(.8,0.)]);self.assertEqual((e.compute_orca_velocities,e.handle_collision_and_update,k.motion.integrate),tuple(callbacks));self.assertEqual(random.getstate(),py_before);after=np.random.get_state();self.assertEqual(after[0],np_before[0]);np.testing.assert_array_equal(after[1],np_before[1]);self.assertEqual(after[2:],np_before[2:]);self.assertIs(adapter.last[0]['group_avoidance'],False)

 def test_declaration_does_not_modify_the_original_profile(self):
  self.assertEqual(profile_config('group_orca_v1'),ORIGINAL_PROFILE);self.assertEqual(validate_profile(ORIGINAL_PROFILE),ORIGINAL_PROFILE);self.assertEqual(validate_profile(PROFILE),PROFILE)
  for value in [True,0]:
   bad=copy.deepcopy(PROFILE);bad['group_avoidance']=value
   with self.assertRaises(ValueError):validate_profile(bad)
  bad=copy.deepcopy(PROFILE);bad['disable_guard']=True
  with self.assertRaises(ValueError):validate_profile(bad)
  with tempfile.TemporaryDirectory() as d:self.assertIsNone(load_profile(object(),d))

 def test_trace_proposals_are_recomputed_and_forgeries_rejected(self):
  g=NS(x=0.,y=0.,goal_x=2.,goal_y=0.,humans=[NS(id=1,x=.72,y=0,width=.72)]);adapter=GroupGoal(NS(env=NS(moving_interactions=[g],MAX_ADVANCE_HUMAN=.8),motion=NS(velocities={})));adapter.velocities();rows=[dict(active_phase=True,group_intent=copy.deepcopy(adapter.last))];okay,metrics=verify_goal_intents(rows);self.assertTrue(okay);self.assertEqual(metrics['group_proposals'],1)
  bad=copy.deepcopy(rows);bad[0]['group_intent'][0]['preferred_velocity'][0]=0.;self.assertFalse(verify_goal_intents(bad)[0]);bad=copy.deepcopy(rows);bad[0]['group_orca']=[];self.assertFalse(verify_goal_intents(bad)[0]);bad=copy.deepcopy(rows);bad[0]['group_intent'][0]['group_avoidance']=True;self.assertFalse(verify_goal_intents(bad)[0])
if __name__=='__main__':unittest.main()
