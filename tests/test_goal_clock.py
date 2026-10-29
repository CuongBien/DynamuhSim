"""Simulation-time deadlines preserve geometry, native setters and freeze ownership."""
import copy,json,sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.simulation.goal_clock import GoalClock,load_goal_clock,PROFILE,verify_trace
from dynamuhsim.runtime.capture import verify_run,dump,sha

class Human:
 def __init__(self):
  self.id=1;self.x=self.y=0.;self.goal_x=10.;self.goal_y=0.;self.width=.72;self.goal_radius=.25;self.type='dynamic';self.initial_time=0.;self.wall=0.;self.set_calls=0
 def set_goal(self,x,y):self.goal_x=x;self.goal_y=y;self.initial_time=self.wall;return 'native-goal'
 def set(self,x,y):self.goal_x=x;self.goal_y=y;self.initial_time=self.wall;self.set_calls+=1;return 'native-set'
 def has_reached_goal(self,offset=None):return self.wall-self.initial_time>50

def kernel(h):return NS(env=NS(static_humans=[],dynamic_humans=[h],moving_interactions=[],static_interactions=[],h_l_interactions=[]))

class ClockTests(unittest.TestCase):
 def test_waiting_wall_time_cannot_expire_and_sim_deadline_is_strict(self):
  h=Human();k=kernel(h);clock=GoalClock(k);h.wall=10000
  self.assertFalse(h.has_reached_goal());self.assertEqual(clock.snapshot()['elapsed_active_sim_s'],0.)
  clock.begin_step(150.,100.);self.assertFalse(h.has_reached_goal());clock.end_step(150.1)
  clock.begin_step(150.1,100.);self.assertTrue(h.has_reached_goal());self.assertAlmostEqual(clock.events[-1]['age_sim_s'],50.1)
  self.assertEqual(h.set_goal(10.,0.),'native-goal');self.assertFalse(h.has_reached_goal());self.assertEqual(clock.snapshot()['humans']['1']['age_sim_s'],0.)

 def test_geometric_arrival_offsets_and_static_rule_are_retained(self):
  h=Human();h.goal_x=.61;clock=GoalClock(kernel(h))
  self.assertFalse(h.has_reached_goal());h.set_goal(.60,0.);self.assertTrue(h.has_reached_goal());self.assertFalse(h.has_reached_goal(offset=0));self.assertTrue(h.has_reached_goal(offset=.4))
  h.type='static';h.goal_x=h.goal_y=None;clock.entries[1]['target']=[None,None];clock.begin_step(100.,0.);self.assertFalse(h.has_reached_goal())

 def test_native_setter_side_effects_and_same_target_resets_survive(self):
  h=Human();clock=GoalClock(kernel(h));clock.begin_step(49.,0.);self.assertEqual(h.set(10.,0.),'native-set');self.assertEqual(h.set_calls,1);self.assertEqual(clock.snapshot()['humans']['1']['assigned_sim_s'],49.);clock.end_step(49.1);clock.begin_step(49.1,0.);h.set_goal(10.,0.);self.assertEqual(clock.snapshot()['humans']['1']['age_sim_s'],0.)

 def test_population_replacement_untracked_mutation_and_clock_discontinuity_are_terminal(self):
  h=Human();k=kernel(h);clock=GoalClock(k);clock.begin_step(1.,0.);clock.end_step(1.1)
  with self.assertRaisesRegex(ValueError,'Noncontiguous'):clock.begin_step(2.,0.)
  with self.assertRaisesRegex(ValueError,'gate changed'):clock.begin_step(1.1,.2)
  h.goal_x=11.
  with self.assertRaisesRegex(RuntimeError,'outside'):clock.begin_step(1.1,0.)
  h.goal_x=10.;k.env.dynamic_humans=[Human()]
  with self.assertRaisesRegex(RuntimeError,'replaced'):clock.begin_step(1.1,0.)

 def test_trace_reconstructs_ages_and_detects_forged_expiry_or_age(self):
  h=Human();clock=GoalClock(kernel(h));ready=copy.deepcopy(clock.snapshot());rows=[dict(active_phase=False,time=2.,state_time=2.1,goal_clock=copy.deepcopy(clock.snapshot()))]
  clock.begin_step(10.02,10.);h.has_reached_goal();clock.end_step(10.12);rows.append(dict(active_phase=True,time=10.02,state_time=10.12,goal_clock=copy.deepcopy(clock.snapshot())))
  clock.begin_step(10.12,10.);h.set_goal(5.,0.);h.has_reached_goal();clock.end_step(10.22);rows.append(dict(active_phase=True,time=10.12,state_time=10.22,goal_clock=copy.deepcopy(clock.snapshot())))
  okay,metrics=verify_trace(rows,10.,ready);self.assertTrue(okay);self.assertEqual(metrics['assignments'],1)
  bad=copy.deepcopy(rows);bad[-1]['goal_clock']['humans']['1']['age_sim_s']+=1;self.assertFalse(verify_trace(bad,10.,ready)[0])
  bad=copy.deepcopy(rows);bad[1]['goal_clock']['events'][-1]['expired']=True;self.assertFalse(verify_trace(bad,10.,ready)[0])

 def test_absent_profile_leaves_native_methods_unchanged(self):
  h=Human();before=h.has_reached_goal
  with tempfile.TemporaryDirectory() as d:self.assertIsNone(load_goal_clock(object(),d));self.assertEqual(h.has_reached_goal,before)
  h.wall=51.;self.assertTrue(h.has_reached_goal())

 def test_clock_asset_is_mandatory_in_freeze(self):
  with tempfile.TemporaryDirectory() as d:
   root=Path(d);run=root/'run';(run/'assets').mkdir(parents=True)
   for name,value in [('plan.json',{'human_goal_clock':'sim_v1'}),('preflight.json',{}),('assets/mecanum_camera.sdf',{}),('assets/human_goal_clock.json',PROFILE)]:dump(run/name,value)
   files=[p for p in run.rglob('*') if p.is_file()];freeze=dict(schema='dynamuhsim_capture_freeze/2',source_sha256={str(p.relative_to(root)):sha(p) for p in files});dump(run/'capture_freeze.json',freeze)
   with patch('dynamuhsim.runtime.capture.capture_sources',return_value=[]):
    verify_run(root,run);del freeze['source_sha256']['run/assets/human_goal_clock.json'];dump(run/'capture_freeze.json',freeze)
    with self.assertRaisesRegex(RuntimeError,'clock missing'):verify_run(root,run)
if __name__=='__main__':unittest.main()
