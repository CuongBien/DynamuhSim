"""Persistent native flags and a separately declared zero-query clock contract."""
import copy,json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
from dynamuhsim.simulation.conversation import PROFILE, PersistentConversation,load_conversation,validate_profile,verify_conversation
from dynamuhsim.simulation.goal_clock import GoalClock,PROFILE as CLOCK,verify_trace
from dynamuhsim.analysis.standing_clock import verify_snapshots
from dynamuhsim.runtime.capture import dump,sha,verify_run,prepare_run
from test_goal_clock import Human
ROOT=Path(__file__).resolve().parents[1]


def fixture():
 humans=[Human() for _ in range(6)]
 for i,h in enumerate(humans,1):h.id=i
 groups=[NS(humans=humans[:3],can_disperse=True),NS(humans=humans[3:],can_disperse=True)]
 return NS(env=NS(static_humans=[],dynamic_humans=[],static_interactions=groups,moving_interactions=[],h_l_interactions=[])),humans


def trace():
 k,hs=fixture();mode=PersistentConversation(k);clock=GoalClock(k);ready=copy.deepcopy(clock.snapshot())
 def row(active,t):
  groups=[dict(type='standing',members=[1,2,3]),dict(type='standing',members=[4,5,6])]
  humans=[dict(id=h.id,x=h.x,y=h.y,yaw=0.) for h in hs]
  return dict(time=t,state_time=t+.1,active_phase=active,groups=groups,groups_before=copy.deepcopy(groups),before=humans,humans=copy.deepcopy(humans),human_lifecycle=mode.snapshot(),goal_clock=copy.deepcopy(clock.snapshot()))
 rows=[row(False,.5)]
 for t in [1.,1.1,1.2]:clock.begin_step(t,1.);clock.end_step(t+.1);rows.append(row(True,t))
 return rows,ready


class ConversationTests(unittest.TestCase):
 def test_absent_input_keeps_native_dispersal_unchanged(self):
  k,hs=fixture()
  with tempfile.TemporaryDirectory() as directory:self.assertIsNone(load_conversation(k,directory))
  self.assertTrue(all(g.can_disperse for g in k.env.static_interactions))

 def test_only_native_flag_changes_no_goals_positions_or_methods(self):
  k,hs=fixture();before=[copy.deepcopy(h.__dict__) for h in hs];methods=[h.set_goal for h in hs];mode=PersistentConversation(k)
  self.assertEqual(before,[h.__dict__ for h in hs]);self.assertEqual(methods,[h.set_goal for h in hs]);self.assertTrue(all(g.can_disperse is False for g in k.env.static_interactions));self.assertEqual(mode.snapshot()['profile'],PROFILE['profile'])

 def test_invalid_population_rejected_before_any_flag_changes(self):
  k,hs=fixture();k.env.static_interactions[1].humans=[hs[3],hs[4]]
  with self.assertRaises(ValueError):PersistentConversation(k)
  self.assertTrue(all(g.can_disperse for g in k.env.static_interactions))
  value=copy.deepcopy(PROFILE);value['can_disperse']=0
  with self.assertRaises(ValueError):validate_profile(value)

 def test_changed_membership_flag_or_identity_is_terminal(self):
  for kind in ['flag','identity','population']:
   k,hs=fixture();mode=PersistentConversation(k)
   if kind=='flag':k.env.static_interactions[0].can_disperse=True
   elif kind=='identity':k.env.static_interactions[0].humans[0]=copy.deepcopy(hs[0])
   else:k.env.dynamic_humans=[hs[0]]
   with self.assertRaises(RuntimeError):mode.snapshot()

 def test_trace_rejects_hidden_dispersal_and_group_changes(self):
  rows,ready=trace();self.assertTrue(verify_conversation(rows)[0])
  bad=copy.deepcopy(rows);bad[-1]['human_lifecycle']['groups'][0]['can_disperse']=True;self.assertFalse(verify_conversation(bad)[0])
  bad=copy.deepcopy(rows);bad[-1]['groups'].pop();self.assertFalse(verify_conversation(bad)[0])

 def test_zero_query_snapshots_pass_only_new_explicit_contract(self):
  rows,ready=trace();self.assertFalse(verify_trace(rows,1.,ready)[0]);good,metrics=verify_snapshots(rows,1.,ready);self.assertTrue(good);self.assertEqual(metrics['human_age_snapshots'],18);self.assertEqual(metrics['arrival_expiry_query_coverage'],'NOT_EXERCISED');self.assertEqual(metrics['evaluations'],0)

 def test_forged_clock_ages_goals_missing_activation_query_and_gaps_rejected(self):
  rows,ready=trace()
  for corruption in ['age','target','activation','query','gap','nan','membership']:
   bad=copy.deepcopy(rows)
   if corruption=='age':bad[-1]['goal_clock']['humans']['1']['age_sim_s']+=.1
   elif corruption=='target':bad[-1]['goal_clock']['humans']['1']['goal']=[2.,3.]
   elif corruption=='activation':bad[1]['goal_clock']['events']=[]
   elif corruption=='query':bad[-1]['goal_clock']['events']=[dict(event='checked')]
   elif corruption=='gap':bad[-1]['time']+=.2
   elif corruption=='nan':bad[-1]['goal_clock']['humans']['1']['age_sim_s']=float('nan')
   else:bad[-1]['groups'][0]['members']=[1,2]
   self.assertFalse(verify_snapshots(bad,1.,ready)[0],corruption)
  self.assertFalse(verify_snapshots(rows[:1],1.,ready)[0])

 def test_lifecycle_asset_is_mandatory_and_cannot_select_snapshot_without_it(self):
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory);run=root/'run';(run/'assets').mkdir(parents=True);plan=dict(scenario='standing',human_goal_clock='sim_v1',human_lifecycle_profile=PROFILE['profile'],goal_clock_validation=PROFILE['clock_validation'])
   for name,value in [('plan.json',plan),('preflight.json',{}),('assets/mecanum_camera.sdf',{}),('assets/human_goal_clock.json',CLOCK),('assets/human_lifecycle.json',PROFILE)]:dump(run/name,value)
   freeze=dict(schema='dynamuhsim_capture_freeze/2',source_sha256={str(p.relative_to(root)):sha(p) for p in run.rglob('*') if p.is_file()});dump(run/'capture_freeze.json',freeze)
   with patch('dynamuhsim.runtime.capture.capture_sources',return_value=[]):
    verify_run(root,run);del freeze['source_sha256']['run/assets/human_lifecycle.json'];dump(run/'capture_freeze.json',freeze)
    with self.assertRaisesRegex(RuntimeError,'lifecycle missing'):verify_run(root,run)

 def test_prepare_rejects_wrong_scene_or_clock_before_creating_run(self):
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory);config=root/'config.json';config.write_bytes((ROOT/'configs/capture/geometry_v7_mppi_resample.json').read_bytes())
   for scenario,clock,motion in [('walking','sim_v1','legacy'),('standing','native_wall','legacy'),('standing','sim_v1','group_cohesion_v2')]:
    with self.assertRaisesRegex(ValueError,'Persistent conversation requires'):prepare_run(root,root/'run',config,scenario,46,human_lifecycle_profile=PROFILE['profile'],human_goal_clock=clock,human_motion_profile=motion)
    self.assertFalse((root/'run').exists())
