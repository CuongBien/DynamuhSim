"""Encounters are opt-in, initially safe, frozen inputs; no runtime forcing."""
import copy,json,math,sys,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.simulation.encounters import fixture,validate_fixture,initial_humans,initial_clearance,initialize,load_fixture
from dynamuhsim.runtime.capture import verify_run,dump,sha

class EncounterTests(unittest.TestCase):
 def test_explicit_poses_are_safe_and_headings_match_initial_intent(self):
  boxes=json.loads((ROOT/'experiments/social_navigation/warehouse/layout.json').read_text())['obstacles']
  for name in ['head_on','crossing']:
   spec=fixture(name);self.assertGreater(min(initial_clearance(spec,boxes).values()),.002);self.assertEqual([p['id'] for p in initial_humans(spec)],list(range(1,7)))
   for g in spec['groups']:
    delta=[b-a for a,b in zip(g['center'],g['goal'])];self.assertAlmostEqual(math.cos(g['yaw']),delta[0]/math.hypot(*delta));self.assertAlmostEqual(math.sin(g['yaw']),delta[1]/math.hypot(*delta))
  spec=fixture('head_on');spec['groups'][0]['center'][0]=-28
  with self.assertRaises(ValueError):validate_fixture(spec)
  bad=copy.deepcopy(boxes);bad.append(dict(x=-24.5,y=.35,sx=3,sy=3))
  with self.assertRaisesRegex(ValueError,'intersects'):initial_clearance(fixture('head_on'),bad)

 def test_absent_fixture_does_not_access_or_mutate_kernel(self):
  with tempfile.TemporaryDirectory() as d:self.assertIsNone(load_fixture(object(),d))

 def test_initialize_changes_only_declared_initial_groups_and_retains_motion_hooks(self):
  spec=fixture('crossing');boxes=json.loads((ROOT/'experiments/social_navigation/warehouse/layout.json').read_text())['obstacles'];goals={};groups=[]
  for members in [[1,2,3],[4,5,6]]:
   hs=[NS(id=i,x=0,y=0,orientation=0,speed=1,position_history=[0]) for i in members]
   class Group:
    def __init__(self,humans):self.humans=humans
    def set_goal(self,x,y):self.goal_x,self.goal_y=x,y
   groups.append(Group(hs))
   for h in hs:goals[h.id]=NS(x=0,y=0)
  e=NS(static_interactions=[],static_humans=[],dynamic_humans=[],h_l_interactions=[],moving_interactions=groups,goals=goals,step=object(),compute_orca_interaction_velocities=object());motion=NS(velocities={1:(1,0)});k=NS(origin=(0.,0.),env=e,boxes=boxes,motion=motion,humans=lambda:[dict(id=h.id,x=h.x,y=h.y,yaw=h.orientation) for g in groups for h in g.humans]);hooks=(e.step,e.compute_orca_interaction_velocities)
  receipt=initialize(k,spec);self.assertEqual(receipt['profile'],'crossing');self.assertEqual(k.initial,initial_humans(spec));self.assertEqual((e.step,e.compute_orca_interaction_velocities),hooks);self.assertEqual(motion.velocities,{})
  for g,row in zip(groups,spec['groups']):self.assertEqual([g.goal_x,g.goal_y],row['goal']);self.assertTrue(all(not h.position_history and h.speed==0 for h in g.humans))

 def test_optional_encounter_asset_must_be_pinned(self):
  with tempfile.TemporaryDirectory() as d:
   root=Path(d);run=root/'run';(run/'assets').mkdir(parents=True)
   for name,value in [('plan.json',{'encounter_profile':'head_on'}),('preflight.json',{}),('assets/mecanum_camera.sdf',{}),('assets/encounter.json',fixture('head_on'))]:dump(run/name,value)
   files=[p for p in run.rglob('*') if p.is_file()];freeze=dict(schema='dynamuhsim_capture_freeze/2',source_sha256={str(p.relative_to(root)):sha(p) for p in files});dump(run/'capture_freeze.json',freeze)
   with patch('dynamuhsim.runtime.capture.capture_sources',return_value=[]):
    verify_run(root,run);del freeze['source_sha256']['run/assets/encounter.json'];dump(run/'capture_freeze.json',freeze)
    with self.assertRaisesRegex(RuntimeError,'Encounter fixture missing'):verify_run(root,run)
if __name__=='__main__':unittest.main()
