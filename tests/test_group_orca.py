"""Group geometry/velocity correction is explicit and preserves native branches."""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
from dynamuhsim.simulation.human_groups import group_state, GroupORCA, load_profile, PROFILE
from dynamuhsim.runtime.capture import verify_run, dump, sha

class FakeRVO:
    def __init__(self, *args): self.agents = []; self.obstacles = []
    def addAgent(self, pos): self.agents.append(dict(pos=pos)); return len(self.agents)-1
    def setAgentRadius(self, i, v): self.agents[i]["radius"] = v
    def setAgentNeighborDist(self, i, v): self.agents[i]["neighbor"] = v
    def setAgentVelocity(self, i, v): self.agents[i]["velocity"] = v
    def setAgentPrefVelocity(self, i, v): self.agents[i]["preferred"] = v
    def addObstacle(self, v): self.obstacles.append(v)
    def processObstacles(self): pass
    def doStep(self): pass
    def getAgentVelocity(self, i): return self.agents[i]["preferred"]

class GroupTests(unittest.TestCase):
    def test_envelope_contains_members_and_uses_measured_velocity(self):
        group=NS(x=0.,y=0.,humans=[NS(id=1,x=.72,y=0.,width=.72),NS(id=2,x=-.72,y=0.,width=.72)])
        radius, velocity=group_state(group,{1:(.4,.2),2:(.2,0.)})
        self.assertAlmostEqual(radius,1.08)
        np.testing.assert_allclose(velocity,(.3,.1))
        group.humans[0].x=1.2
        self.assertAlmostEqual(group_state(group,{})[0],1.56)
        with self.assertRaises(ValueError):group_state(NS(humans=[]),{})

    def test_rvo_receives_new_state_and_retains_individual_and_obstacle_branches(self):
        human=NS(id=1,x=.72,y=0.,width=.72)
        group=NS(x=0.,y=0.,humans=[human],goal_x=2.,goal_y=0.)
        solo=NS(x=3.,y=4.,position_history=[(1.,2.,0.,0.)]*3,speed=.5,goal_x=5.,goal_y=4.)
        e=NS(TIMESTEP=.1,orca_neighborDist=5.,total_humans=2,orca_timeHorizon=5.,orca_timeHorizonObst=5.,HUMAN_DIAMETER=.72,orca_maxSpeed=.8,MAX_ADVANCE_HUMAN=.8,
             static_humans=[],dynamic_humans=[solo],tables=[],chairs=[],laptops=[],plants=[],walls=['wall'],static_interactions=[NS(name='human-human-interaction',type='stationary')],h_l_interactions=[NS(name='human-laptop-interaction',human='laptop-human')],moving_interactions=[group],get_obstacle_corners=lambda x:x)
        sim=FakeRVO();adapter=GroupORCA(NS(env=e,motion=NS(velocities={1:(.2,.1)})),lambda *args:sim)
        adapter.velocities()
        self.assertAlmostEqual(sim.agents[1]['radius'],1.08)
        self.assertEqual(sim.agents[1]['velocity'],(.2,.1))
        self.assertAlmostEqual(sim.agents[1]['neighbor'],2.16)
        np.testing.assert_allclose(sim.agents[0]['pos'],(1.05,2.))
        self.assertEqual(len(sim.obstacles),3)
        self.assertEqual(adapter.last[0]['members'],[1])

    def test_absent_profile_leaves_legacy_kernel_untouched(self):
        k=NS(env=NS(compute_orca_interaction_velocities=object()))
        before=k.env.compute_orca_interaction_velocities
        with tempfile.TemporaryDirectory() as d:
            self.assertIsNone(load_profile(k,d))
            self.assertIs(k.env.compute_orca_interaction_velocities,before)
            (Path(d)/'human_motion.json').write_text('{"profile":"unknown"}')
            with self.assertRaises(ValueError):load_profile(k,d)

    def test_profile_asset_must_be_frozen_and_match_plan(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);run=root/'run';(run/'assets').mkdir(parents=True)
            dump(run/'plan.json',{'human_motion_profile':'group_orca_v1'})
            dump(run/'preflight.json',{});dump(run/'assets/mecanum_camera.sdf',{})
            dump(run/'assets/human_motion.json',PROFILE)
            files=[p for p in run.rglob('*') if p.is_file()]
            freeze={'schema':'dynamuhsim_capture_freeze/2','source_sha256':{str(p.relative_to(root)):sha(p) for p in files}}
            dump(run/'capture_freeze.json',freeze)
            with patch('dynamuhsim.runtime.capture.capture_sources',return_value=[]):
                verify_run(root,run)
                del freeze['source_sha256']['run/assets/human_motion.json'];dump(run/'capture_freeze.json',freeze)
                with self.assertRaisesRegex(RuntimeError,'profile missing'):verify_run(root,run)

if __name__ == '__main__': unittest.main()
