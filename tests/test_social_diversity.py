"""Context/persistence regressions for the opt-in pre-safety preference layer."""
import math,random,sys,unittest
from pathlib import Path
from types import SimpleNamespace as NS
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.simulation.social_diversity import SocialDiversity,traits,encounter
from dynamuhsim.simulation.human_motion_profiles import profile_config,validate_profile


def fixture(positions=((0.,0.),(1.2,-.7)),velocities=None,group=False):
    hs=[NS(id=i+1,x=x,y=y,goal_x=8.,goal_y=0.,width=.72,type='dynamic') for i,(x,y) in enumerate(positions)]
    gs=[NS(humans=hs,x=sum(h.x for h in hs)/len(hs),y=sum(h.y for h in hs)/len(hs),goal_x=8.,goal_y=0.)] if group else []
    e=NS(static_humans=[],dynamic_humans=[] if group else hs,moving_interactions=gs,static_interactions=[],h_l_interactions=[],TIMESTEP=.1,HUMAN_DIAMETER=.72,MAX_ADVANCE_HUMAN=.8,robot=NS(radius=.25))
    motion=NS(velocities=velocities or {},tick=1,robot_start=(-20.,0.),robot_velocity=(0.,0.))
    k=NS(env=e,motion=motion);c=SocialDiversity(k,45)
    return c,hs


def step(c,hs,t):
    c.kernel.motion.tick=round(t/.1)+1
    return c.preferences(hs,[(h.x,h.y) for h in hs],[(0.,0.)]*len(hs),set())


class SocialTests(unittest.TestCase):
    def test_seed_traits_do_not_consume_global_rng(self):
        random.seed(9);state=random.getstate();a=traits(45,1);self.assertEqual(random.getstate(),state);self.assertEqual(a,traits(45,1));self.assertNotEqual(a,traits(45,2));self.assertNotEqual(a,traits(46,1));self.assertLessEqual(a['acceleration_mps2'],.8)

    def test_passing_side_and_partner_do_not_chatter(self):
        c,hs=fixture(positions=((0.,0.),(1.,0.)),velocities={2:(-.3,0.)})
        step(c,hs,0.);self.assertEqual(c.states[1]['state'],'passing')
        for t,y in [(.1,.01),(.2,-.01),(.3,.02)]:
            hs[1].y=y;step(c,hs,t);self.assertEqual(c.states[1]['partner'],2);self.assertEqual(c.states[1]['side'],1)
        hs[1].x=10.;step(c,hs,1.5);step(c,hs,2.2);self.assertEqual(c.states[1]['state'],'resuming');step(c,hs,4.);self.assertEqual(c.states[1]['state'],'walking')

    def test_yield_wait_has_bounded_exit_and_cooldown(self):
        c,hs=fixture(positions=((0.,0.),(10.,0.)))
        c.traits[1]['yielding_tendency']=1.;m=c.kernel.motion;m.robot_start=(.5,-.5);m.robot_velocity=(0.,.5)
        step(c,hs,0.);self.assertEqual(c.states[1]['state'],'waiting');self.assertTrue(any(e['event']=='yielding_start' for e in c.events))
        step(c,hs,.9);self.assertEqual(c.states[1]['state'],'passing');self.assertGreater(c.states[1]['cooldown'],.9)

    def test_stopped_robot_is_passed_instead_of_always_yielding(self):
        c,hs=fixture();c.kernel.motion.robot_start=(1.,0.);c.traits[1]['yielding_tendency']=1.;step(c,hs,0.);self.assertEqual(c.states[1]['state'],'passing')

    def test_overtaking_is_context_triggered(self):
        c,hs=fixture(positions=((0.,0.),(1.,0.)),velocities={2:(.12,0.)});step(c,hs,0.);self.assertEqual(c.states[1]['state'],'overtaking')

    def test_regrouping_has_hysteresis_and_releases(self):
        c,hs=fixture(positions=((-1.4,0.),(1.4,0.)),group=True);v=step(c,hs,0.);self.assertEqual(c.states[1]['state'],'regrouping');self.assertGreater(v[0][0],0.)
        hs[0].x=-1.;hs[1].x=1.;step(c,hs,.1);self.assertEqual(c.states[1]['state'],'regrouping')
        hs[0].x=-.8;hs[1].x=.8;step(c,hs,.2);self.assertEqual(c.states[1]['state'],'walking')

    def test_stationary_conversation_has_zero_preference(self):
        c,hs=fixture();out=c.preferences(hs,[(h.x,h.y) for h in hs],[(.8,0.)]*2,{1,2});self.assertEqual(out,[(0.,0.)]*2);self.assertEqual(c.states[1]['state'],'conversation')

    def test_goal_intention_and_actual_group_envelope(self):
        c,hs=fixture(positions=((-1.,0.),(1.,0.)),group=True);v=step(c,hs,0.);self.assertTrue(all(x[0]>0 for x in v));self.assertAlmostEqual(c.group_detail[0]['envelope_m'],1.36);self.assertTrue(all(math.hypot(*x)<=c.traits[h.id]['preferred_speed_mps']+1e-12 for h,x in zip(hs,v)))

    def test_reversed_goal_turns_at_rest_inside_bounded_heading_contract(self):
        from dynamuhsim.analysis.evidence import module_from_path
        nav=ROOT/'experiments/social_navigation/navigation'
        geometry=module_from_path(nav/'socnavgym_warehouse_lifecycle/geometry.py','_social_turn_geometry')
        projection=module_from_path(nav/'socnavgym_response_fix/projection.py','_social_turn_projection',{'geometry':geometry})
        contract=module_from_path(nav/'socnavgym_response_fix/motion_contract.py','_social_turn_contract')
        class RVO:
            def __init__(self,*args):self.pref=[]
            def addAgent(self,*args):self.pref.append((0.,0.));return len(self.pref)-1
            def setAgentPrefVelocity(self,i,v):self.pref[i]=v
            def getAgentVelocity(self,i):return self.pref[i]
            def __getattr__(self,name):return lambda *args:None
        h=NS(id=1,x=0.,y=0.,orientation=0.,width=.72,type='dynamic',position_history=[])
        e=NS(TIMESTEP=.1,static_humans=[],static_interactions=[],moving_interactions=[],walls=[],MAX_ADVANCE_HUMAN=.8,MAX_ADVANCE_ROBOT=.8,MAX_ROTATION_HUMAN=1.57,orca_neighborDist=10.,orca_timeHorizon=5.,orca_timeHorizonObst=5.,HUMAN_DIAMETER=.72,robot=NS(radius=.25))
        continuous=NS(ContinuousGroups=object,people=lambda env:[h],angle_delta=lambda a,b:math.atan2(math.sin(a-b),math.cos(a-b)))
        mod=module_from_path(nav/'socnavgym_response_fix/responsive_motion.py','_social_turn_motion',{'rvo2':NS(PyRVOSimulator=RVO),'continuous_groups':continuous,'projection':projection,'motion_contract':contract})
        m=mod.ResponsiveMotion.__new__(mod.ResponsiveMotion);m.env=e;m.original={'handle_collision_and_update':lambda:None};m.start={1:(0.,0.,0.)};m.velocities={};m.robot_start=(-10.,-10.);m.robot_velocity=(0.,0.);m.boxes=[dict(x=10.,y=10.,sx=1.,sy=1.)];m.steps=[];m.epoch=0;m.tick=1;m.motion_limits=contract.DEFAULT_LIMITS
        m.behavior_controller=NS(traits={1:traits(45,1)},preferences=lambda *args:[(-.5,0.)]);m.integrate()
        self.assertAlmostEqual(h.x,0.);self.assertAlmostEqual(h.y,0.);self.assertGreater(h.orientation,.1);self.assertLessEqual(h.orientation,.155);self.assertAlmostEqual(m.steps[-1]['robot_projection_radius_m'],.37)

    def test_declared_profile_and_diverging_traffic(self):
        self.assertEqual(validate_profile(profile_config('social_diversity_v1'))['profile'],'social_diversity_v1')
        self.assertFalse(encounter((0,0),(-.4,0),(1,0),(.4,0),1.,2.)[0])

if __name__=='__main__':unittest.main()
