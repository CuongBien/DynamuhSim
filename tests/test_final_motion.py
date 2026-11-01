"""Interval motion reachability is solved together with swept physical clearance."""
import math,sys,unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.analysis.evidence import module_from_path
NAV=ROOT/'experiments/social_navigation/navigation'
g=module_from_path(NAV/'socnavgym_warehouse_lifecycle/geometry.py','_motion_geometry')
p=module_from_path(NAV/'socnavgym_response_fix/projection.py','_motion_projection',{'geometry':g})
c=module_from_path(NAV/'socnavgym_response_fix/motion_contract.py','_motion_contract')

class FinalMotionTests(unittest.TestCase):
 def project(self,desired,previous=(0.,0.),yaw=0.,robot=(-10.,-10.),robot_delta=(0.,0.),starts=None,boxes=None):
  pref=c.preferred_velocity(desired,yaw,1.57,.1)
  budgets=c.velocity_change_limits([previous],[pref],c.DEFAULT_LIMITS)
  kw=dict(starts=starts or [(0.,0.)],deltas=[tuple(v*.1 for v in pref)],radii=[.36],robot=robot,robot_delta=robot_delta,robot_radius=.25,boxes=boxes or [dict(x=10.,y=10.,sx=1.,sy=1.)],dt=.1,max_speed=.8,fixed=set())
  out,a=p.project(**kw,previous_velocities=[previous],acceleration_limits=budgets)
  v=np.array(out[0])/.1;y=c.final_heading(yaw,v,0.,1.57,.1)
  self.assertGreaterEqual(min(a['predicted_clearance'].values()),.002-1e-8)
  self.assertLessEqual(np.linalg.norm(v),.8+1e-7)
  self.assertLessEqual(abs(math.atan2(math.sin(y-yaw),math.cos(y-yaw))),.157+1e-8)
  if a['motion_constraints_satisfied']:self.assertLessEqual(np.linalg.norm(v-np.array(previous))/.1,budgets[0]+1e-6)
  return v,y,a

 def test_unconfigured_responsive_preserves_legacy_yaw_and_speed_limits(self):
  from types import SimpleNamespace as NS
  class RVO:
   def __init__(self,*args):self.pref=[]
   def addAgent(self,*args):self.pref.append((0.,0.));return len(self.pref)-1
   def setAgentPrefVelocity(self,i,v):self.pref[i]=v
   def getAgentVelocity(self,i):return self.pref[i]
   def __getattr__(self,name):return lambda *args:None
  h=NS(id=1,x=0.,y=0.,orientation=0.,width=.72,type='dynamic',position_history=[])
  env=NS(TIMESTEP=.1,static_humans=[],static_interactions=[],moving_interactions=[],walls=[],MAX_ADVANCE_HUMAN=.8,MAX_ADVANCE_ROBOT=.8,MAX_ROTATION_HUMAN=1.57,orca_neighborDist=10.,orca_timeHorizon=5.,orca_timeHorizonObst=5.,HUMAN_DIAMETER=.72,robot=NS(radius=.25))
  continuous=NS(ContinuousGroups=object,people=lambda e:[h],angle_delta=lambda a,b:math.atan2(math.sin(a-b),math.cos(a-b)))
  module=module_from_path(NAV/'socnavgym_response_fix/responsive_motion.py','_motion_responsive_compat',{'rvo2':NS(PyRVOSimulator=RVO),'continuous_groups':continuous,'projection':p,'motion_contract':c})
  motion=module.ResponsiveMotion.__new__(module.ResponsiveMotion)
  def native():h.x=.08;h.orientation=math.pi/2
  motion.env=env;motion.original={'handle_collision_and_update':native};motion.start={1:(0.,0.,0.)};motion.velocities={};motion.robot_start=(-10.,-10.);motion.robot_velocity=(0.,0.);motion.boxes=[dict(x=10.,y=10.,sx=1.,sy=1.)];motion.steps=[];motion.epoch=0;motion.tick=1
  motion.integrate()
  self.assertAlmostEqual(h.orientation,.157);self.assertAlmostEqual(h.x,.08);self.assertAlmostEqual(h.vx,.8)
  self.assertNotIn('motion_limit_relaxed',motion.steps[-1]['projection'])

 def test_straight_acceleration_from_rest_and_braking_stop(self):
  v=(0.,0.);history=[]
  for _ in range(10):
   v,y,a=self.project((.8,0.),v);history.append(float(v[0]));self.assertTrue(a['motion_constraints_satisfied'])
  np.testing.assert_allclose(history,np.arange(1,11)*.08,atol=1e-7)
  for _ in range(5):v,y,a=self.project((0.,0.),v);self.assertTrue(a['motion_constraints_satisfied'])
  np.testing.assert_allclose(v,(0.,0.),atol=1e-7)

 def test_90_degree_and_reversal_preferences_brake_and_turn(self):
  for desired in [(0.,.8),(-.8,0.)]:
   v,y,a=self.project(desired,(.8,0.));self.assertTrue(a['motion_constraints_satisfied'])
   self.assertLess(np.linalg.norm(v),.8);self.assertLessEqual(np.linalg.norm(v-(.8,0.))/.1,1.6+1e-6)
   self.assertAlmostEqual(y,math.atan2(v[1],v[0]),places=7)

 def test_lateral_safety_correction_and_robot_braking_direction_change(self):
  # Approaching robot forces a transverse correction; hard delta-v may be infeasible.
  v,y,a=self.project((0.,0.),robot=(.65,0.),robot_delta=(-.08,0.))
  self.assertLess(v[0],0.);self.assertTrue(a['motion_limit_relaxed']);self.assertFalse(a['motion_constraints_satisfied'])
  self.assertIsNotNone(a['limiting_condition'])
  self.project((0.,0.),v,y,robot=(.57,0.),robot_delta=(0.,.03),starts=[tuple(v*.1)])

 def test_lateral_correction_keeps_final_yaw_rate_and_reports_disagreement(self):
  v,y,a=self.project((.8,0.),previous=(.8,0.),robot=(0.,-.65),robot_delta=(0.,.08))
  self.assertGreater(v[1],0.)
  self.assertTrue(a['motion_limit_relaxed'])
  # A safety sidestep is allowed even when body heading cannot follow instantly.
  self.assertGreater(abs(math.atan2(v[1],v[0])-y),0.)

 def test_moving_person_becoming_static_reports_emergency_braking(self):
  out,a=p.project([(0.,0.)],[(0.,0.)],[.36],(-10.,-10.),(0.,0.),.25,[dict(x=10.,y=10.,sx=1.,sy=1.)],.1,.8,{0},previous_velocities=[(.8,0.)],acceleration_limits=[1.6])
  np.testing.assert_allclose(out,[(0.,0.)],atol=1e-7)
  self.assertTrue(a['motion_limit_relaxed']);self.assertFalse(a['motion_constraints_satisfied'])

 def test_near_zero_and_static_facing(self):
  v,y,a=self.project((1e-12,0.),yaw=1.2);np.testing.assert_allclose(v,(0.,0.));self.assertEqual(y,1.2)
  self.assertAlmostEqual(c.final_heading(0.,(0.,0.),1.,1.57,.1,stationary=True),.157)

 def test_narrow_clearance_and_pair_sweeps_remain_safe(self):
  boxes=[dict(x=0.,y=.375,sx=4.,sy=.02),dict(x=0.,y=-.375,sx=4.,sy=.02)]
  v,y,a=self.project((.8,.2),boxes=boxes);self.assertLessEqual(abs(v[1]),.03+1e-6)
  starts=[(-.4,0.),(.4,0.)];deltas=[(.08,0.),(-.08,0.)]
  out,a=p.project(starts,deltas,[.36,.36],(-10.,-10.),(0.,0.),.25,[dict(x=10.,y=10.,sx=1.,sy=1.)],.1,.8,set(),previous_velocities=[(.8,0.),(-.8,0.)],acceleration_limits=[1.6,1.6])
  gaps=g.clearance(starts,out,[.36,.36],(-10.,-10.),(0.,0.),.25,[dict(x=10.,y=10.,sx=1.,sy=1.)])
  self.assertGreaterEqual(gaps['pair'],.002-1e-8)
  self.assertTrue(a['motion_limit_relaxed'])

 def test_impossible_safety_remains_terminal_and_legacy_api_works(self):
  with self.assertRaisesRegex(RuntimeError,'No validated'):
   self.project((0.,0.),robot=(.62,0.),robot_delta=(-.3,0.))
  out,a=p.project([(0.,0.)],[(.08,0.)],[.36],(-10.,-10.),(0.,0.),.25,[dict(x=10.,y=10.,sx=1.,sy=1.)],.1,.8,set())
  np.testing.assert_allclose(out,[(.08,0.)]);self.assertNotIn('motion_limit_relaxed',a)
  with self.assertRaises(ValueError):c.validate_limits(dict(acceleration_mps2=0.,deceleration_mps2=1.))

if __name__=='__main__':unittest.main()
