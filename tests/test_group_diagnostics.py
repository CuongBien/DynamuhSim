"""Recording forwards native solver state/return without replacing global RVO."""
import sys,unittest
from pathlib import Path
from types import SimpleNamespace as NS
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.simulation.group_diagnostics import GroupDiagnostics
class Simulator:
 def __init__(self,*args):self.values=[]
 def addAgent(self,position):self.values.append(dict(position=position,velocity=(0.,0.),radius=.36,preferred=(0.,0.)));return len(self.values)-1
 def setAgentRadius(self,i,radius):self.values[i]['radius']=radius
 def setAgentPrefVelocity(self,i,v):self.values[i]['preferred']=v
 def getAgentPosition(self,i):return self.values[i]['position']
 def getAgentVelocity(self,i):return self.values[i]['velocity']
 def getAgentRadius(self,i):return self.values[i]['radius']
 def getAgentPrefVelocity(self,i):return self.values[i]['preferred']
 def doStep(self):
  for v in self.values:v['velocity']=v['preferred']
rvo2=NS(PyRVOSimulator=Simulator)
class Env:
 def __init__(self):self.moving_interactions=[NS(goal_x=2.,goal_y=3.,humans=[NS(id=1),NS(id=2)])]
 def compute_orca_interaction_velocities(self):
  sim=rvo2.PyRVOSimulator();a=sim.addAgent((1.,2.));sim.setAgentRadius(a,1.44);sim.setAgentPrefVelocity(a,(.4,.2));sim.doStep();return [sim.getAgentVelocity(a)]
class GroupDiagnosticTests(unittest.TestCase):
 def test_native_return_and_globals_unchanged_and_raw_state_observable(self):
  e=Env();expected=e.compute_orca_interaction_velocities();original=Env.compute_orca_interaction_velocities
  d=GroupDiagnostics(NS(env=e));self.assertEqual(e.compute_orca_interaction_velocities(),expected)
  self.assertIs(Env.compute_orca_interaction_velocities,original);self.assertIs(original.__globals__['rvo2'].PyRVOSimulator,Simulator)
  self.assertEqual(d.last,[dict(members=[1,2],goal=[2.,3.],center=[1.,2.],envelope_radius_m=1.44,solver_input_velocity=[0.,0.],raw_preferred_velocity=[.4,.2],post_orca_velocity=[.4,.2])])
  e.moving_interactions[0].goal_x=4.;e.compute_orca_interaction_velocities();self.assertEqual(d.last[0]['goal'],[4.,3.])
if __name__=='__main__':unittest.main()
