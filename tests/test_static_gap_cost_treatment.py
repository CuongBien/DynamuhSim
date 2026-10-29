import copy,json,sys,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import yaml
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.runtime.capture import validate_config,capture_live_parameters
from dynamuhsim.simulation.capture import apply_nav2_treatment,config_changes,nav2_parameter_contract
class StaticGapCostTests(unittest.TestCase):
 def config(self):return json.loads((ROOT/'configs/capture/geometry_v5_mppi_native_cost.json').read_text())
 def test_only_two_declared_critic_parameters_change(self):
  old=json.loads((ROOT/'configs/capture/geometry_v4_inflation045.json').read_text());new=self.config();self.assertEqual(validate_config(new),new)
  base=yaml.safe_load((ROOT/'experiments/social_navigation/warehouse/nav2_warehouse.yaml').read_text());a=apply_nav2_treatment(base,old['nav2'],navigation=old['navigation']);b=apply_nav2_treatment(base,new['nav2'],navigation=new['navigation'],controller=new['controller']);changes=config_changes(a,b)
  self.assertEqual({c['path'] for c in changes},{'/controller_server/ros__parameters/FollowPath/CostCritic/'+k for k in ['cost_weight','near_collision_cost']})
  self.assertEqual(b['controller_server']['ros__parameters']['FollowPath']['CostCritic']['collision_cost'],2000000.);self.assertFalse(b['controller_server']['ros__parameters']['FollowPath']['CostCritic']['consider_footprint'])
  for name in ['global_costmap','local_costmap']:self.assertEqual(a[name],b[name])
 def test_old_schema_cannot_silently_acquire_controller_tuning(self):
  config=self.config();config['schema']='dynamuhsim_capture_config/4'
  with self.assertRaises(ValueError):validate_config(config)
  for key,value in [('cost_weight',0.),('near_collision_cost',254)]:
   config=self.config();config['controller']['cost_critic'][key]=value
   with self.assertRaises(ValueError):validate_config(config)
 def test_live_integer_near_collision_parameter_is_checked_as_integer(self):
  config=self.config();expected=nav2_parameter_contract(config['nav2'],config['navigation'],controller=config['controller'])
  class Client:
   def __init__(self,node,remote):self.remote=remote
   def services_are_ready(self):return True
   def get_parameters(self,names):
    def value(w):return NS(bool_value=w if isinstance(w,bool) else False,string_value=w if isinstance(w,str) else '',integer_value=w if isinstance(w,int) and not isinstance(w,bool) else 0,double_value=w if isinstance(w,float) else 0.)
    values=[value(expected[self.remote][name]) for name in names];return NS(done=lambda:True,result=lambda:NS(values=values))
  with patch.dict(sys.modules,{'rclpy.parameter_client':NS(AsyncParameterClient=Client)}):
   observed=capture_live_parameters(None,lambda predicate,seconds:predicate(),config['nav2'],config['navigation'],controller=config['controller'])
  self.assertEqual(observed,expected);self.assertEqual(observed['/controller_server']['FollowPath.CostCritic.near_collision_cost'],253)
if __name__=='__main__':unittest.main()
