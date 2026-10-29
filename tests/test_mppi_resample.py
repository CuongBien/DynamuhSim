"""The resampling experiment changes only one native controller parameter."""
import copy,json,sys,unittest
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.runtime.capture import validate_config
from dynamuhsim.simulation.capture import apply_nav2_treatment,config_changes,nav2_parameter_contract
class ResampleTests(unittest.TestCase):
 def test_only_noise_regeneration_changes_and_is_live_checked(self):
  base=yaml.safe_load((ROOT/'experiments/social_navigation/warehouse/nav2_warehouse.yaml').read_text());configs=[json.loads((ROOT/'configs/capture'/n).read_text()) for n in ['geometry_v6_mppi_horizon5.json','geometry_v7_mppi_resample.json']]
  applied=[apply_nav2_treatment(base,c['nav2'],navigation=c['navigation'],controller=c['controller']) for c in configs]
  self.assertEqual({x['path'] for x in config_changes(*applied)},{'/controller_server/ros__parameters/FollowPath/regenerate_noises'})
  c=validate_config(configs[1]);live=nav2_parameter_contract(c['nav2'],c['navigation'],controller=c['controller'])['/controller_server'];self.assertIs(live['FollowPath.regenerate_noises'],True);self.assertEqual(live['FollowPath.time_steps'],100)
  for v in [1,False,None]:
   other=copy.deepcopy(c);other['controller']['regenerate_noises']=v
   with self.assertRaises(ValueError):validate_config(other)
if __name__=='__main__':unittest.main()
