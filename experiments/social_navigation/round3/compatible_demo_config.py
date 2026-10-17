"""External config for unmodified upstream test.py, using its native CV-MPC policy.
Provides missing concrete class exports and avoids unavailable RGL weights/SGAN path.
"""
import copy
from pathlib import Path
import yaml
from crowd_nav.configs.icra_benchmark.config import (
    BaseEnvConfig, BasePolicyConfig, BaseTrainConfig, BaseExperimentsConfig)

class ExperimentsConfig(BaseExperimentsConfig):
    def __init__(self,debug=False):
        self.exp=copy.deepcopy(BaseExperimentsConfig.exp)
        # Upstream visualization selects experiment 0, condition 2, episode 1.
        for key in ['num_orca','num_sf','num_static','num_linear','dx','dy','scenarios','randomize_attributes']:
            setattr(self.exp,key,[getattr(self.exp,key)[0][:3]])

class EnvConfig(BaseEnvConfig):
    def __init__(self,debug=False):
        super().__init__(debug)
        self.MPC=copy.deepcopy(BaseEnvConfig.MPC)
        self.MPC.model='cv'
        path=Path('/opt/ComplexityNav/crowd_nav/configs/params/cv.yaml')
        self.MPC.mpc=yaml.safe_load(path.read_text())
        self.MPC.save_path='/work/reports/cv_mpc_logs'
        self.MPC.exp_name='compatible_native_demo'

class PolicyConfig(BasePolicyConfig):
    def __init__(self,debug=False):
        super().__init__(debug)
        self.name='vecmpc'

class TrainConfig(BaseTrainConfig):
    pass
