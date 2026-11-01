"""100-episode API/unit stress test, NOT real Gazebo acceptance."""
import sys
import types
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import pytest

# Small test-only spaces shim because ROS/Gazebo container may lack gymnasium.
# A real environment MUST use the real gymnasium package.
try:
    import gymnasium
except ImportError:
    gm=types.ModuleType('gymnasium')
    class Env:
        def reset(self, *, seed=None, options=None): self.np_random=np.random.default_rng(seed)
    class Box:
        def __init__(self, low, high, shape=None, dtype=np.float32):
            self.dtype=dtype
            self.low=np.broadcast_to(low,shape).astype(dtype) if shape else np.asarray(low,dtype=dtype)
            self.high=np.broadcast_to(high,shape).astype(dtype) if shape else np.asarray(high,dtype=dtype)
        def contains(self,x):
            a=np.asarray(x);return a.shape==self.low.shape and np.isfinite(a).all() and np.all(a>=self.low) and np.all(a<=self.high)
    class Dict:
        def __init__(self, items):self.items=items
        def contains(self,x):return set(x)==set(self.items) and all(v.contains(x[k]) for k,v in self.items.items())
    gm.Env=Env;gm.spaces=types.SimpleNamespace(Box=Box,Dict=Dict)
    sys.modules['gymnasium']=gm

from core import SensorSnapshot, SimulatorFault
from env import DynamuhGazeboEnv

class FakeBackend:
    def __init__(self): self.episode=None;self.time=0.;self.closed=0;self.x=0.
    def reset(self,scenario,epoch):
        self.episode=epoch;self.time=.4;self.x=0.
        return self.snap()
    def snap(self):
        return SensorSnapshot(self.episode,self.time,self.time-.05,
            np.full(360,2.,dtype=np.float32),.12,3.5,
            self.time-.01,np.array([self.x,0.,0.,0.,0.,0.]),self.time-.02)
    def advance(self,a,steps,epoch):
        if epoch!=self.episode:raise SimulatorFault('epoch mismatch')
        self.time+=.004*steps;self.x+=float(a[0])*.004*steps
        return self.snap()
    def close(self):self.closed+=1


def test_100_resets_and_3_steps_no_cross_epoch():
    backend=FakeBackend();env=DynamuhGazeboEnv(backend)
    try:
        for i in range(100):
            obs,info=env.reset(seed=i,options={'scenario':'empty'})
            assert info['epoch']==i+1
            assert env.observation_space.contains(obs)
            assert np.isclose(obs['goal'][0],.6)
            last=info['sim_time']
            for _ in range(3):
                obs,r,terminated,truncated,inf=env.step(np.zeros(3,dtype=np.float32))
                assert np.isfinite(r) and not terminated and not truncated
                assert inf['epoch']==i+1
                assert np.isclose(inf['sim_time']-last,.2)
                last=inf['sim_time']
    finally:
        env.close()
    assert backend.closed==1


def test_reject_stale_cross_epoch_and_terminal_step():
    class Bad(FakeBackend):
        def advance(self,a,steps,epoch):
            s=super().advance(a,steps,epoch)
            s.epoch-=1
            return s
    env=DynamuhGazeboEnv(Bad())
    env.reset(seed=2)
    with pytest.raises(SimulatorFault,match='epoch'):
        env.step(np.zeros(3))
    env.close()
    env=DynamuhGazeboEnv(FakeBackend(),max_sim_seconds=.19)
    env.reset(seed=1)
    obs,rew,term,trunc,info=env.step(np.zeros(3))
    assert trunc and not term
    with pytest.raises(RuntimeError):env.step(np.zeros(3))
    env.close()
