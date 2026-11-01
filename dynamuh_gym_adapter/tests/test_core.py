import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import pytest
from core import (Scenario, SimulatorFault, SensorSnapshot, encode_scan,
                  require_fresh, goal_in_odom, clamp_action)


def s(epoch=1, t=.4):
    return SensorSnapshot(epoch=epoch, sim_time=t, scan_time=t-.08,
                          scan=np.full(360, 2., dtype=np.float32), range_min=.12,
                          range_max=3.5, odom_time=t-.02,
                          odom=np.array([0.,0.,0.,0.,0.,0.]), imu_time=t-.04)


def test_min_pool_and_mask():
    ranges=np.full(360, 2.,dtype=np.float32)
    ranges[:5]=np.nan
    distances,mask=encode_scan(ranges,.12,3.5)
    assert distances.shape==(72,) and mask.shape==(72,)
    assert mask[0]==0 and np.all(mask[1:]==1)


def test_epoch_and_staleness_rejected():
    with pytest.raises(SimulatorFault, match='epoch'):
        require_fresh(s(epoch=1), epoch=2)
    a=s(); a.scan_time=-10
    with pytest.raises(SimulatorFault, match='scan'):
        require_fresh(a, 1)


def test_goal_in_local_odom():
    target=goal_in_odom(Scenario(start=(-13.,0.,0.),goal=(-7.,0.)),
                        np.array([0.,0.,0.,0.,0.,0.]))
    np.testing.assert_allclose(target, [6.,0.])


def test_slew_rate_and_box():
    a=clamp_action(np.array([.6,.4,1.8]), np.zeros(3))
    np.testing.assert_allclose(a, [.3,.3,.3], rtol=1e-5)
    with pytest.raises(ValueError): clamp_action([np.nan,0.,0.], np.zeros(3))
