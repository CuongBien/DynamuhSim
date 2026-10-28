import json
from pathlib import Path
import unittest
import numpy as np
from dynamuhsim.datasets.normalization import causal_indices, interpolate_labels, depth_points, observed_bev, validate_clock, group_membership

class ContractTests(unittest.TestCase):
    def test_late_delivery_cannot_enter_past(self):
        # A frame with an old header but arriving late must stay unavailable.
        i,v=causal_indices([100,150,190],[100,500,190],[200],1000)
        self.assertEqual(i.tolist(),[2]);self.assertTrue(v.all())
        # Poison future samples; past selection must be unchanged.
        j,w=causal_indices([100,150,190,10000],[100,500,190,10000],[200],1000)
        np.testing.assert_array_equal(i,j);np.testing.assert_array_equal(v,w)

    def test_stale_and_missing_are_masked(self):
        _,v=causal_indices([100,200],[100,200],[50,100,400],100)
        self.assertEqual(v.tolist(),[False,True,False])

    def test_future_labels_interpolate_without_extrapolation(self):
        xy,v=interpolate_labels([0,100],np.array([[0.,0.],[2.,4.]]),[-1,50,101],100)
        np.testing.assert_allclose(xy[1],[1,2]);self.assertEqual(v.tolist(),[False,True,False])
        _,v=interpolate_labels([0,1000],np.zeros((2,6,2)),[500],100)
        self.assertFalse(v[0])

    def test_optical_forward_and_left_axes(self):
        t=np.array([[0,0,1,.12],[-1,0,0,0],[0,-1,0,.08],[0,0,0,1.]])
        p=depth_points(np.full((1,3),2.),[1,0,1,0,1,0,0,0,1],t,stride=1)
        np.testing.assert_allclose(p,[[2.12,2,.08],[2.12,0,.08],[2.12,-2,.08]])

    def test_unknown_depth_does_not_make_free_space(self):
        t=np.array([[0,0,1,.12],[-1,0,0,0],[0,-1,0,.08],[0,0,0,1.]])
        d=np.full((8,8),np.nan)
        self.assertTrue((observed_bev(d,[2,0,4,0,2,4,0,0,1],t)==-1).all())
        d[:]=np.inf
        self.assertTrue((observed_bev(d,[2,0,4,0,2,4,0,0,1],t)==-1).all())
        d[4,4]=2
        b=observed_bev(d,[2,0,4,0,2,4,0,0,1],t)
        self.assertEqual(b[21,32],1)
        self.assertEqual(b[30,32],-1) # no free-space carving behind endpoint
        self.assertEqual(b[10,32],0)

    def test_clock_reset_rejected_instead_of_merging_epochs(self):
        validate_clock([0,100,100,200])
        with self.assertRaises(ValueError):validate_clock([0,100,0,100])

    def test_native_group_modes_and_ungrouped_ids(self):
        ids,modes=group_membership([dict(id=7,type='moving',members=[1,2]),
                                   dict(id=8,type='standing',members=[3])],[1,2,3,4])
        self.assertEqual(ids,[7,7,8,-1]);self.assertEqual(modes,[1,1,2,0])
        with self.assertRaises(ValueError):
            group_membership([dict(id=7,type='moving',members=[1]),
                              dict(id=8,type='standing',members=[1])],[1])

    def test_persona_unknown_is_not_a_fake_h_class(self):
        c=json.loads((Path(__file__).resolve().parents[1]/'python/dynamuhsim/datasets/normalization_contract.json').read_text())
        self.assertIsNone(c['persona_unknown']['persona'])
        self.assertFalse(c['persona_unknown']['persona_valid'])
        self.assertEqual(c['action_components'],['vx','vy','wz'])
        self.assertEqual(len(c['history_offsets_ns']),20)
        self.assertEqual(len(c['future_offsets_ns']),30)
        self.assertEqual(c['future_action_offsets_ns'][0],0)
        np.testing.assert_array_equal(np.array(c['future_action_offsets_ns'])+100000000,c['future_offsets_ns'])
        self.assertTrue(all(t<=0 for t in c['history_offsets_ns']))
        self.assertTrue(all(t>0 for t in c['future_offsets_ns']))

if __name__=='__main__':unittest.main(verbosity=2)
