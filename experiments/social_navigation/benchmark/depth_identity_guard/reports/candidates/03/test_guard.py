#!/usr/bin/env python3
import unittest
import numpy as np
from guard_tracker import IdentityGuard,mutual_margin


def det(x,y=0,shape=None):return dict(xy=np.array([x,y]),shape=np.array(shape if shape is not None else [.8,.15,.08]),points=30,extent=np.array([.4,.4,.8]))

def mature(t,points):
    for i in range(5):out=t.update(i*100_000_000,[det(*p) for p in points],np.zeros(3))
    return out


class Tests(unittest.TestCase):
    def test_mutual_matching_does_not_force_losing_track(self):
        d=np.array([[.5,.7],[.01,1.]]);ok=np.array([[True,False],[True,False]])
        match,amb=mutual_margin(d,ok,.08);self.assertEqual(match,{0:1});self.assertEqual(amb,set())

    def test_equal_candidates_are_unknown(self):
        match,amb=mutual_margin(np.array([[.1,.12]]),np.ones((1,2),bool),.08)
        self.assertEqual(match,{});self.assertEqual(amb,{0,1})

    def test_two_tracks_compete_for_one_detection(self):
        match,amb=mutual_margin(np.array([[.1],[.12]]),np.ones((2,1),bool),.08)
        self.assertEqual(match,{});self.assertEqual(amb,{0})

    def test_large_gap_displacement_cannot_borrow_identity(self):
        t=IdentityGuard();before=mature(t,[(2.,0)])
        after=t.update(700_000_000,[det(2.,.76)],np.zeros(3))
        self.assertNotEqual(before[0]['track_id'],after[0]['track_id'])

    def test_joint_displacement_and_shape_jump_rejects_identity(self):
        t=IdentityGuard();before=mature(t,[(2.,0)])
        after=t.update(700_000_000,[det(2.,.70,[.2,.01,.005])],np.zeros(3))
        self.assertNotEqual(before[0]['track_id'],after[0]['track_id'])

    def test_partial_visibility_shape_change_alone_preserves_identity(self):
        t=IdentityGuard();before=mature(t,[(2.,0)])
        after=t.update(700_000_000,[det(2.03,0,[.2,.01,.005])],np.zeros(3))
        self.assertEqual(before[0]['track_id'],after[0]['track_id'])

    def test_nearby_ambiguity_retains_proposal_without_id(self):
        t=IdentityGuard();before=mature(t,[(2.,0),(2.3,0)])
        out=t.update(500_000_000,[det(2.15,0)],np.zeros(3))
        self.assertEqual(len(out),1);self.assertEqual(out[0]['track_id'],-1);self.assertFalse(out[0]['confirmed']);self.assertFalse(out[0]['velocity_valid'])
        # Old tracks remain internal; no imputed observations are emitted.
        self.assertTrue(all(a['track_id'] in t.tracks for a in before))
        out=t.update(600_000_000,[det(2.,0),det(2.3,0)],np.zeros(3))
        self.assertEqual([a['track_id'] for a in out],[a['track_id'] for a in before])

    def test_missing_has_no_ghost_observations(self):
        t=IdentityGuard();mature(t,[(2.,0)]);self.assertEqual(t.update(800_000_000,[],np.zeros(3)),[])

    def test_dormant_clear_identity_can_be_reused(self):
        t=IdentityGuard();before=mature(t,[(2.,0)]);after=t.update(1_100_000_000,[det(2.05,0)],np.zeros(3))
        self.assertEqual(before[0]['track_id'],after[0]['track_id']);self.assertTrue(after[0]['reacquired_after_gap'])

    def test_expired_identity_not_reused(self):
        t=IdentityGuard();before=mature(t,[(2.,0)]);after=t.update(2_000_000_000,[det(2.,0)],np.zeros(3));self.assertNotEqual(before[0]['track_id'],after[0]['track_id'])

    def test_ego_motion_compensation(self):
        t=IdentityGuard();t.update(0,[det(2.)],np.zeros(3));after=t.update(100_000_000,[det(1.8)],np.array([.2,0,0]))
        np.testing.assert_allclose(after[0]['velocity_odom'],[0,0],atol=1e-12)

    def test_clock_reset_rejected(self):
        t=IdentityGuard();t.update(100,[],np.zeros(3))
        with self.assertRaises(ValueError):t.update(99,[],np.zeros(3))

if __name__=='__main__':unittest.main(verbosity=2)
