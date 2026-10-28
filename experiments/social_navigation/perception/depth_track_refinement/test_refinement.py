#!/usr/bin/env python3
import unittest
import numpy as np
from refinement import enrich,DormantTracker
from snapshots.sensor import Tracker,odom_at


def det(x,y,shape=None):
    return dict(xy=np.array([x,y]),points=30,extent=np.array([.4,.4,.8]),shape=np.array(shape if shape is not None else [.8,.12,.08]))


def mature(tracker,positions):
    for i in range(5):out=tracker.update(i*100_000_000,[det(*p) for p in positions],np.zeros(3))
    return out


class Tests(unittest.TestCase):
    def test_tall_column_rejected_short_leg_kept(self):
        column=np.array([[2.,y,z] for y in np.linspace(-.1,.1,8) for z in np.linspace(.05,1.75,20)])
        leg=np.array([[1+x,.6+y,z] for x in [0,.02,.04] for y in [0,.03,.06] for z in np.linspace(.05,.8,15)])
        kept,rejected=enrich(np.vstack([column,leg]));self.assertEqual(len(kept),1);self.assertEqual(len(rejected),1)
        self.assertLess(kept[0]['shape'][0],1.55)

    def test_proxy_like_human_is_rejected_showing_domain_limit(self):
        # A tall cylindrical human proxy is geometrically indistinguishable from a post.
        p=np.array([[2+.36*np.cos(a),.36*np.sin(a),z] for a in np.linspace(2,4.2,30) for z in np.linspace(.05,1.7,20)])
        kept,rejected=enrich(p);self.assertEqual(kept,[]);self.assertEqual(len(rejected),1)

    def test_tall_nonuniform_humanlike_geometry_retained(self):
        p=np.array([[2+.16*np.sin(5*z)+x,y,z] for z in np.linspace(.05,1.75,30) for x in [-.08,.08] for y in [-.12,.12]])
        kept,rejected=enrich(p);self.assertEqual(len(kept),1);self.assertEqual(rejected,[])

    def test_prior_rotates_with_geometry(self):
        p=np.array([[2+x,.3+y,z] for x in [-.05,0,.05] for y in [-.05,0,.05] for z in np.linspace(.05,1.75,20)])
        a=enrich(p);angle=.4;c,s=np.cos(angle),np.sin(angle);p[:,:2]=p[:,:2]@np.array([[c,s],[-s,c]])
        q=enrich(p);self.assertEqual((len(a[0]),len(a[1])),(len(q[0]),len(q[1])))

    def test_dormant_reacquisition_after_700ms(self):
        tracker=DormantTracker();before=mature(tracker,[(2.,0)])
        self.assertEqual(tracker.update(800_000_000,[],np.zeros(3)),[])
        after=tracker.update(1_100_000_000,[det(2.05,0)],np.zeros(3))
        self.assertEqual(before[0]['track_id'],after[0]['track_id']);self.assertTrue(after[0]['reacquired_after_gap'])

    def test_dormant_ambiguous_identity_is_not_forced(self):
        tracker=DormantTracker();before=mature(tracker,[(2.,0),(2.3,0)]);ids={t['track_id'] for t in before}
        after=tracker.update(1_100_000_000,[det(2.15,0)],np.zeros(3));self.assertNotIn(after[0]['track_id'],ids)

    def test_dormant_incompatible_shape_is_not_reused(self):
        tracker=DormantTracker();before=mature(tracker,[(2.,0)])
        after=tracker.update(1_100_000_000,[det(2.,0,[3.,.8,.6])],np.zeros(3));self.assertNotEqual(before[0]['track_id'],after[0]['track_id'])

    def test_long_absence_does_not_reuse_identity(self):
        tracker=DormantTracker();before=mature(tracker,[(2.,0)])
        after=tracker.update(2_000_000_000,[det(2.,0)],np.zeros(3));self.assertNotEqual(before[0]['track_id'],after[0]['track_id'])

    def test_unconfirmed_candidate_is_not_dormant_identity(self):
        tracker=DormantTracker();before=tracker.update(0,[det(2.,0)],np.zeros(3))
        after=tracker.update(700_000_000,[det(2.,0)],np.zeros(3));self.assertNotEqual(before[0]['track_id'],after[0]['track_id'])

    def test_active_path_matches_legacy(self):
        a=Tracker();b=DormantTracker()
        for i in range(12):
            d=[det(2+i*.02,.2)];x=a.update(i*100_000_000,d,np.zeros(3));y=b.update(i*100_000_000,d,np.zeros(3))
            self.assertEqual(x[0]['track_id'],y[0]['track_id']);np.testing.assert_array_equal(x[0]['velocity_odom'],y[0]['velocity_odom'])

    def test_missing_stays_missing_and_clock_reset_rejected(self):
        tracker=DormantTracker();mature(tracker,[(2.,0)])
        self.assertEqual(tracker.update(600_000_000,[],np.zeros(3)),[])
        with self.assertRaises(ValueError):tracker.update(500_000_000,[],np.zeros(3))

    def test_ego_motion_does_not_create_velocity(self):
        tracker=DormantTracker();tracker.update(0,[det(2.,0)],np.zeros(3));row=tracker.update(100_000_000,[det(1.8,0)],np.array([.2,0,0]))
        np.testing.assert_allclose(row[0]['velocity_odom'],[0,0],atol=1e-12)

    def test_late_future_odom_never_used(self):
        times=np.array([0,100,200]);ready=np.array([0,300,200]);v=np.zeros((3,9));v[1:]=1e6
        p=odom_at(150,times,ready,v);np.testing.assert_array_equal(p['pose'],[0,0,0])

if __name__=='__main__':unittest.main(verbosity=2)
