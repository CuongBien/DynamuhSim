#!/usr/bin/env python3
import unittest
import numpy as np
from sensor import Tracker,candidates,odom_at,project
from association import associate
from geometry import raster,warp,nearest_frame,interpolate


def detection(x,y):return dict(xy=np.array([x,y]),points=20,extent=np.array([.2,.2,.6]))


class ContractTests(unittest.TestCase):
    def test_projection_optical_axes(self):
        k=[2,0,1,0,2,1,0,0,1];t=np.array([[0,0,1,.12],[-1,0,0,0],[0,-1,0,.08],[0,0,0,1]])
        d=np.full((3,3),np.nan);d[1,1]=2
        np.testing.assert_allclose(project(d,k,t,stride=1),[[2.12,0,.08]])

    def test_clusters_reject_wall_extent(self):
        human=np.array([[1+x,.01*y,.05+z] for x in [0,.05,.1] for y in range(3) for z in [0,.2,.5]])
        wall=np.array([[3,y,z] for y in np.linspace(-2,2,80) for z in [.1,.4,.8]])
        self.assertEqual(len(candidates(np.vstack([human,wall]))),1)

    def test_tracker_ego_compensation(self):
        t=Tracker();a=t.update(0,[detection(2,0)],np.zeros(3))
        b=t.update(100_000_000,[detection(1.8,0)],np.array([.2,0,0]))
        self.assertEqual(a[0]['track_id'],b[0]['track_id'])
        np.testing.assert_allclose(b[0]['velocity_odom'],[0,0],atol=1e-12)

    def test_tracker_missing_is_not_observation(self):
        t=Tracker();t.update(0,[detection(2,0)],np.zeros(3))
        self.assertEqual(t.update(100_000_000,[],np.zeros(3)),[])

    def test_tracker_expired_identity_and_clock_reset(self):
        t=Tracker();a=t.update(0,[detection(2,0)],np.zeros(3))
        b=t.update(600_000_000,[detection(2,0)],np.zeros(3))
        self.assertNotEqual(a[0]['track_id'],b[0]['track_id'])
        with self.assertRaises(ValueError):t.update(500_000_000,[],np.zeros(3))

    def test_odom_future_and_late_delivery_poison(self):
        stamps=np.array([0,100,200]);available=np.array([0,300,200]);v=np.zeros((3,9));v[1:,:3]=1e6
        p=odom_at(150,stamps,available,v);np.testing.assert_array_equal(p['pose'],[0,0,0])
        self.assertIsNone(odom_at(150,stamps,available,v,max_age_ns=10))

    def test_association_one_to_one_and_ambiguity(self):
        ids,_=associate([[0,0],[.02,0]],[[0,0]])
        self.assertTrue(np.all(ids==-1))
        ids,_=associate([[0,0]],[[.1,0],[-.1,0]])
        self.assertEqual(ids.tolist(),[-1])
        ids,_=associate([[0,0],[2,0]],[[.1,0],[2.1,0]])
        self.assertEqual(ids.tolist(),[0,1])

    def test_invalid_depth_gives_unknown(self):
        d=np.array([[np.nan,np.inf],[0.,10.]])
        p=project(d,[1,0,0,0,1,0,0,0,1],np.eye(4),stride=1)
        self.assertEqual(len(p),0);self.assertTrue(np.all(raster(p,np.zeros(3))==-1))

    def test_anchor_rotation_and_translation(self):
        p=warp(np.array([[1.,0,.3]]),np.array([1.,0,np.pi/2]),np.array([0.,0,np.pi/2]))
        np.testing.assert_allclose(p,[[1,-1,.3]],atol=1e-12)
        grid=raster(p,np.array([0.,0,0]));self.assertEqual(grid[10,22],1)

    def test_ray_does_not_clear_behind_surface(self):
        g=raster(np.array([[1.,0,.3]]),np.zeros(3))
        self.assertEqual(g[10,32],1);self.assertEqual(g[15,32],-1)
        self.assertTrue(np.any(g[:10,32]==0))

    def test_outside_endpoint_carves_only_valid_ray(self):
        g=raster(np.array([[10.,0,.3]]),np.zeros(3))
        self.assertTrue(np.all(g[:,32]==0));self.assertTrue(np.all(g[:,40]==-1))

    def test_future_nearest_and_pose_support(self):
        i,v=nearest_frame(np.array([100,200,300]),np.array([200,350,351]),50)
        self.assertEqual(i.tolist(),[1,2,2]);self.assertEqual(v.tolist(),[True,True,False])
        p,v=interpolate(np.array([100,200]),np.array([[0.,0],[1,2]]),np.array([100,150,200,201]),100)
        self.assertEqual(v.tolist(),[True,True,True,False]);np.testing.assert_allclose(p[1],[.5,1])

if __name__=='__main__':unittest.main(verbosity=2)
