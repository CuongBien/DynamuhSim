import unittest
import numpy as np
from geometry import world_rays,floor_depth,cylinder_depth,optical_transform

class GeometryTests(unittest.TestCase):
    def test_floor_depends_on_height_and_downward_rays(self):
        directions=np.array([[[1,0,-.5],[1,0,0],[1,0,.5]]],float)
        np.testing.assert_allclose(floor_depth([0,0,.12],directions)[0,0],.24)
        np.testing.assert_allclose(floor_depth([0,0,.4],directions)[0,0],.8)
        self.assertTrue(np.isinf(floor_depth([0,0,.4],directions)[0,1:]).all())

    def test_low_and_high_rays_have_same_xy(self):
        robot=dict(x=4,y=-2,yaw=.7)
        a,da=world_rays([200,0,2,0,200,2,0,0,1],(4,4),robot,.08)
        b,db=world_rays([200,0,2,0,200,2,0,0,1],(4,4),robot,.36)
        np.testing.assert_allclose(a[:2],b[:2]);np.testing.assert_allclose(b[2]-a[2],.28)
        np.testing.assert_array_equal(da,db)

    def test_cylinder_is_still_occluding_at_both_heights(self):
        rays=np.array([[[1,0,0],[1,2,0]]],float)
        for h in [.12,.4]:
            d=cylinder_depth(np.array([0,0,h]),rays,[dict(x=1,y=0)])
            self.assertAlmostEqual(d[0,0],.64)
            self.assertTrue(np.isinf(d[0,1]))

    def test_ray_above_cylinder_does_not_hit_side(self):
        d=cylinder_depth(np.array([0,0,2.]),np.array([[[1.,0,0]]]),[dict(x=1,y=0)])
        self.assertTrue(np.isinf(d[0,0]))

    def test_fixture_freezes_actor_and_sensor_extrinsics(self):
        import json,tempfile
        from pathlib import Path
        import xml.etree.ElementTree as ET
        from fixture import create_fixture,HERE
        scene=json.loads((HERE/'inputs/snapshots.json').read_text())[0]
        with tempfile.TemporaryDirectory() as folder:
            world=create_fixture(Path(folder),scene)
            tree=ET.parse(world)
            actors=tree.findall('world/actor')
            self.assertEqual(len(actors),6)
            for a in actors:
                self.assertEqual(a.findtext('pose'),'0 0 0 0 0 0')
                self.assertEqual(a.findtext('script/auto_start'),'false')
            plugin=tree.find("world/plugin[@name='height_experiment::FreezeActors']")
            self.assertEqual(len(plugin.findall('frozen_actor')),6)
            self.assertEqual(tree.findtext("world/model[@name='mecanum_robot']/static"),'true')
            cam=tree.find("world/model[@name='mecanum_robot']/link[@name='camera_link']")
            self.assertEqual(len(cam.findall('sensor')),2)
            self.assertAlmostEqual(float(cam.find("sensor[@name='height_high']/pose").text.split()[2]),.28)

if __name__=='__main__':unittest.main(verbosity=2)
