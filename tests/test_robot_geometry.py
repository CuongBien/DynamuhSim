"""Native geometry alignment is isolated, immutable, and checked against recorded TF."""
import copy
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.simulation.robot_geometry import aligned_urdf
from dynamuhsim.runtime.capture import validate_config, CaptureContext
from dynamuhsim.analysis.capture import aligned_static_tf
from unittest.mock import patch

class GeometryTests(unittest.TestCase):
    def test_native_derivation_and_immutable_inputs(self):
        sdf=ROOT/'src/custom_corridor/models/mecanum_robot/model.sdf'
        template=ROOT/'src/custom_corridor/urdf/mecanum_robot.urdf'
        before=[p.read_bytes() for p in [sdf,template]]
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'aligned.urdf'; receipt=aligned_urdf(sdf,template,path);root=ET.parse(path).getroot()
            self.assertEqual(root.find("link[@name='base_link']/collision/geometry/box").get('size'),'0.24 0.20 0.08')
            self.assertEqual(receipt['joints']['base_link']['xyz'],[0.,0.,0.])
            self.assertEqual(receipt['joints']['camera_link']['xyz'],[.12,0.,.08])
            self.assertEqual(receipt['joints']['front_left_wheel']['xyz'],[.08,.1,0.])
            self.assertEqual(root.find("link[@name='front_left_wheel']/collision/geometry/sphere").get('radius'),'0.04')
            with self.assertRaises(FileExistsError):aligned_urdf(sdf,template,path)
        self.assertEqual([p.read_bytes() for p in [sdf,template]],before)

    def test_versioned_config_rejects_undeclared_tuning_and_keeps_old_contract(self):
        for name in ['geometry_v4_inflation045','geometry_v4_inflation080']:
            config=json.loads((ROOT/'configs/capture'/f'{name}.json').read_text());self.assertEqual(validate_config(config),config)
            for field,value in [('robot_radius',.1),('inflation_radius',.3)]:
                other=copy.deepcopy(config);other['nav2'][field]=value
                with self.assertRaises(ValueError):validate_config(other)
            other=copy.deepcopy(config);other['geometry_profile']='unknown'
            with self.assertRaises(ValueError):validate_config(other)
        for name in ['moderate','moderate_v2','moderate_v3']:
            config=json.loads((ROOT/'configs/capture'/f'{name}.json').read_text());self.assertEqual(validate_config(config),config)
            config['nav2']['robot_radius']=.17
            with self.assertRaises(ValueError):validate_config(config)

    def test_static_tf_checks_numeric_mount_and_every_duplicate(self):
        def row(x):return NS(header=NS(frame_id='base_link'),transform=NS(translation=NS(x=x,y=0,z=.08),rotation=NS(x=0,y=0,z=0,w=1)))
        joints={'camera_link':dict(parent='base_link',type='fixed',xyz=[.12,0,.08],rpy=[0,0,0])}
        self.assertTrue(aligned_static_tf({'camera_link':[row(.12)]},joints))
        self.assertFalse(aligned_static_tf({'camera_link':[row(.14)]},joints))
        self.assertFalse(aligned_static_tf({'camera_link':[row(.12),row(.14)]},joints))
        self.assertFalse(aligned_static_tf({},joints))

    def test_launch_override_is_only_set_for_version4(self):
        context=CaptureContext.__new__(CaptureContext)
        with tempfile.TemporaryDirectory() as d,patch('dynamuhsim.runtime.capture.subprocess.Popen'):
            for version in [3,4]:
                context.config={'schema':f'dynamuhsim_capture_config/{version}'};env={'DATASET_ROBOT_URDF':'stale'};files=[]
                context.start_bridge(Path(d),Path(d),env,[],files)
                self.assertEqual(env.get('DATASET_ROBOT_URDF'),str(Path(d)/'mecanum_aligned.urdf') if version==4 else None)
                for f in files:f.close()

if __name__=='__main__':unittest.main()
