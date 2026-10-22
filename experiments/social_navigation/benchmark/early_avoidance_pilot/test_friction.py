"""Native SDF regression for literal Gazebo extension names, with no simulator run."""
import json,subprocess,sys,tempfile,unittest
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent
class FrictionPrefixRegression(unittest.TestCase):
    def test_native_parser_distinguishes_bad_prefix_and_restored_original(self):
        sys.path.insert(0,str(BENCH/'depth_proxy_lifecycle'))
        from namespace_prepare import preserve_friction_prefix
        import xml.etree.ElementTree as ET
        source=HERE.parents[3]/'src/custom_corridor/models/mecanum_robot/model.sdf'
        xml=source.read_text().replace('<sdf version="1.8">','<sdf version="1.8" xmlns:gz="urn:gz">')
        checker=HERE/'generated/check_friction'
        with tempfile.TemporaryDirectory() as folder:
            before=Path(folder)/'bad.sdf'
            ET.register_namespace('incorrect','urn:gz');ET.ElementTree(ET.fromstring(xml)).write(before,encoding='unicode')
            bad=subprocess.run([str(checker),str(before)],capture_output=True,text=True)
            self.assertEqual(bad.returncode,1);self.assertEqual(json.loads(bad.stdout)['recognized_gz_base_link_frames'],0)
            fixed=Path(folder)/'robot.sdf';fixed.write_bytes(before.read_bytes());preserve_friction_prefix(fixed)
            good=subprocess.run([str(checker),str(fixed)],capture_output=True,text=True)
            self.assertEqual(good.returncode,0);self.assertEqual(json.loads(good.stdout)['recognized_gz_base_link_frames'],4)

if __name__=='__main__':unittest.main(verbosity=2)
