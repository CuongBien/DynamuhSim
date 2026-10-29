import copy,json,tempfile,unittest
from pathlib import Path
import xml.etree.ElementTree as ET
from fixture import HERE,create_fixture,canonical,PROXY_FLAG,ALL_MASK,NO_PROXY_MASK,collision_signature

class FixtureTests(unittest.TestCase):
    def test_mask_excludes_proxy_but_keeps_ordinary_visuals_and_lidar(self):
        self.assertEqual(PROXY_FLAG & 0xf0000003, 0)  # Reject reserved selection/GUI/selectable bits.
        self.assertEqual(NO_PROXY_MASK&PROXY_FLAG,0)
        self.assertNotEqual(NO_PROXY_MASK&ALL_MASK,0)
        self.assertNotEqual(ALL_MASK&PROXY_FLAG,0)

    def test_three_conditions_keep_collisions_and_native_scan_definition(self):
        scene=json.loads((HERE/'snapshots.json').read_text())[1]
        with tempfile.TemporaryDirectory() as folder:
            metadata=[];native=[]
            for condition in ['baseline','masked','null_visuals']:
                out=Path(folder)/condition;out.mkdir()
                path,meta=create_fixture(out,scene,condition);metadata.append(meta)
                tree=ET.parse(path);world=tree.getroot().find('world')
                self.assertEqual(meta['collision'],collision_signature(world))
                s=world.find("model[@name='mecanum_robot']/link[@name='base_scan']/sensor[@name='ablation_native']")
                s=copy.deepcopy(s);s.find('topic').text='constant';native.append(canonical(s))
                if condition=='masked':
                    cams=world.findall("model[@name='mecanum_robot']/link[@name='camera_link']/sensor")
                    self.assertEqual(len(cams),4)
                    for c in cams:
                        expected=NO_PROXY_MASK if c.get('name').endswith('_off') else ALL_MASK
                        self.assertEqual(int(c.findtext('camera/visibility_mask')),expected)
                    self.assertEqual(len(world.findall('actor')),6)
                elif condition=='null_visuals':
                    self.assertFalse(world.findall('actor'))
                    for m in world.findall('model'):
                        if m.get('name','').endswith('_proxy'):self.assertFalse(m.findall('link/visual'))
            self.assertEqual(metadata[0]['collision'],metadata[1]['collision'])
            self.assertEqual(metadata[0]['collision'],metadata[2]['collision'])
            self.assertEqual(native[0],native[1]);self.assertEqual(native[0],native[2])

if __name__=='__main__':unittest.main(verbosity=2)
