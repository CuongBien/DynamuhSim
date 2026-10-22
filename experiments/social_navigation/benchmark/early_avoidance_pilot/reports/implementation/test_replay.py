import importlib.util,json,math,subprocess,sys,tempfile,unittest
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent
spec=importlib.util.spec_from_file_location('group_metric_geometry',HERE/'geometry.py');g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
sys.path.insert(0,str(BENCH/'socnavgym_warehouse_lifecycle'));sys.path.insert(0,str(BENCH/'socnavgym_group_nav2'))
from standing_projection import project_standing

class ReplayTests(unittest.TestCase):
    def project(self,scale):
        saved=json.loads((BENCH/'depth_proxy_lifecycle/reports/motion01/failed_worker_request.json').read_text());req=saved['request'];bot=req['robot'];humans=saved['last_safe_humans'];dt=req['dt']
        yielding={i for i,h in enumerate(humans) if math.hypot(h['x']-bot['x'],h['y']-bot['y'])<1.4}
        return project_standing([(h['x'],h['y']) for h in humans],[(0.,0.)]*6,[h['radius'] for h in humans],(bot['x'],bot['y']),(bot['vx']*scale*dt,bot['vy']*scale*dt),.25,[dict(x=100000.,y=100000.,sx=1.,sy=1.)],dt,.8,set(range(6)),yielding)
    def test_old_failure_stays_rejected(self):
        with self.assertRaisesRegex(RuntimeError,'Separation target exceeds available bounded speed'):self.project(1.)
    def test_reduced_approach_can_be_feasible_without_raising_human_cap(self):
        d,a=self.project(.18/math.hypot(.3504797527798281,-.03678787053471345))
        self.assertLessEqual(max(math.hypot(*v)/.1 for v in d),.2+1e-7)
        self.assertGreaterEqual(min(a['predicted_clearance'].values()),.002-1e-8)
    def test_hull_intrusion_includes_interior_and_edge_but_not_clear_disk(self):
        p=g.hull([(0,0),(2,0),(1,2)])
        self.assertEqual(g.distance_to_hull((1,.5),p),0)
        self.assertEqual(g.distance_to_hull((1,0),p),0)
        self.assertAlmostEqual(g.distance_to_hull((1,-.3),p),.3)
        self.assertGreater(g.distance_to_hull((1,-.3),p),.25)

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
