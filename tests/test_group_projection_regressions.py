"""Current projection math against the immutable recorded infeasible state."""
import json, math, unittest
from pathlib import Path
from dynamuhsim.analysis.evidence import module_from_path
ROOT = Path(__file__).resolve().parents[1]
NAV = ROOT / 'experiments/social_navigation/navigation'
geometry = module_from_path(NAV / 'socnavgym_warehouse_lifecycle/geometry.py', '_regression_geometry')
projection = module_from_path(NAV / 'socnavgym_group_nav2/standing_projection.py', '_regression_standing', {'geometry': geometry})

class RecordedProjection(unittest.TestCase):
    def project(self, scale):
        saved = json.loads((ROOT / 'experiments/archive/social_navigation/reports/depth_proxy_lifecycle/motion01/failed_worker_request.json').read_text())
        request = saved['request']; robot = request['robot']; humans = saved['last_safe_humans']; dt = request['dt']
        yielding = {i for i, human in enumerate(humans) if math.hypot(human['x']-robot['x'], human['y']-robot['y']) < 1.4}
        return projection.project_standing([(h['x'],h['y']) for h in humans], [(0.,0.)]*6, [h['radius'] for h in humans],
            (robot['x'],robot['y']), (robot['vx']*scale*dt,robot['vy']*scale*dt), .25,
            [dict(x=100000.,y=100000.,sx=1.,sy=1.)], dt, .8, set(range(6)), yielding)
    def test_recorded_infeasible_motion_stays_rejected(self):
        with self.assertRaisesRegex(RuntimeError, 'Separation target exceeds available bounded speed'):
            self.project(1.)
    def test_reduced_approach_keeps_human_speed_and_clearance_contract(self):
        delta, audit = self.project(.18/math.hypot(.3504797527798281,-.03678787053471345))
        self.assertLessEqual(max(math.hypot(*v)/.1 for v in delta), .2+1e-7)
        self.assertGreaterEqual(min(audit['predicted_clearance'].values()), .002-1e-8)
