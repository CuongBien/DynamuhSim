"""Reproduce the exact saved infeasible human state, without running a simulator."""
import json,math,sys,unittest
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent
sys.path.insert(0,str(BENCH.parent/'navigation/socnavgym_warehouse_lifecycle'))
sys.path.insert(0,str(BENCH.parent/'navigation/socnavgym_group_nav2'))
from standing_projection import project_standing

class FailureRegression(unittest.TestCase):
    def test_captured_motion_state_is_infeasible_with_only_distant_obstacle(self):
        row=json.loads((HERE/'reports/motion01/failed_worker_request.json').read_text());request=row['request'];robot=request['robot'];humans=row['last_safe_humans'];dt=request['dt']
        starts=[(h['x'],h['y']) for h in humans];yielding={i for i,h in enumerate(humans) if math.hypot(h['x']-robot['x'],h['y']-robot['y'])<1.4}
        with self.assertRaisesRegex(RuntimeError,'Separation target exceeds available bounded speed'):
            project_standing(starts,[(0.,0.)]*len(humans),[h['radius'] for h in humans],
                (robot['x'],robot['y']),(robot['vx']*dt,robot['vy']*dt),.25,[dict(x=100000.,y=100000.,sx=1.,sy=1.)],dt,.8,set(range(len(humans))),yielding)

if __name__=='__main__':unittest.main(verbosity=2)
