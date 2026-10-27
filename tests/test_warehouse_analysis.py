"""Warehouse measurement parity, boundary cases and unchanged analyzer policies."""
import ast
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
from dynamuhsim.analysis import warehouse as measurements
from dynamuhsim.analysis.warehouse import robot_human_gaps
HAS_SHAPELY = importlib.util.find_spec("shapely") is not None
NAMES = ("common_gazebo", "socnavgym_response_fix")
FIXTURE = ROOT / "tests/fixtures/warehouse_analysis.json"


def synthetic(dt, prefix, duplicate=False):
    proxies = {i: f"{prefix}_{i}_proxy" for i in range(1,7)}
    worker = [dict(time=0., before=[dict(id=1,x=-1.,y=0.),dict(id=2,x=0.,y=-1.)],
                   humans=[dict(id=1,x=1.,y=0.),dict(id=2,x=0.,y=1.)],
                   robot_input=dict(x=3.,y=3.,vx=.1,vy=0.))]
    times = [-dt, 0., dt/2, dt, 2*dt, 2*dt+.00002]
    if duplicate:
        times.insert(2, 0.)
    complete = [dict(time=t,poses={
        proxies[1]:dict(x=-1+2*t/dt,y=0.,yaw=math.radians(179 if i%2 else -179)),
        proxies[2]:dict(x=0.,y=-1+2*t/dt,yaw=0.),
        "mecanum_robot":dict(x=3+.1*t,y=3.,yaw=0.),
    }) for i,t in enumerate(times)]
    for hid in range(3,7):
        human = dict(id=hid,x=10.*hid,y=10.)
        worker[0]["before"].append(human.copy())
        worker[0]["humans"].append(human.copy())
        for row in complete:
            row["poses"][proxies[hid]] = dict(x=human["x"],y=human["y"],yaw=0.)
    boxes = [dict(x=8.,y=8.,sx=1.,sy=1.)]
    return dict(PROXIES=proxies,worker=worker,complete=complete,dt=dt,pose_times=times,
                worker_times=[0.],boxes=boxes,scans=[],assessment_end=None)


def current_result(data):
    before,after,rb,re = measurements.pose_intervals(data["complete"],data["PROXIES"])
    errors,prediction = measurements.trajectory_errors(data["complete"],data["worker"],data["dt"],data["PROXIES"],data["pose_times"])
    speed,yaw,path = measurements.human_motion(data["complete"],data["PROXIES"])
    lidar,ids = measurements.lidar_measurements(data["scans"],data["complete"],data["pose_times"],data["boxes"],data["PROXIES"])
    return dict(gaps=measurements.measured_gaps(before,after,rb,re,polygons=data["polygons"]),
                errors=errors,robot_prediction=prediction,max_speed=speed,max_yaw_rate=yaw,human_path=path,lidar_errors=lidar,lidar_ids=sorted(ids))


def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest()


def policy(source):
    tree = ast.parse(source)
    assertions = [ast.dump(n,include_attributes=False) for n in ast.walk(tree) if isinstance(n,ast.Assert)]
    gates = [ast.dump(n,include_attributes=False) for n in ast.walk(tree) if isinstance(n,ast.For)
             and any(word in ast.unparse(n.iter) for word in ("source_sha256", "preserved.items", "hashes.json"))]
    validation = next(n.value for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=="validation" for t in n.targets))
    parser = [ast.dump(n,include_attributes=False) for n in ast.walk(tree) if isinstance(n,ast.Call)
              and isinstance(n.func,ast.Attribute) and n.func.attr=="add_argument"]
    return dict(assertions=digest(assertions),source_gates=digest(gates),validation_fields=digest(ast.dump(validation,include_attributes=False)),cli_arguments=digest(parser),assertion_count=len(assertions))


class WarehouseAnalysisTests(unittest.TestCase):
    @unittest.skipUnless(HAS_SHAPELY, "Run geometry checks in the existing Shapely analyzer environment")
    def test_exact_legacy_measurement_outputs_with_interpolation_edges_and_duplicate_times(self):
        import shapely
        golden = json.loads(FIXTURE.read_text())
        for dt in (.05, .1, .2):
            for prefix in ("common", "lifecycle"):
                if prefix == "lifecycle" and dt != .1:
                    continue
                for duplicate in (False, True):
                    key = f"{dt}:{prefix}:{duplicate}"
                    with self.subTest(case=key):
                        data = synthetic(dt,prefix,duplicate)
                        data["polygons"] = np.array([shapely.box(7.5,7.5,8.5,8.5)],dtype=object)
                        self.assertEqual(digest(current_result(data)),golden["measurements"][key])

    @unittest.skipUnless(HAS_SHAPELY, "Shapely measurement dependency")
    def test_swept_gap_detects_collision_between_separated_endpoints(self):
        import shapely
        gaps = measurements.measured_gaps([[[-1.,0.],[0.,-1.]]],[[[1.,0.],[0.,1.]]],[[3.,3.]],[[3.,3.]],
                                          polygons=np.array([shapely.box(7.,7.,8.,8.)],dtype=object))
        self.assertEqual(gaps["pair"],-.72)
        self.assertGreater(gaps["robot"],0.)
        self.assertGreater(gaps["static"],0.)

    def test_lidar_uses_human_visibility_and_rejects_old_or_missing_measurements(self):
        complete = [dict(time=0.,poses={"mecanum_robot":dict(x=0.,y=0.,yaw=0.),"human":dict(x=2.,y=0.)})]
        scan = dict(time=0.,angle_min=0.,angle_increment=.1,ranges=[1.64],range_min=.1,range_max=10.)
        args = (complete,[0.],[],{7:"human"})
        errors,ids = measurements.lidar_measurements([scan],*args)
        self.assertEqual(len(errors),1); self.assertAlmostEqual(errors[0],0.); self.assertEqual(ids,{7})
        for missing in (None, float("nan"), float("inf")):
            errors,ids = measurements.lidar_measurements([{**scan,"ranges":[missing]}],*args)
            self.assertEqual(errors,[10.]); self.assertEqual(ids,set())
        self.assertEqual(measurements.lidar_measurements([{**scan,"time":.025001}],*args),([],set()))
        boxes = [dict(x=1.,y=0.,sx=.2,sy=1.)]
        self.assertEqual(measurements.lidar_measurements([scan],complete,[0.],boxes,{7:"human"}),([],set()))

    def test_motion_skips_duplicate_clock_and_wraps_yaw(self):
        rows = [dict(time=t,poses={"human":dict(x=x,y=0.,yaw=math.radians(yaw))})
                for t,x,yaw in [(0.,0.,0.),(0.,1.,179.),(.1,1.1,-179.)]]
        speed,yaw,path = measurements.human_motion(rows,{1:"human"})
        self.assertAlmostEqual(speed,1.)
        self.assertAlmostEqual(yaw,math.radians(2)/.1)
        self.assertAlmostEqual(path,.1)

    def test_every_acceptance_source_gate_cli_and_validation_field_is_unchanged(self):
        fixture = json.loads(FIXTURE.read_text())
        for name in NAMES:
            with self.subTest(analyzer=name):
                source = (ROOT / "experiments/social_navigation/benchmark" / name / "analyze.py").read_text()
                self.assertEqual(policy(source),fixture["policies"][name])

    def test_entrypoints_wire_the_original_proxy_names_and_time_steps(self):
        for name in NAMES:
            tree = ast.parse((ROOT / "experiments/social_navigation/benchmark" / name / "analyze.py").read_text())
            proxies = next(n for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=="PROXIES" for t in n.targets))
            ns = {}; exec(compile(ast.Module(body=[proxies],type_ignores=[]),name,"exec"),ns)
            self.assertEqual(ns["PROXIES"],{i:f"common_{i}_proxy" for i in range(1,7)})
            call = next(n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=="trajectory_errors")
            dt = eval(compile(ast.Expression(call.args[2]),name,"eval"),dict(dt=.2))
            self.assertEqual(dt,.2)
            self.assertFalse(any(isinstance(n,ast.FunctionDef) and n.name=="measured_gaps" for n in tree.body))


    def test_interval_gaps_match_audited_linear_closest_approach_boundaries(self):
        # Crossing, stationary, past/future closest approach: no extrapolation.
        before = [[[1.,0.]],[[1.,0.]],[[1.,0.]],[[2.,0.]]]
        after = [[[-1.,0.]],[[1.,0.]],[[2.,0.]],[[1.,0.]]]
        robot = [[0.,0.]]*4
        np.testing.assert_array_equal(robot_human_gaps(before,after,robot,robot),[-.61,.39,.39,.39])


if __name__ == "__main__":
    unittest.main()
