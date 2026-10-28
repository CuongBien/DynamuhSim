"""Exercise runner orchestration with in-memory ROS/Gazebo endpoints and owned subprocesses."""
from contextlib import ExitStack, redirect_stdout, redirect_stderr
from functools import partial
import io
import json
from pathlib import Path
from dynamuhsim.workspace import stage_directory
import shutil
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
from dynamuhsim.runtime import nav2_trial as trial
from dynamuhsim.runtime.navigation import execute_goal, wait_for
from dynamuhsim.runtime.profiles import PROFILES
from test_nav2_runtime import message

BENCH = ROOT / "experiments/social_navigation/benchmark"
FIXTURES = ROOT / "tests/fixtures/nav2_runtime"


class RunnerHarness:
    def __init__(self, root, name, flags, fault=False):
        self.root, self.profile, self.fault = root, PROFILES[name], fault
        self.events, self.subscriptions, self.gz_subscriptions, self.processes = [], [], {}, []
        self.wall = self.sim = 0.
        self.goal_active = False
        self.goal_ticks = self.canceled = 0
        self.fixture = json.loads((FIXTURES / "telemetry.json").read_text())
        self.stage = stage_directory(root,name)
        self.stage.mkdir(parents=True)
        shutil.copyfile(stage_directory(ROOT,name) / "protocol.json", self.stage / "protocol.json")
        self.entry = self.stage / "trial.py"
        self.entry.write_text("# compatibility entry point\n")
        self.args = self.profile.parse_args(flags + ["--report-dir", str(root.parent / "report")])
        self.runtime = self.stage / "generated/report"
        benchmark = self.stage.parent
        for profile in PROFILES.values():
            stage = benchmark / profile.name
            for relative in ["reports/gate01/validation.json", "reports/reaction_gate.json",
                             "reports/gate03/validation.json", "reports/offline05/validation.json",
                             "reports/empty01/validation.json"]:
                path = stage / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text('{"status": "PASS", "actual_goal_reached": true}')
        warehouse = root / "experiments/social_navigation/warehouse"
        warehouse.mkdir(parents=True)
        (warehouse / "nav2_warehouse.yaml").write_text(json.dumps({"amcl": {"ros__parameters": {"initial_pose": {"x": 0, "y": 0, "yaw": 0}}}}))
        if self.profile.container_worker:
            shutil.copyfile(stage_directory(ROOT,name) / "runtime.py", self.stage / "runtime.py")
            inputs = benchmark/'common_response/inputs'
            inputs.mkdir(parents=True)
            (inputs / "images.json").write_text(json.dumps({"dynamuhsim-crowdsimplus:round2": "pinned:csp", "dynamuhsim-complexitynav:round3": "pinned:cn"}))
        self.humans = [dict(id=i, x=-25 + i / 10, y=i / 10) for i in range(1, 7)]

    def now(self):
        return self.wall

    def popen(self, command, **kwargs):
        self.events.append("spawn:" + command[0])
        if command[0] == "ros2":
            kwargs["stdout"].write("[INFO] [gz-1]: process has finished cleanly [pid 123]\n")
            kwargs["stdout"].flush()
        process = NS(command=command, returncode=None)
        process.poll = lambda: process.returncode
        process.send_signal = lambda sig: self.stop(process)
        process.wait = lambda timeout: process.returncode
        self.processes.append(process)
        if command[0] not in ["ros2", "gz"]:
            (self.runtime / "ready.json").write_text(json.dumps({"humans": self.humans, "requires_start_gate": True}))
        return process

    def stop(self, process):
        self.events.append("stop:" + process.command[0])
        process.returncode = 0

    def cancel(self):
        self.canceled += 1
        self.events.append("cancel")
        return NS(done=lambda: True)

    def client(self, node, kind, topic):
        handle = NS(accepted=True, cancel_goal_async=self.cancel)
        def get_result():
            self.goal_active = True
            return NS(done=lambda: self.goal_ticks >= 3,
                      result=lambda: NS(status=4, result=NS(error_code=0, error_msg="")))
        handle.get_result_async = get_result
        def send(goal, feedback_callback):
            self.events.append("goal")
            self.assert_goal = (goal.pose.header.frame_id, goal.pose.pose.position.x, goal.pose.pose.position.y, goal.pose.pose.orientation.w)
            return NS(done=lambda: True, result=lambda: handle)
        return NS(server_is_ready=lambda: True, send_goal_async=send)

    def spin(self, node, timeout_sec):
        self.wall += .1
        self.sim += .1
        if self.goal_active:
            self.goal_ticks += 1
        for kind, topic, callback, qos in self.subscriptions:
            key = {"/common/ground_truth": "gt", "/scan": "scan", "/imu": "imu", "/odom": "odom", "/clock": "clock",
                   "/amcl_pose": "amcl", "/cmd_vel": "cmd", "/plan": "plan", "/local_costmap/costmap": "local_costmap", "/rosout": "log"}[topic]
            if key == "clock":
                callback(NS(clock=NS(sec=int(self.sim), nanosec=round((self.sim % 1) * 1e9))))
            elif key == "gt":
                if self.args.condition != "empty":
                    callback(NS(data=json.dumps({"adapter_ok": not (self.fault and self.goal_active), "error": "injected worker fault"})))
            elif key == "imu":
                callback(NS())
            else:
                row = next(x for x in self.fixture["messages"] if x["key"] == key)
                value = message(row["message"])
                if key == "scan":
                    value.ranges = [1., 2., float("nan"), float("inf")]
                callback(value)
        pose = dict(position=dict(x=-20.1442 if self.goal_active else -28., y=0., z=0.), orientation=dict(x=0., y=0., z=0., w=1.))
        humans = [{"name": f"common_{h['id']}_proxy", "position": {"x": h["x"], "y": h["y"], "z": 0}, "orientation": pose["orientation"]} for h in self.humans]
        self.gz_subscriptions["/world/social_warehouse/pose/info"](message({"header": {"stamp": {"sec": int(self.sim), "nsec": round((self.sim % 1) * 1e9)}}, "pose": [{"name": "mecanum_robot", **pose}, *humans]}))

    def dependencies(self):
        outer = self
        class Node:
            def __init__(self, name, **kwargs):
                outer.events.append("node:" + name)
            def create_subscription(self, kind, topic, callback, qos):
                outer.subscriptions.append((kind, topic, callback, qos))
            def create_client(self, kind, topic):
                return NS(service_is_ready=lambda: True, call_async=lambda request: NS(done=lambda: True, result=lambda: NS(current_state=NS(id=3))))
            def get_publishers_info_by_topic(self, topic):
                return [NS(node_name="controller_server", node_namespace="/")]
            def destroy_node(self):
                outer.events.append("destroy_node")
        class Guard:
            token = "fake-owned-session-token"
            def __init__(self, runtime):
                outer.events.append("guard")
            def cleanup(self):
                outer.events.append("cleanup")
                return {"status": "PASS", "remaining": []}
        def make_world(runtime, socket_path, humans, mode):
            shutil.copyfile(FIXTURES / "world_input.sdf", runtime / "warehouse.sdf")
        def goal():
            return message({"pose": {"header": {}, "pose": {"position": {"x": 0, "y": 0}, "orientation": {"w": 0}}}})
        def server_stop(guard):
            outer.events.append("server_stop")
            return {"status": "STOPPED"}
        def tf_lookup(*args):
            return message({"transform": {"translation": {"x": 0, "y": 0}, "rotation": {"x": 0, "y": 0, "z": 0, "w": 1}}})
        values = {key: object() for key in ["String", "LaserScan", "Imu", "Odometry", "Clock", "PoseWithCovarianceStamped", "TwistStamped", "RosPath", "OccupancyGrid", "Log", "Pose_V", "GzImage", "qos_profile_sensor_data"]}
        return NS(**values, Node=Node, ActionClient=self.client, GetState=NS(Request=lambda: NS()), NavigateToPose=NS(Goal=goal),
                  rclpy=NS(init=lambda: None, shutdown=lambda: outer.events.append("ros_shutdown"), spin_once=self.spin,
                           parameter=NS(Parameter=lambda *args, **kwargs: NS()), time=NS(Time=lambda: None)),
                  GzNode=lambda: NS(subscribe=lambda kind, topic, callback: outer.gz_subscriptions.update({topic: callback})),
                  Buffer=lambda: NS(lookup_transform=tf_lookup), TransformListener=lambda *args: NS(),
                  gpu=NS(RuntimeGuard=Guard, gpu_preflight=lambda runtime: {"hardware_verified": True}),
                  shutdown=NS(launch_environment=lambda runtime: ({}, {"test_stub": True}), stop_server=server_stop),
                  adapter=NS(make_world=make_world, stop=self.stop),
                  observer=NS(observer_qos=lambda mode: "transient-local", publisher_evidence=lambda node: []),
                  yaml=NS(safe_load=json.loads, safe_dump=lambda value, sort_keys: json.dumps(value)))

    def run(self, capture=None):
        with ExitStack() as stack:
            stack.enter_context(patch.object(trial, "stop_paused", side_effect=lambda guard, stop: (self.events.append("quiesce"), stop(guard))[1]))
            stack.enter_context(patch.object(trial, "source_files", return_value=[self.entry, self.stage / "protocol.json"]))
            stack.enter_context(patch.object(trial.subprocess, "Popen", side_effect=self.popen))
            stack.enter_context(patch.object(trial.subprocess, "check_output", side_effect=lambda args, **kwargs: "fixed-commit" if args[0] == "git" else ""))
            stack.enter_context(patch.object(trial.time, "monotonic", side_effect=self.now))
            stack.enter_context(patch.object(trial, "wait_for", partial(wait_for, monotonic=self.now)))
            stack.enter_context(patch.object(trial, "execute_goal", partial(execute_goal, monotonic=self.now)))
            stack.enter_context(redirect_stdout(io.StringIO()))
            stack.enter_context(redirect_stderr(io.StringIO()))
            code = trial.run_trial(self.profile, self.args, self.root, self.entry, dependencies=self.dependencies(), capture=capture)
        return code, json.loads((self.args.report_dir / "result.json").read_text())


class RunnerIntegrationTests(unittest.TestCase):
    def test_all_supported_workers_preserve_goal_and_cleanup_contracts(self):
        cases = [("socnavgym_nav2_pilot", ["--condition", "empty"]),
                 ("socnavgym_nav2_pilot", ["--condition", "people"]),
                 ("socnavgym_group_nav2", ["--scenario", "standing", "--seed", "42", "--gui"]),
                 ("shared_nav2_pilot", ["--model", "crowd_orca_plus"]),
                 ("shared_nav2_pilot", ["--model", "complex_geometry_mixed"])]
        for name, flags in cases:
            with self.subTest(name=name, flags=flags), tempfile.TemporaryDirectory() as directory:
                harness = RunnerHarness(Path(directory) / "repo", name, flags)
                code, result = harness.run()
                self.assertEqual(code, 0, result)
                self.assertEqual(result["status"], "MEASURED")
                self.assertTrue(result["actual_goal_reached"])
                self.assertEqual(harness.assert_goal, ("map", -20., 0., 1.))
                self.assertEqual(result["cleanup"], {"status": "PASS", "remaining": []})
                self.assertEqual(result["ros_launch_shutdown"]["status"], "STOPPED")
                self.assertTrue(all(proc.returncode == 0 for proc in harness.processes))
                self.assertLess(harness.events.index("quiesce"), harness.events.index("server_stop"))
                self.assertLess(harness.events.index("server_stop"), harness.events.index("cleanup"))
                self.assertEqual(len(result["source_sha256"]), 2)
                if harness.profile.container_worker:
                    self.assertEqual(result["container_remaining"], "")
                    self.assertEqual(result["native_dt"], .1 if "crowd_orca_plus" in flags else .25)
                if harness.profile.groups:
                    self.assertIn("gui_start_mode", result)
                    self.assertLess(harness.events.index("stop:gz"), harness.events.index("server_stop"))
                with self.assertRaises(FileExistsError):
                    harness.run()

    def test_fault_after_goal_acceptance_cancels_then_cleans_owned_processes(self):
        with tempfile.TemporaryDirectory() as directory:
            harness = RunnerHarness(Path(directory) / "repo", "socnavgym_nav2_pilot", ["--condition", "people"], fault=True)
            code, result = harness.run()
            self.assertEqual(code, 1)
            self.assertEqual(result["status"], "ERROR")
            self.assertIn("Terminal adapter fault", result["error"])
            self.assertEqual(harness.canceled, 1)
            self.assertEqual(len(result["faults"]), 1)
            self.assertLess(harness.events.index("cancel"), harness.events.index("server_stop"))
            self.assertEqual(harness.events[-1], "cleanup")
            self.assertTrue(all(proc.returncode == 0 for proc in harness.processes))


class QuiesceTests(unittest.TestCase):
    def test_capture_and_nav2_use_the_same_shutdown_helper(self):
        from dynamuhsim.runtime.capture import stop_paused as capture_stop
        from dynamuhsim.runtime.shutdown import stop_paused
        self.assertIs(capture_stop, stop_paused)
        self.assertIs(trial.stop_paused, stop_paused)

    def test_acknowledgement_and_clock_stability_are_recorded_before_ordered_stop(self):
        from dynamuhsim.runtime import shutdown
        for mode in ["stable", "advancing", "rejected", "unsubscribed"]:
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                events = []
                callbacks = []
                class Transport:
                    def subscribe(self, kind, topic, callback):
                        events.append("subscribe"); callbacks.append(callback)
                        return mode != "unsubscribed"
                    def unsubscribe(self, topic):
                        events.append("unsubscribe")
                def stats(nsec):
                    return NS(paused=True, sim_time=NS(sec=10, nsec=nsec))
                def request(*args, **kwargs):
                    events.append("pause_request")
                    callbacks[0](stats(0))
                    return NS(returncode=0 if mode != "rejected" else 1, stdout="data: true" if mode != "rejected" else "", stderr="")
                def settle(seconds):
                    events.append("settle")
                    self.assertEqual(seconds, .5)
                    callbacks[0](stats(1 if mode == "advancing" else 0))
                guard = NS(runtime=Path(directory))
                def stop(received):
                    self.assertIs(received, guard); events.append("stop")
                    return {"status": "STOPPED"}
                modules = {"gz": NS(__path__=[]), "gz.transport13": NS(Node=Transport),
                           "gz.msgs10": NS(__path__=[]), "gz.msgs10.world_stats_pb2": NS(WorldStatistics=object)}
                with patch.dict(sys.modules, modules), patch.object(shutdown.subprocess, "run", side_effect=request), patch("time.sleep", side_effect=settle):
                    result = shutdown.stop_paused(guard, stop)
                self.assertLess(events.index("pause_request"), events.index("unsubscribe"))
                self.assertLess(events.index("unsubscribe"), events.index("stop"))
                self.assertEqual(events.count("stop"), 1)
                self.assertEqual(result["status"], "STOPPED")
                self.assertEqual(result["quiesce"]["pause_acknowledged"], mode in ["stable", "advancing"])
                self.assertEqual(result["quiesce"]["simulation_time_stable"], mode == "stable")
                self.assertFalse(result["quiesce"]["is_gpu_fence"])
                self.assertEqual(json.loads((Path(directory) / "quiesce_shutdown.json").read_text()), result)


if __name__ == "__main__":
    unittest.main()
