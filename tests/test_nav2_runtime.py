"""Regression contracts from the pushed pilots; no simulator or ROS process is started."""
import copy
import importlib.util
import json
import math
import signal
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
from dynamuhsim.runtime.navigation import execute_goal, goal_metrics, wait_for
from dynamuhsim.runtime.profiles import PROFILES
from dynamuhsim.runtime.recording import TrialRecording
from dynamuhsim.runtime.nav2_trial import source_files, stop_ros_launch
from dynamuhsim.simulation.warehouse import configure_world, generate_launch_description, runtime_nav2_config

FIXTURES = ROOT / "tests/fixtures/nav2_runtime"
BENCH = ROOT / "experiments/social_navigation/benchmark"


def message(value):
    if isinstance(value, dict):
        return NS(**{key: message(item) for key, item in value.items()})
    if isinstance(value, list):
        return [message(item) for item in value]
    return value


class ProfileTests(unittest.TestCase):
    def test_existing_argument_contracts(self):
        individual = PROFILES["socnavgym_nav2_pilot"]
        for condition in ["empty", "people"]:
            args = individual.parse_args(["--condition", condition, "--report-dir", "/tmp/fixed"])
            self.assertEqual(args.condition, condition)
        groups = PROFILES["socnavgym_group_nav2"]
        args = groups.parse_args(["--scenario", "standing", "--seed", "43", "--report-dir", "/tmp/fixed", "--gui"])
        self.assertEqual((args.condition, args.scenario, args.seed, args.gui), ("people", "standing", 43, True))
        self.assertEqual(groups.result_fields(args, .1)["response_variant"], "bounded_reactive_standing_v3")
        shared = PROFILES["shared_nav2_pilot"]
        plan = json.loads((BENCH / shared.name / "protocol.json").read_text())
        for model, dt in [("crowd_orca_plus", .1), ("complex_geometry_mixed", .25)]:
            args = shared.parse_args(["--model", model, "--report-dir", "/tmp/fixed"])
            self.assertEqual(shared.planning_dt(plan, args), dt)
            self.assertEqual(args.condition, "people")

    def test_worker_commands_preserve_isolated_native_runtimes(self):
        social = Path("/fixture/experiments/social_navigation")
        runtime = Path("/fixture/runtime")
        for name, flags in [("socnavgym_nav2_pilot", ["--condition", "people"]),
                            ("socnavgym_group_nav2", ["--scenario", "walking", "--seed", "42"])]:
            profile = PROFILES[name]
            args = profile.parse_args(flags + ["--report-dir", "/tmp/test"])
            command = profile.worker_command(social, runtime, "/fixture/worker.sock", args)
            self.assertEqual(command[0], str(social / ".venv-socnavgym/bin/python"))
            self.assertEqual(command[1], str(social / "benchmark" / name / "worker.py"))
            self.assertEqual(command[command.index("--socket") + 1], "/fixture/worker.sock")
            if profile.groups:
                self.assertEqual(command[-4:], ["--scenario", "walking", "--seed", "42"])
            else:
                self.assertEqual(command[-4:], ["--model", "soc_guarded_orca", "--case", "crossing_near"])

    def test_existing_prerequisite_gates_still_apply(self):
        for profile in PROFILES.values():
            profile.validate_gates(BENCH)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "socnavgym_nav2_pilot/reports/gate01/validation.json"
            path.parent.mkdir(parents=True)
            path.write_text('{"status": "FAIL"}')
            with self.assertRaisesRegex(RuntimeError, "Prerequisite gate failed"):
                PROFILES["socnavgym_nav2_pilot"].validate_gates(root)

    def test_source_manifest_includes_shared_implementation_and_launch(self):
        for profile in PROFILES.values():
            entry = BENCH / profile.name / "trial.py"
            sources = source_files(ROOT, profile, entry)
            self.assertIn(ROOT / "python/dynamuhsim/runtime/navigation.py", sources)
            self.assertIn(ROOT / "src/custom_corridor/launch/_dynamuhsim_nav2.py", sources)
            self.assertIn(ROOT / "src/custom_corridor/launch/_dynamuhsim_simulation.py", sources)
            self.assertIn(ROOT / "src/custom_corridor/launch/corridor_tb3.launch.py", sources)
            self.assertIn(ROOT / "python/dynamuhsim/runtime/nav2_trial.py", sources)
            self.assertIn(ROOT / "python/dynamuhsim/analysis/nav2.py", sources)
            self.assertIn(ROOT / "python/dynamuhsim/simulation/warehouse.py", sources)
            self.assertIn(BENCH / "common_gazebo/worker.py", sources)
            self.assertIn(entry, sources)
            self.assertIn(entry.parent / "warehouse.launch.py", sources)


class ConfigurationTests(unittest.TestCase):
    def test_world_bytes_match_legacy_outputs_for_each_profile(self):
        contract = json.loads((FIXTURES / "contract.json").read_text())
        for name, spec in contract["worlds"].items():
            with self.subTest(name=name):
                tree = ET.parse(FIXTURES / "world_input.sdf")
                profile = PROFILES[spec["profile"]]
                configure_world(tree, Path("/fixture/build/libresponse_adapter.so"), spec["condition"], spec["dt"],
                                planning_dt_text=None if profile.container_worker else ".1")
                ET.indent(tree)
                self.assertEqual(ET.tostring(tree.getroot(), encoding="unicode"), (FIXTURES / (name + ".sdf")).read_text())

    def test_nav2_config_changes_only_original_initial_pose_fields(self):
        initial = {"amcl": {"ros__parameters": {"initial_pose": {"x": 0., "y": 2., "yaw": 1., "z": 0.}, "extra": 7}},
                   "controller_server": {"ros__parameters": {"vx_max": .6, "vy_max": .4}}}
        before = copy.deepcopy(initial)
        actual = runtime_nav2_config(initial)
        expected = copy.deepcopy(before)
        expected["amcl"]["ros__parameters"]["initial_pose"].update(x=-28., y=0., yaw=0.)
        self.assertEqual(actual, expected)
        self.assertEqual(initial, before)

    def test_launch_keeps_original_topics_arguments_and_startup_delay(self):
        class LaunchItem:
            def __init__(self, *args, **kwargs):
                self.args = args
                self.kwargs = kwargs
        modules = {
            "launch": NS(LaunchDescription=LaunchItem),
            "launch.actions": NS(DeclareLaunchArgument=LaunchItem, IncludeLaunchDescription=LaunchItem, TimerAction=LaunchItem),
            "launch.launch_description_sources": NS(PythonLaunchDescriptionSource=LaunchItem),
            "launch.substitutions": NS(LaunchConfiguration=LaunchItem),
            "launch_ros.actions": NS(Node=LaunchItem),
            "ament_index_python.packages": NS(get_package_share_directory=lambda name: "/fixture/share/" + name),
        }
        with patch.dict(sys.modules, modules), patch.dict("os.environ", NAV2_PILOT_WORLD="/fixture/world.sdf", NAV2_PILOT_PARAMS="/fixture/nav2.yaml"):
            launch = generate_launch_description(BENCH / "socnavgym_nav2_pilot/warehouse.launch.py")
        declare, sim, bridge, timer = launch.args[0]
        args = dict(sim.kwargs["launch_arguments"])
        self.assertEqual(args["custom_world"], "/fixture/world.sdf")
        self.assertEqual((args["robot_model"], args["obstacle"], args["x"], args["y"], args["yaw"]), ("mecanum", "human", "-28", "0", "0"))
        self.assertEqual(timer.kwargs["period"], 6.)
        self.assertEqual(bridge.kwargs["arguments"], ["/common/ground_truth@std_msgs/msg/String[gz.msgs.StringMsg"])
        nav = dict(timer.kwargs["actions"][0].kwargs["launch_arguments"])
        self.assertEqual(nav["controller"], "mppi")
        self.assertEqual(nav["params_file"], "/fixture/nav2.yaml")


class TelemetryTests(unittest.TestCase):
    def test_goal_audit_uses_recorded_sample_before_teardown_tail(self):
        from dynamuhsim.analysis.nav2 import goal_sample
        for distance in (.12, .21):
            point = dict(x=-20.-distance, y=0., yaw=.1)
            result = dict(final_world_pose=point, action_status=4, goal_xy_error_world_m=abs(point["x"]+20.),
                          goal_yaw_error_world_rad=.1, actual_goal_reached=distance <= .20)
            complete = [dict(time=1., phase="active", poses={"mecanum_robot": point}),
                        dict(time=2., phase="finished", poses={"mecanum_robot": dict(x=-19.4, y=0., yaw=.4)})]
            final, _, _, reached = goal_sample(complete, result, [-20., 0., 0.])
            self.assertEqual(final['time'], 1.)
            self.assertEqual(reached, distance <= .20)
            with self.assertRaisesRegex(AssertionError, "Recorded goal pose not found"):
                goal_sample(complete[1:], result, [-20., 0., 0.])

    def test_goal_audit_keeps_action_distance_and_heading_as_separate_requirements(self):
        from dynamuhsim.analysis.nav2 import goal_sample
        for status, x, yaw, expected in [(4,.20,.25,True),(4,.200001,0.,False),
                                        (4,0.,.250001,False),(6,.1,0.,False),(5,.1,0.,False)]:
            with self.subTest(status=status,x=x,yaw=yaw):
                pose=dict(x=x,y=0.,yaw=yaw)
                result=dict(final_world_pose=pose,action_status=status,goal_xy_error_world_m=x,
                            goal_yaw_error_world_rad=yaw,actual_goal_reached=expected)
                rows=[dict(time=1.,phase='active',poses={'mecanum_robot':pose})]
                self.assertEqual(goal_sample(rows,result,[0.,0.,0.])[-1],expected)
                result['actual_goal_reached']=not expected
                with self.assertRaises(AssertionError):goal_sample(rows,result,[0.,0.,0.])

    def test_worker_statistics_preserve_backend_specific_fields(self):
        from dynamuhsim.analysis.nav2 import worker_statistics
        moving=[dict(time=1.1,motion_guard=dict(projection=dict(margin_m=.02,velocity_change_norm=.2))),
                dict(time=1.2,motion_guard=dict(projection=dict(margin_m=.002,velocity_change_norm=0.)))]
        common=dict(inactive_transport_steps=1,active_native_steps=2,first_active_sim=1.1)
        for profile, dt in [('individual',.1),('container',.25),('groups',.1)]:
            with self.subTest(profile=profile):
                stats=worker_statistics(moving,[{}],dict(goal_start_sim=1.),profile,dt)
                self.assertAlmostEqual(stats.pop('start_gate_delay_s'),.1)
                expected=dict(common,**(dict(margins_used_m=[.002,.02],changed_proposals=1)
                    if profile=='individual' else dict(native_dt=dt,guard_present_steps=2)))
                self.assertEqual(stats,expected)

    def test_sensor_rows_and_jsonl_match_legacy_callbacks(self):
        fixture = json.loads((FIXTURES / "telemetry.json").read_text())
        recording = TrialRecording()
        recording.phase = "active"
        for row in fixture["messages"]:
            msg = message(row["message"])
            if row["key"] == "scan":
                msg.ranges = [1., 2., float("nan"), float("inf")]
            recording.receive(row["key"], msg)
        recording.scene(message(fixture["scene"]))
        recording.feedback(message(fixture["feedback"]))
        self.assertEqual(recording.rows, fixture["expected"])
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assertIsNone(recording.write(root))
            self.assertEqual({p.stem for p in root.glob("*.jsonl")}, set(recording.STREAMS))
            for name, rows in fixture["expected"].items():
                parsed = [json.loads(line) for line in (root / (name + ".jsonl")).read_text().splitlines()]
                self.assertEqual(parsed, rows)

    def test_worker_fault_is_terminal_and_preserved(self):
        recording = TrialRecording()
        fault = {"adapter_ok": False, "error": "socket lost"}
        with self.assertRaisesRegex(RuntimeError, "Terminal adapter fault: socket lost"):
            recording.receive("gt", NS(data=json.dumps(fault)))
        self.assertEqual(recording.faults, [fault])
        self.assertEqual(recording.rows["ground_truth"], [])
        self.assertEqual(recording.counts["gt"], 1)

    def test_observer_frames_are_only_collected_during_active_phase(self):
        recording = TrialRecording()
        frame = NS(header=NS(stamp=NS(sec=2, nsec=0)), width=1, height=1, step=3, data=b"abc")
        for phase in ["warmup", "active", "finished"]:
            recording.phase = phase
            recording.camera(frame)
        self.assertEqual(recording.frames, [(2., 1, 1, 3, b"abc")])


class NavigationTests(unittest.TestCase):
    def exercise(self, *, completed=True, accepted=True, budget=45., wall_timeout=False, fail_wait=False):
        clock = NS(sim=0., wall=0.)
        recorder = TrialRecording()
        recorder.simtime = lambda: clock.sim
        pending = NS(done=lambda: completed, result=lambda: NS(status=4, result=NS(error_code=0, error_msg="")))
        cancel_calls = []
        def cancel():
            cancel_calls.append(True)
            return NS(done=lambda: True)
        handle = NS(accepted=accepted, get_result_async=lambda: pending, cancel_goal_async=cancel)
        client = NS(send_goal_async=lambda goal, feedback_callback: NS(done=lambda: True, result=lambda: handle))
        waits = []
        def wait(predicate, seconds):
            waits.append(seconds)
            if fail_wait and seconds == .04:
                raise RuntimeError("Owned launch or worker exited")
            clock.sim += 0. if wall_timeout else seconds
            clock.wall += 300. if wall_timeout and seconds == .04 else seconds
            return predicate()
        accepted_handles = []
        with tempfile.TemporaryDirectory() as directory:
            result = {"sim_budget_s": budget}
            if not accepted or fail_wait:
                with self.assertRaisesRegex(RuntimeError, "Goal rejected" if not accepted else "Owned launch"):
                    execute_goal(client, object(), recorder, Path(directory), result, wait, lambda: None,
                                 on_accepted=accepted_handles.append, monotonic=lambda: clock.wall)
            else:
                execute_goal(client, object(), recorder, Path(directory), result, wait, lambda: None,
                             on_accepted=accepted_handles.append, monotonic=lambda: clock.wall)
            gate = Path(directory) / "start_gate"
            self.assertEqual(gate.exists(), accepted)
        return result, cancel_calls, accepted_handles

    def test_success_action_status(self):
        result, cancels, handles = self.exercise()
        self.assertEqual(result["navigation_status"], "SUCCEEDED")
        self.assertEqual(result["action_status"], 4)
        self.assertEqual(cancels, [])
        self.assertEqual(len(handles), 1)

    def test_simulation_timeout_cancels_and_preserves_timeout(self):
        result, cancels, handles = self.exercise(completed=False, budget=.1)
        self.assertEqual(result["navigation_status"], "TIMEOUT")
        self.assertIsNone(result["action_status"])
        self.assertEqual(cancels, [True])

    def test_paused_simulation_wall_timeout_still_cancels(self):
        result, cancels, handles = self.exercise(completed=False, wall_timeout=True)
        self.assertEqual(result["navigation_status"], "TIMEOUT")
        self.assertEqual(cancels, [True])

    def test_rejected_goal_never_opens_start_gate(self):
        result, cancels, handles = self.exercise(accepted=False)
        self.assertEqual(handles, [])

    def test_accepted_handle_survives_fault_for_outer_cleanup(self):
        result, cancels, handles = self.exercise(completed=False, fail_wait=True)
        self.assertEqual(len(handles), 1)

    def test_action_success_does_not_override_actual_goal_failure(self):
        self.assertFalse(goal_metrics(dict(x=-19.7941, y=0., yaw=0.), 4)["actual_goal_reached"])
        self.assertTrue(goal_metrics(dict(x=-20.1442, y=0., yaw=.0685), 4)["actual_goal_reached"])
        self.assertFalse(goal_metrics(dict(x=-20., y=0., yaw=.251), 4)["actual_goal_reached"])
        self.assertFalse(goal_metrics(dict(x=-20., y=0., yaw=0.), None)["actual_goal_reached"])

    def test_exited_owned_process_aborts_wait(self):
        with self.assertRaisesRegex(RuntimeError, "Owned launch or worker exited"):
            wait_for(lambda: True, 10, spin_once=lambda: None, processes=[NS(poll=lambda: 1)])


class LaunchShutdownTests(unittest.TestCase):
    def test_only_launch_parent_gets_sigint_during_graceful_shutdown(self):
        events = []
        process = NS(returncode=None, poll=lambda: None)
        process.send_signal = lambda sig: events.append(("parent_signal", sig))
        def wait(timeout):
            events.append(("wait", timeout))
            process.returncode = 0
        process.wait = wait
        result = stop_ros_launch(process, lambda proc: events.append(("group_stop", proc)))
        self.assertEqual(events, [("parent_signal", signal.SIGINT), ("wait", 15)])
        self.assertEqual((result["status"], result["returncode"]), ("STOPPED", 0))

    def test_hung_launch_uses_group_fallback_after_grace_period(self):
        events = []
        process = NS(returncode=None, poll=lambda: None)
        process.send_signal = lambda sig: events.append("parent_signal")
        def wait(timeout):
            events.append("wait_expired")
            raise subprocess.TimeoutExpired("ros2 launch", timeout)
        def fallback(proc):
            self.assertIs(proc, process)
            events.append("group_stop")
            proc.returncode = -15
        process.wait = wait
        result = stop_ros_launch(process, fallback)
        self.assertEqual(events, ["parent_signal", "wait_expired", "group_stop"])
        self.assertEqual(result["status"], "ESCALATED")

    def test_exited_launch_gets_no_second_signal(self):
        process = NS(returncode=0, poll=lambda: 0)
        result = stop_ros_launch(process, lambda proc: self.fail("already exited"))
        self.assertEqual(result, {"status": "ALREADY_EXITED", "returncode": 0})


class EntryPointTests(unittest.TestCase):
    def test_legacy_entrypoints_help_without_ros(self):
        for name in PROFILES:
            path = BENCH / name / "trial.py"
            result = subprocess.run([sys.executable, str(path), "--help"], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("--report-dir", result.stdout)

    def test_common_cli_lists_profiles_and_forwards_help(self):
        for args in [["--help"], ["--experiment", "socnavgym_group_nav2", "--help"]]:
            result = subprocess.run([sys.executable, str(ROOT / "tools/nav2_trial.py"), *args], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("socnavgym_group_nav2", result.stdout)


if __name__ == "__main__":
    unittest.main()
