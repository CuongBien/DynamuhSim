"""Capture migration contracts: accepted camera/config, source freeze, raw precision and runner wiring."""
import ast
import copy
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
from dynamuhsim.runtime import capture
from dynamuhsim.runtime.capture_recording import Recording, Pairs, TOPICS
from dynamuhsim.simulation.capture import prepare_camera, apply_nav2_treatment, config_changes
from test_nav2_runner_integration import RunnerHarness


class FixtureTests(unittest.TestCase):
    def test_native_camera_matches_frozen_accepted_bytes_and_source_stays_unchanged(self):
        source = ROOT / "src/custom_corridor/models/mecanum_robot/model.sdf"
        before = source.read_bytes()
        with tempfile.TemporaryDirectory() as out:
            result = prepare_camera(out, ROOT)
            self.assertEqual(result["runtime_sha256"], capture.CAMERA_SHA256)
            self.assertEqual(hashlib.sha256((Path(out) / "mecanum_camera.sdf").read_bytes()).hexdigest(), capture.CAMERA_SHA256)
            self.assertEqual(source.read_bytes(), before)

    def test_legacy_camera_launch_carries_its_helper_and_source_hash(self):
        from dynamuhsim.runtime.nav2_trial import load_file_module
        legacy = load_file_module("_legacy_camera_copy_test", ROOT / "experiments/social_navigation/benchmark/dataset_pilot/prepare.py")
        helper = ROOT / "src/custom_corridor/launch/_dynamuhsim_simulation.py"
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            result = legacy.prepare_camera(out, ROOT)
            self.assertEqual((out / helper.name).read_bytes(), helper.read_bytes())
            digest = hashlib.sha256(helper.read_bytes()).hexdigest()
            self.assertEqual(result[helper.name]["runtime_sha256"], digest)
            self.assertEqual(result[helper.name]["original_sha256"], digest)
            self.assertNotIn("        camera_link_tf,\n", (out / "mecanum_camera.launch.py").read_text())
            self.assertIn(helper, capture.capture_sources(ROOT))

    def test_shared_camera_frames_preserve_mount_ownership_and_clock_contract(self):
        from dynamuhsim.runtime.nav2_trial import load_file_module
        actions = NS(Node=lambda **kwargs: kwargs)
        with patch.dict(sys.modules, {"launch_ros": NS(actions=actions), "launch_ros.actions": actions}):
            helper = load_file_module("_camera_frame_contract_test", ROOT / "src/custom_corridor/launch/_dynamuhsim_simulation.py")
        legacy = helper.camera_tf_nodes()
        native = helper.camera_tf_nodes(mount_tf=False, zero="0", optical_sim_time=True)
        self.assertEqual([n["name"] for n in legacy], ["camera_link_tf", "camera_optical_tf", "camera_imu_tf"])
        self.assertEqual([n["name"] for n in native], ["camera_optical_tf", "camera_imu_tf"])
        self.assertNotIn("parameters", legacy[0])
        self.assertNotIn("parameters", legacy[1])
        self.assertEqual(legacy[2]["parameters"], [{"use_sim_time": True}])
        self.assertTrue(all(n["parameters"] == [{"use_sim_time": True}] for n in native))
        self.assertEqual(legacy[0]["arguments"][:6], ["--x", "0.08", "--y", "0.0", "--z", "0.20"])
        self.assertTrue(all(n["arguments"][-1] != "camera_link" for n in native))

    def test_nav2_treatment_matches_legacy_config_without_mutating_input(self):
        import yaml
        base = yaml.safe_load((ROOT / "experiments/social_navigation/warehouse/nav2_warehouse.yaml").read_text())
        before = copy.deepcopy(base)
        source = ROOT / "experiments/social_navigation/benchmark/early_avoidance_pilot/config_runtime.py"
        namespace = {}
        tree = ast.parse(source.read_text())
        function = next(x for x in tree.body if isinstance(x, ast.FunctionDef) and x.name == "apply_runtime_config")
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), "exec"), namespace)
        expected = namespace["apply_runtime_config"](copy.deepcopy(base))
        expected["controller_server"]["ros__parameters"]["FollowPath"].update(vx_max=.30, vx_min=-.15, vy_max=.20)
        config = json.loads((ROOT / "configs/capture/moderate.json").read_text())
        actual = apply_nav2_treatment(base, config["nav2"])
        self.assertEqual(actual, expected)
        self.assertEqual(base, before)
        self.assertIn("/local_costmap/local_costmap/ros__parameters/footprint", [x["path"] for x in config_changes(before, actual)])

    def test_v2_requires_simultaneous_precise_goal_without_disabling_detour_following(self):
        import yaml
        config = json.loads((ROOT / "configs/capture/moderate_v2.json").read_text())
        base = yaml.safe_load((ROOT / "experiments/social_navigation/warehouse/nav2_warehouse.yaml").read_text())
        before = copy.deepcopy(base)
        v1 = apply_nav2_treatment(base, config["nav2"])
        v2 = apply_nav2_treatment(base, config["nav2"], navigation=config["navigation"])
        expected = {"/controller_server/ros__parameters/goal_checker/" + key for key in ["stateful", "xy_goal_tolerance", "yaw_goal_tolerance"]}
        expected |= {"/controller_server/ros__parameters/FollowPath/" + name + "/threshold_to_consider" for name in ["GoalCritic", "PathFollowCritic"]}
        self.assertEqual({c["path"] for c in config_changes(v1, v2)}, expected)
        goal = v2["controller_server"]["ros__parameters"]["goal_checker"]
        self.assertFalse(goal["stateful"])
        self.assertLess(goal["xy_goal_tolerance"], .20)
        self.assertLess(goal["yaw_goal_tolerance"], .25)
        follow = v2["controller_server"]["ros__parameters"]["FollowPath"]
        horizon = follow["vx_max"] * follow["time_steps"] * follow["model_dt"]
        for name in ["GoalCritic", "PathFollowCritic"]:
            radius = follow[name]["threshold_to_consider"]
            self.assertAlmostEqual(radius, horizon)
            self.assertLess(radius, 1.55)  # The recorded 11.49m detour at XY1.55m stays in path mode.
        self.assertEqual(base, before)

    def test_v2_rejects_loosening_or_unrecorded_navigation_treatment(self):
        config = json.loads((ROOT / "configs/capture/moderate_v2.json").read_text())
        self.assertEqual(capture.validate_config(config), config)
        for mutate in [lambda n: n["goal_checker"].update(stateful=True),
                       lambda n: n["goal_checker"].update(stateful=0),
                       lambda n: n["goal_checker"].update(xy_goal_tolerance=.3),
                       lambda n: n.update(goal_critic_distance_m=2.5)]:
            changed = copy.deepcopy(config); mutate(changed["navigation"])
            with self.assertRaises(ValueError): capture.validate_config(changed)

    def test_config_rejects_oracle_or_silent_camera_or_controller_changes(self):
        config = json.loads((ROOT / "configs/capture/moderate.json").read_text())
        self.assertEqual(capture.validate_config(config), config)
        for mutation in [lambda x: x.update(oracle=True), lambda x: x.update(camera_sha256="0" * 64),
                         lambda x: x.update(label_tail_s=0), lambda x: x.update(sim_budget_s=float("nan")),
                         lambda x: x["nav2"]["speed"].update(vx_max=1), lambda x: x.update(allowed_seeds=[True])]:
            changed = copy.deepcopy(config)
            mutation(changed)
            with self.assertRaises(ValueError): capture.validate_config(changed)

    def test_new_entrypoint_help_requires_no_ros(self):
        for flags in [["--help"], ["prepare", "--help"], ["run", "--help"]]:
            proc = subprocess.run([sys.executable, str(ROOT / "tools/capture_episode.py"), *flags], capture_output=True, text=True)
            self.assertEqual(proc.returncode, 0, proc.stderr)

    def test_active_capture_modules_do_not_execute_or_patch_report_source(self):
        paths = [ROOT / "python/dynamuhsim/runtime/capture.py", ROOT / "python/dynamuhsim/simulation/capture.py",
                 ROOT / "python/dynamuhsim/simulation/capture_launch.py", ROOT / "tools/capture_episode.py"]
        for path in paths:
            tree = ast.parse(path.read_text())
            calls = [x for x in ast.walk(tree) if isinstance(x, ast.Call)]
            self.assertFalse(any(isinstance(x.func, ast.Name) and x.func.id in ["exec", "eval", "compile"] for x in calls), path)
            self.assertFalse(any(isinstance(x.func, ast.Attribute) and x.func.attr == "replace" for x in calls), path)
            self.assertNotIn("reports/implementation", path.read_text())


class FreezeTests(unittest.TestCase):
    def prepare_fixture(self, root):
        run = root / "new_run"
        run.mkdir()
        (run / "assets").mkdir()
        for name in ["plan.json", "preflight.json", "assets/mecanum_camera.sdf"]: (run / name).write_text("{}")
        source = root / "source.py"
        source.write_text("# pinned\n")
        files = [source, run / "plan.json", run / "preflight.json", run / "assets/mecanum_camera.sdf"]
        freeze = dict(schema="dynamuhsim_capture_freeze/2", source_sha256={str(p.relative_to(root)): capture.sha(p) for p in files})
        capture.dump(run / "capture_freeze.json", freeze)
        return run, source, freeze

    def test_freeze_detects_source_and_plan_mutations(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run, source, freeze = self.prepare_fixture(root)
            with patch.object(capture, "capture_sources", return_value=[source]):
                self.assertEqual(capture.verify_run(root, run), freeze)
                source.write_text("# changed\n")
                with self.assertRaisesRegex(RuntimeError, "changed"): capture.verify_run(root, run)
                source.write_text("# pinned\n")
                (run / "plan.json").write_text('{"seed":49}')
                with self.assertRaisesRegex(RuntimeError, "changed"): capture.verify_run(root, run)

    def test_missing_current_source_or_asset_cannot_pass_a_partial_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run, source, freeze = self.prepare_fixture(root)
            with patch.object(capture, "capture_sources", return_value=[source, root / "new_module.py"]):
                with self.assertRaisesRegex(RuntimeError, "Required source"): capture.verify_run(root, run)
            del freeze["source_sha256"][str((run / "assets/mecanum_camera.sdf").relative_to(root))]
            capture.dump(run / "capture_freeze.json", freeze)
            with patch.object(capture, "capture_sources", return_value=[source]):
                with self.assertRaisesRegex(RuntimeError, "Required run input"): capture.verify_run(root, run)

    def test_historical_manifest_schema_is_not_rewritten(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run, source, freeze = self.prepare_fixture(root)
            path = run / "capture_freeze.json"
            capture.dump(path, dict(schema="wm_capture_freeze/1"))
            before = path.read_bytes()
            with self.assertRaises(ValueError): capture.verify_run(root, run)
            self.assertEqual(path.read_bytes(), before)


class RecordingTests(unittest.TestCase):
    def test_recorder_process_exit_is_terminal_and_bad_close_is_rejected(self):
        context = capture.CaptureContext.__new__(capture.CaptureContext)
        context.recorder = NS(process=NS(poll=lambda: 1), close=lambda: dict(returncode=1, metadata_present=False))
        context.pairs = None
        with self.assertRaisesRegex(RuntimeError, "Recorder exited"): context.check_health()
        result = {}
        with self.assertRaisesRegex(RuntimeError, "MCAP recorder"): context.close(result)
        self.assertEqual(result["recording"]["returncode"], 1)

    def test_event_and_label_scene_schema_are_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = Recording.__new__(Recording)
            recorder.output, recorder.seq = Path(directory), 0
            recorder.message_type = lambda **values: NS(**values)
            events, scenes = [], []
            recorder.event_pub, recorder.scene_pub = NS(publish=events.append), NS(publish=scenes.append)
            recorder.event("goal_accepted", 123, goal_world=[-20., 0., 0.])
            self.assertEqual(json.loads(events[0].data), dict(schema="social_episode/1", seq=1, stamp_ns=123, event="goal_accepted", goal_world=[-20., 0., 0.]))
            entities = dict(mecanum_robot=dict(x=-28, y=0, yaw=0), common_6_proxy=dict(x=2, y=3, yaw=0), common_2_proxy=dict(x=4, y=5, yaw=0))
            recorder.scene(456, entities, "label_tail")
            row = json.loads(scenes[0].data)
            self.assertEqual(row["schema"], "social_scene/1")
            self.assertEqual([x["id"] for x in row["humans"]], [2, 6])
            self.assertEqual(row["phase"], "label_tail")
            self.assertFalse(row["persona_valid"])
            self.assertIn("/common/ground_truth", TOPICS)
            self.assertIn("/dataset/scene_poses", TOPICS)

    def test_paired_depth_keeps_float32_bits_padding_and_duplicate_stamp_policy(self):
        for big in [False, True]:
            with self.subTest(big=big), tempfile.TemporaryDirectory() as directory:
                pairs = Pairs.__new__(Pairs)
                pairs.output, pairs.phase = Path(directory), lambda: "active"
                pairs.rows = {"on": {}, "off": {}}
                (pairs.output / "off").mkdir()
                array = np.arange(240 * 426, dtype=np.float32).reshape(240, 426)
                array[0, :3] = [float("nan"), float("inf"), -1.]
                payload = array.astype(">f4" if big else "<f4").tobytes()
                msg = NS(encoding="32FC1", height=240, width=424, step=426*4, is_bigendian=big, data=payload,
                         header=NS(frame_id="camera_optical_frame", stamp=NS(sec=1, nanosec=2)))
                pairs.receive("off", msg)
                pairs.receive("off", msg)
                saved = np.load(pairs.output / "off/1000000002.npy")
                np.testing.assert_array_equal(saved.view(np.uint32), array[:, :424].copy().view(np.uint32))
                self.assertEqual(len(pairs.rows["off"]), 1)


class FakeCapture:
    def __init__(self, harness):
        self.h = harness
        self.config = dict(sim_budget_s=7.)
        self.runtime_dir = harness.stage / "generated/capture_api_run"
        harness.runtime = self.runtime_dir
    def source_paths(self): return []
    def mark(self, name): self.h.events.append("capture:" + name)
    def prepare(self, runtime, result): self.mark("prepare")
    def configure_world(self, tree): self.mark("world")
    def configure_nav(self, config, output): self.mark("config"); return config
    def attach(self, node, output, telemetry): self.mark("attach")
    def start_bridge(self, runtime, output, env, processes, files): self.mark("bridge")
    def ready(self, node, wait, output, result, telemetry): self.mark("ready")
    def check_health(self): pass
    def started(self, start): self.mark("accepted")
    def finish(self, wait, telemetry, result): self.mark("tail")
    def close(self, result): self.mark("close")
    def stop_server(self, guard, original): self.mark("quiesce"); return original(guard)


class RunnerCaptureTests(unittest.TestCase):
    def test_capture_uses_shared_goal_runner_and_closes_recorder_before_server(self):
        with tempfile.TemporaryDirectory() as directory:
            h = RunnerHarness(Path(directory) / "repo", "socnavgym_group_nav2", ["--scenario", "standing", "--seed", "42"])
            code, result = h.run(FakeCapture(h))
            self.assertEqual(code, 0)
            self.assertEqual(result["sim_budget_s"], 7.)
            self.assertEqual(Path(result["runtime"]).name, "capture_api_run")
            self.assertIn("capture_warehouse.launch.py", h.processes[-1].command[2])
            events = [x for x in h.events if x.startswith("capture:")]
            self.assertEqual(events, ["capture:prepare", "capture:world", "capture:config", "capture:attach", "capture:bridge", "capture:ready", "capture:accepted", "capture:tail", "capture:close", "capture:quiesce"])
            self.assertLess(h.events.index("capture:close"), h.events.index("server_stop"))

    def test_fault_after_acceptance_closes_capture_without_manufacturing_tail(self):
        with tempfile.TemporaryDirectory() as directory:
            h = RunnerHarness(Path(directory) / "repo", "socnavgym_group_nav2", ["--scenario", "standing", "--seed", "42"], fault=True)
            code, result = h.run(FakeCapture(h))
            self.assertEqual(code, 1)
            self.assertEqual(h.canceled, 1)
            self.assertNotIn("capture:tail", h.events)
            self.assertIn("capture:close", h.events)
            self.assertLess(h.events.index("cancel"), h.events.index("capture:close"))


if __name__ == "__main__": unittest.main()
