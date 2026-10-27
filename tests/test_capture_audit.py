import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
from dynamuhsim.analysis.capture import audit_capture
from dynamuhsim.analysis.evidence import dump_json, sha256_file


class CaptureAuditTests(unittest.TestCase):
    def fixture(self, directory):
        root = Path(directory)
        run = root / "run"
        out = run / "episode"
        for name in ["runtime", "episode/paired_depth/on", "episode/paired_depth/off", "episode/raw_bag"]: (run / name).mkdir(parents=True, exist_ok=True)
        (run / "runtime/mecanum_camera.sdf").write_bytes(b"camera")
        (root / "producer.py").write_bytes(b"source")
        dump_json(run / "capture_freeze.json", {})
        dump_json(run / "preflight.json", dict(native_friction=dict(recognized_gz_base_link_frames=4)))
        (out / "launch.log").write_text("teardown pre=153 post=153\n")
        (out / "raw_bag/test.mcap").write_bytes(b"fixture")
        events = [dict(event=name, stamp_ns=stamp) for name, stamp in [("ready", 0), ("goal_accepted", 0), ("action_end", 1_000_000_000), ("observation_end", 5_000_000_000)]]
        (out / "events.jsonl").write_text("".join(json.dumps(x) + "\n" for x in events))
        poses = []
        for t, x in [(0, -28.), (.5, -25.), (1., -20.1)]:
            entities = dict(mecanum_robot=dict(x=x, y=0., yaw=0.))
            entities.update({f"common_{i}_proxy": dict(x=-26+i, y=2., yaw=0.) for i in range(1, 7)})
            poses.append(dict(time=t, poses=entities))
        (out / "gazebo_poses.jsonl").write_text("".join(json.dumps(x) + "\n" for x in poses))
        (out / "odom.jsonl").write_text("".join(json.dumps(dict(time=x["time"], x=x["poses"]["mecanum_robot"]["x"]+28, y=0, yaw=0)) + "\n" for x in poses))
        result = dict(capture_schema="dynamuhsim_capture/2", status="MEASURED", rendering=dict(hardware_verified=True),
            cleanup=dict(status="PASS", remaining=[], ogre_renderer_lines=["NVIDIA"]), gazebo_clean_exit=True,
            server_shutdown=dict(status="STOPPED", quiesce=dict(pause_acknowledged=True, simulation_time_stable=True)),
            ros_launch_shutdown=dict(status="STOPPED", returncode=0), faults=[], recording=dict(returncode=0, metadata_present=True),
            source_sha256={"producer.py": sha256_file(root / "producer.py")}, head="0"*40,
            capture_freeze_sha256=sha256_file(run / "capture_freeze.json"),
            live_parameters={"/controller_server": {"FollowPath.vx_max": .3, "FollowPath.vx_min": -.15, "FollowPath.vy_max": .2, "FollowPath.CostCritic.consider_footprint": False}},
            final_world_pose=poses[-1]["poses"]["mecanum_robot"], action_status=None, navigation_status="TIMEOUT",
            actual_goal_reached=False, goal_xy_error_world_m=abs(-20.1+20), goal_yaw_error_world_rad=0., active_sim_seconds=1.)
        dump_json(out / "result.json", result)
        policy = json.loads((ROOT / "configs/audit/capture_integrity.json").read_text())
        # Small synthetic message fixture; default production policy remains unchanged.
        policy.update(camera_sha256=sha256_file(run / "runtime/mecanum_camera.sdf"), minimum_paired_frames_exclusive=0,
                      minimum_future_depth_frames=1, depth_shape=[1, 1])
        stamps = [2_000_000_000, 2_100_000_000, 2_200_000_000]
        dump_json(out / "paired_depth/index.json", {side: {str(stamp): {} for stamp in stamps} for side in ["on", "off"]})
        image = np.array([[1.]], dtype=np.float32)
        for side in ["on", "off"]:
            for stamp in stamps: np.save(out / "paired_depth" / side / (str(stamp)+".npy"), image, allow_pickle=False)
        selected = {"/camera/depth/image_rect_raw", "/dataset/events", "/dataset/scene_poses", "/tf_static"}
        def messages(path):
            for topic in policy["required_topics"]:
                if topic not in selected: yield topic, None
            for stamp in stamps:
                yield "/camera/depth/image_rect_raw", NS(encoding="32FC1", height=1, width=1, step=4, is_bigendian=False, data=image.tobytes(), header=NS(frame_id="camera_optical_frame", stamp=NS(sec=stamp//1_000_000_000, nanosec=stamp%1_000_000_000)))
            for row in events: yield "/dataset/events", NS(data=json.dumps(row))
            yield "/dataset/scene_poses", NS(data=json.dumps(dict(schema="social_scene/1", persona_valid=False, humans=[{}]*6)))
            yield "/tf_static", NS(transforms=[NS(child_frame_id="camera_link", header=NS(frame_id="base_link")), NS(child_frame_id="camera_optical_frame", header=NS(frame_id="camera_link"))])
        return root, run, policy, messages

    def test_v2_audit_requires_the_declared_live_goal_and_critic_parameters(self):
        with tempfile.TemporaryDirectory() as directory:
            root, run, policy, messages = self.fixture(directory)
            config = json.loads((ROOT / "configs/capture/moderate_v2.json").read_text())
            dump_json(run / "plan.json", dict(config=config))
            result = json.loads((run / "episode/result.json").read_text())
            result["capture_recipe_schema"] = config["schema"]
            dump_json(run / "episode/result.json", result)
            audit = audit_capture(run, policy, source_roots=[root], read_bag=messages)
            self.assertFalse(audit["checks"]["live_moderate_parameters"])
            controller = result["live_parameters"]["/controller_server"]
            controller.update({"goal_checker.stateful": False, "goal_checker.xy_goal_tolerance": .05, "goal_checker.yaw_goal_tolerance": .15,
                               "FollowPath.GoalCritic.threshold_to_consider": .45, "FollowPath.PathFollowCritic.threshold_to_consider": .45})
            dump_json(run / "episode/result.json", result)
            audit = audit_capture(run, policy, source_roots=[root], read_bag=messages)
            self.assertTrue(audit["checks"]["live_moderate_parameters"])
            self.assertTrue(audit["checks"]["declared_navigation_recipe"])
            controller["goal_checker.stateful"] = True; dump_json(run / "episode/result.json", result)
            self.assertFalse(audit_capture(run, policy, source_roots=[root], read_bag=messages)["checks"]["live_moderate_parameters"])

    def test_integrity_pass_does_not_turn_timeout_into_success(self):
        with tempfile.TemporaryDirectory() as directory:
            root, run, policy, messages = self.fixture(directory)
            before = (run / "episode/result.json").read_bytes()
            integrity = audit_capture(run, policy, source_roots=[root], read_bag=messages)
            goal_policy = copy.deepcopy(policy)
            goal_policy.update(name="navigation_goal_v1", require_actual_goal=True)
            goal = audit_capture(run, goal_policy, source_roots=[root], read_bag=messages)
            self.assertEqual(integrity["status"], "PASS")
            self.assertEqual(goal["status"], "FAIL")
            self.assertEqual([name for name, passed in goal["checks"].items() if not passed], ["actual_goal_required"])
            self.assertEqual(integrity["metrics"]["navigation_status"], "TIMEOUT")
            self.assertFalse(integrity["training_ready"])
            self.assertEqual((run / "episode/result.json").read_bytes(), before)

    def test_missing_on_payload_cannot_pass_from_index_alone(self):
        with tempfile.TemporaryDirectory() as directory:
            root, run, policy, messages = self.fixture(directory)
            (run / "episode/paired_depth/on/2000000000.npy").unlink()
            report = audit_capture(run, policy, source_roots=[root], read_bag=messages)
            self.assertEqual(report["status"], "FAIL")
            self.assertFalse(report["checks"]["paired_index_files_present"])

    def test_bad_depth_bits_and_missing_sources_do_not_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            root, run, policy, messages = self.fixture(directory)
            (root / "producer.py").write_bytes(b"changed")
            report = audit_capture(run, policy, source_roots=[root], read_bag=messages)
            self.assertFalse(report["checks"]["source_hashes"])
            np.save(run / "episode/paired_depth/off/2000000000.npy", np.array([[2.]], dtype=np.float32))
            with self.assertRaisesRegex(ValueError, "Raw depth differs"): audit_capture(run, policy, source_roots=[root], read_bag=messages)

    def test_default_policies_differ_only_by_the_actual_goal_gate(self):
        integrity = json.loads((ROOT / "configs/audit/capture_integrity.json").read_text())
        goal = json.loads((ROOT / "configs/audit/navigation_goal.json").read_text())
        self.assertFalse(integrity.pop("require_actual_goal"))
        self.assertTrue(goal.pop("require_actual_goal"))
        integrity.pop("name"); goal.pop("name")
        self.assertEqual(integrity, goal)

    def test_cli_help_does_not_require_ros_and_archived_raw_is_explicit(self):
        for tool in ["audit_episode.py", "artifact_store.py"]:
            process = subprocess.run([sys.executable, str(ROOT / "tools" / tool), "--help"], capture_output=True, text=True)
            self.assertEqual(process.returncode, 0, process.stderr)
        with tempfile.TemporaryDirectory() as directory:
            root, run, policy, messages = self.fixture(directory)
            dump_json(run / "artifact_ref.json", dict(object_id="0"*64))
            with self.assertRaisesRegex(FileNotFoundError, "restore"): audit_capture(run, policy, read_bag=messages)


if __name__ == "__main__": unittest.main()
