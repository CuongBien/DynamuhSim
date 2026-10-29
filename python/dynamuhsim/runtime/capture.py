"""Versioned capture hooks. New runs never execute report snapshots or rewrite source."""
from pathlib import Path, PurePosixPath
from dynamuhsim.workspace import stage_directory
import copy
import datetime
import json
import math
import shutil
import subprocess

from dynamuhsim.analysis.evidence import sha256_file
from .nav2_trial import source_files
from .profiles import PROFILES
from .shutdown import stop_paused
from dynamuhsim.simulation.capture import prepare_camera, apply_nav2_treatment, config_changes, nav2_parameter_contract

CAMERA_SHA256 = "14c15412664eb37dd2f0fb2d61a79b09f98722f204f97d97212f0ec814ad3d9f"


def sha(path):
    return sha256_file(path)


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def validate_config(config):
    expected = {"schema", "allowed_seeds", "sim_budget_s", "label_tail_s", "tail_wall_budget_s", "camera_sha256", "nav2"}
    version = config.get("schema")
    if version in ["dynamuhsim_capture_config/2", "dynamuhsim_capture_config/3", "dynamuhsim_capture_config/4", "dynamuhsim_capture_config/5", "dynamuhsim_capture_config/6", "dynamuhsim_capture_config/7"]:
        expected.add("navigation")
    if version in ["dynamuhsim_capture_config/4", "dynamuhsim_capture_config/5", "dynamuhsim_capture_config/6", "dynamuhsim_capture_config/7"]:
        expected.add("geometry_profile")
        if config.get("geometry_profile") != "native_sdf_urdf_v1":raise ValueError("Unknown geometry profile")
    if version in ["dynamuhsim_capture_config/5", "dynamuhsim_capture_config/6", "dynamuhsim_capture_config/7"]:
        expected.add("controller")
        declared_controller = {"cost_critic": {"cost_weight": 3.81, "near_collision_cost": 253}}
        if version in ["dynamuhsim_capture_config/6", "dynamuhsim_capture_config/7"]:declared_controller["time_steps"] = 100
        if version == "dynamuhsim_capture_config/7":
            declared_controller["regenerate_noises"] = True
            if type(config.get("controller", {}).get("regenerate_noises")) is not bool:raise ValueError("Noise regeneration must be boolean")
        if config.get("controller") != declared_controller:raise ValueError("Undeclared MPPI cost/horizon treatment")
    if set(config) != expected or version not in ["dynamuhsim_capture_config/1", "dynamuhsim_capture_config/2", "dynamuhsim_capture_config/3", "dynamuhsim_capture_config/4", "dynamuhsim_capture_config/5", "dynamuhsim_capture_config/6", "dynamuhsim_capture_config/7"]:
        raise ValueError("Unsupported capture config schema or keys")
    if config["camera_sha256"] != CAMERA_SHA256:
        raise ValueError("Native camera/friction contract cannot change in config version1")
    seeds = config["allowed_seeds"]
    if not isinstance(seeds, list) or not seeds or any(type(s) is not int or not 0 <= s < 2**31 for s in seeds) or len(set(seeds)) != len(seeds):
        raise ValueError("Expected distinct integer seeds")
    for key in ["sim_budget_s", "label_tail_s", "tail_wall_budget_s"]:
        if type(config[key]) not in [int, float] or not math.isfinite(config[key]) or config[key] <= 0:
            raise ValueError("Invalid capture budget: " + key)
    if config["sim_budget_s"] > 90 or config["label_tail_s"] < 3.2:
        raise ValueError("Capture version1 requires budget<=90s and label tail>=3.2s")
    nav = config["nav2"]
    if set(nav) != {"robot_radius", "footprint_padding", "inflation_radius", "cost_scaling_factor", "speed"}:
        raise ValueError("Unknown Nav2 treatment")
    wanted_radius = .17 if version in ["dynamuhsim_capture_config/4", "dynamuhsim_capture_config/5", "dynamuhsim_capture_config/6", "dynamuhsim_capture_config/7"] else .25
    wanted_inflation = nav.get("inflation_radius") if version in ["dynamuhsim_capture_config/4", "dynamuhsim_capture_config/5", "dynamuhsim_capture_config/6", "dynamuhsim_capture_config/7"] else .8
    if version in ["dynamuhsim_capture_config/4", "dynamuhsim_capture_config/5", "dynamuhsim_capture_config/6", "dynamuhsim_capture_config/7"] and wanted_inflation not in [.8,.45]:raise ValueError("Undeclared geometry experiment inflation")
    if nav != dict(robot_radius=wanted_radius, footprint_padding=.01, inflation_radius=wanted_inflation, cost_scaling_factor=3.,
                   speed=dict(vx_max=.30, vx_min=-.15, vy_max=.20)):
        raise ValueError("Capture version1 retains the accepted moderate Nav2 treatment")
    if version in ["dynamuhsim_capture_config/2", "dynamuhsim_capture_config/3", "dynamuhsim_capture_config/4", "dynamuhsim_capture_config/5", "dynamuhsim_capture_config/6", "dynamuhsim_capture_config/7"]:
        navigation = dict(goal_checker=dict(stateful=False, xy_goal_tolerance=.05, yaw_goal_tolerance=.15),
                          goal_critic_distance_m=.45)
        if version in ["dynamuhsim_capture_config/3", "dynamuhsim_capture_config/4", "dynamuhsim_capture_config/5", "dynamuhsim_capture_config/6", "dynamuhsim_capture_config/7"]:
            navigation["scan"] = dict(inf_is_valid=True, obstacle_max_range=3.0, raytrace_max_range=3.5)
            if type(config["navigation"].get("scan", {}).get("inf_is_valid")) is not bool:
                raise ValueError("Capture version3 requires declared no-return scan clearing")
        if config["navigation"] != navigation or type(config["navigation"]["goal_checker"]["stateful"]) is not bool:
            raise ValueError("Capture version2 retains the declared precise-goal navigation treatment")
    return copy.deepcopy(config)


def capture_sources(repo):
    files = source_files(repo, PROFILES["socnavgym_group_nav2"], repo / "tools/capture_episode.py")
    files += [repo / "python/dynamuhsim/simulation/robot_geometry.py", repo / "tools/capture_warehouse.launch.py", repo / "configs/capture/storage.yaml", repo / "configs/capture/qos.yaml",
              repo / "src/custom_corridor/models/mecanum_robot/model.sdf", repo / "src/custom_corridor/urdf/mecanum_robot.urdf",
              repo / "src/custom_corridor/config/mecanum_bridge.yaml",
              stage_directory(repo, 'depth_proxy_ablation') / 'depth_proxy_filter.cc', stage_directory(repo, 'depth_proxy_ablation') / 'CMakeLists.txt',
              stage_directory(repo, 'depth_proxy_ablation') / 'generated/build_jazzy/libdepth_proxy_filter.so',
              repo / 'python/dynamuhsim/runtime/native/check_friction.cc', repo / 'python/dynamuhsim/runtime/native/generated/check_friction']
    return sorted(set(files))


def prepare_run(repo, run_dir, config_path, scenario, seed, gui=False, *, planner_diagnostics=False, human_motion_profile="legacy", encounter_profile=None, human_goal_clock="native_wall"):
    repo, run_dir, config_path = Path(repo).resolve(), Path(run_dir).resolve(), Path(config_path).resolve()
    run_dir.relative_to(repo)
    config = validate_config(json.loads(config_path.read_text()))
    if scenario not in ["standing", "walking"] or type(seed) is not int or seed not in config["allowed_seeds"]:
        raise ValueError("Scenario/seed not allowed by capture config")
    if human_motion_profile not in ["legacy", "group_orca_v1", "group_goal_v1", "group_cohesion_v1"]:
        raise ValueError("Unsupported human motion profile")
    if human_goal_clock not in ["native_wall", "sim_v1"]:
        raise ValueError("Unsupported human goal clock")
    if human_motion_profile in ["group_goal_v1", "group_cohesion_v1"] and human_goal_clock != "sim_v1":
        raise ValueError("Shared group goal profile requires declared sim_v1 clock")
    encounter = None
    if encounter_profile is not None:
        from dynamuhsim.simulation.encounters import fixture, initial_clearance
        if scenario != "walking":
            raise ValueError("Encounter profiles require walking scenario")
        encounter = fixture(encounter_profile)
        initial_clearance(encounter, json.loads((repo / "experiments/social_navigation/warehouse/layout.json").read_text())["obstacles"])
    files = capture_sources(repo) + [config_path]
    for path in files:
        path.relative_to(repo)
        if not path.is_file():
            raise FileNotFoundError("Prepare dependency first: " + str(path))
    PROFILES["socnavgym_group_nav2"].validate_gates(repo / "experiments/social_navigation/benchmark")
    run_dir.mkdir(parents=True, exist_ok=False)
    assets = run_dir / "assets"
    assets.mkdir()
    camera = prepare_camera(assets, repo)
    if camera["runtime_sha256"] != CAMERA_SHA256:
        raise RuntimeError("Camera fixture differs from accepted native bytes")
    checker = repo / "python/dynamuhsim/runtime/native/generated/check_friction"
    friction = subprocess.run([str(checker), str(assets / "mecanum_camera.sdf")], capture_output=True, text=True, check=True)
    friction = json.loads(friction.stdout)
    if friction["recognized_gz_base_link_frames"] != 4:
        raise RuntimeError("Native SDFormat friction frames were not recognized")
    geometry = None
    if config["schema"] in ["dynamuhsim_capture_config/4", "dynamuhsim_capture_config/5", "dynamuhsim_capture_config/6", "dynamuhsim_capture_config/7"]:
        from dynamuhsim.simulation.robot_geometry import aligned_urdf
        geometry = aligned_urdf(repo / "src/custom_corridor/models/mecanum_robot/model.sdf", repo / "src/custom_corridor/urdf/mecanum_robot.urdf", assets / "mecanum_aligned.urdf")
        files.append(assets / "mecanum_aligned.urdf")
    preflight = dict(camera=camera, native_friction=friction)
    if geometry is not None:preflight["geometry"] = geometry
    dump(run_dir / "preflight.json", preflight)
    plan = dict(schema="dynamuhsim_capture_plan/1", config=config, scenario=scenario, seed=seed, gui=bool(gui))
    if human_motion_profile != "legacy":
        from dynamuhsim.simulation.human_motion_profiles import profile_config
        dump(assets / "human_motion.json", profile_config(human_motion_profile))
        files.append(assets / "human_motion.json")
        plan["human_motion_profile"] = human_motion_profile
    if encounter is not None:
        dump(assets / "encounter.json", encounter)
        files.append(assets / "encounter.json")
        plan["encounter_profile"] = encounter_profile
    if human_goal_clock != "native_wall":
        from dynamuhsim.simulation.goal_clock import PROFILE
        dump(assets / "human_goal_clock.json", PROFILE)
        files.append(assets / "human_goal_clock.json")
        plan["human_goal_clock"] = human_goal_clock
    if planner_diagnostics:plan["planner_diagnostics"] = True
    dump(run_dir / "plan.json", plan)
    files += [assets / "mecanum_camera.sdf", run_dir / "plan.json", run_dir / "preflight.json"]
    manifest = dict(schema="dynamuhsim_capture_freeze/2", git_head=subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip(),
                    created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                    source_sha256={str(path.relative_to(repo)): sha(path) for path in sorted(set(files))},
                    historical_manifest_modified=False, retry_policy="Never overwrite or automatically retry an existing episode")
    dump(run_dir / "capture_freeze.json", manifest)
    verify_run(repo, run_dir)
    return manifest


def verify_run(repo, run_dir):
    repo, run_dir = Path(repo).resolve(), Path(run_dir).resolve()
    run_dir.relative_to(repo)
    manifest = json.loads((run_dir / "capture_freeze.json").read_text())
    if manifest["schema"] != "dynamuhsim_capture_freeze/2":
        raise ValueError("Expected a new capture freeze, not an historical manifest")
    for name, digest in manifest["source_sha256"].items():
        path = PurePosixPath(name)
        if path.is_absolute() or ".." in path.parts or sha(repo / name) != digest:
            raise RuntimeError("Frozen capture input changed: " + name)
    required_sources = {str(path.relative_to(repo)) for path in capture_sources(repo)}
    if not required_sources.issubset(manifest["source_sha256"]):
        raise RuntimeError("Required source missing from capture freeze")
    for name in ["plan.json", "preflight.json", "assets/mecanum_camera.sdf"]:
        if str((run_dir / name).relative_to(repo)) not in manifest["source_sha256"]:
            raise RuntimeError("Required run input missing from freeze: " + name)
    plan = json.loads((run_dir / "plan.json").read_text())
    if plan.get("config", {}).get("schema") in ["dynamuhsim_capture_config/4", "dynamuhsim_capture_config/5", "dynamuhsim_capture_config/6", "dynamuhsim_capture_config/7"] and str((run_dir / "assets/mecanum_aligned.urdf").relative_to(repo)) not in manifest["source_sha256"]:
        raise RuntimeError("Aligned robot description missing from freeze")
    if "human_motion_profile" in plan:
        from dynamuhsim.simulation.human_motion_profiles import validate_profile
        asset = run_dir / "assets/human_motion.json"
        if str(asset.relative_to(repo)) not in manifest["source_sha256"]:
            raise RuntimeError("Human motion profile missing from freeze")
        if validate_profile(json.loads(asset.read_text()))["profile"] != plan["human_motion_profile"]:
            raise RuntimeError("Human motion profile differs from plan")
    if "encounter_profile" in plan:
        from dynamuhsim.simulation.encounters import validate_fixture
        asset = run_dir / "assets/encounter.json"
        if str(asset.relative_to(repo)) not in manifest["source_sha256"]:
            raise RuntimeError("Encounter fixture missing from freeze")
        if validate_fixture(json.loads(asset.read_text()))["profile"] != plan["encounter_profile"]:
            raise RuntimeError("Encounter fixture differs from plan")
    if "human_goal_clock" in plan:
        from dynamuhsim.simulation.goal_clock import validate_profile
        asset = run_dir / "assets/human_goal_clock.json"
        if str(asset.relative_to(repo)) not in manifest["source_sha256"]:
            raise RuntimeError("Human goal clock missing from freeze")
        if validate_profile(json.loads(asset.read_text()))["profile"] != plan["human_goal_clock"]:
            raise RuntimeError("Human goal clock differs from plan")
    return manifest


class CaptureContext:
    def __init__(self, repo, run_dir):
        self.repo, self.run_dir = Path(repo).resolve(), Path(run_dir).resolve()
        self.manifest = verify_run(self.repo, self.run_dir)
        self.plan = json.loads((self.run_dir / "plan.json").read_text())
        self.config = validate_config(self.plan["config"])
        self.runtime_dir = self.run_dir / "runtime"
        self.recorder = self.pairs = self.diagnostics = None

    def source_paths(self):
        return [self.repo / path for path in self.manifest["source_sha256"]] + [self.run_dir / "capture_freeze.json"]

    def prepare(self, runtime, result):
        verify_run(self.repo, self.run_dir)
        shutil.copyfile(self.run_dir / "assets/mecanum_camera.sdf", runtime / "mecanum_camera.sdf")
        if self.config["schema"] in ["dynamuhsim_capture_config/4", "dynamuhsim_capture_config/5", "dynamuhsim_capture_config/6", "dynamuhsim_capture_config/7"]:
            shutil.copyfile(self.run_dir / "assets/mecanum_aligned.urdf", runtime / "mecanum_aligned.urdf")
            result["robot_geometry_alignment"] = json.loads((self.run_dir / "preflight.json").read_text())["geometry"]
        if "human_motion_profile" in self.plan:
            shutil.copyfile(self.run_dir / "assets/human_motion.json", runtime / "human_motion.json")
            result["human_motion_profile"] = self.plan["human_motion_profile"]
        if "encounter_profile" in self.plan:
            shutil.copyfile(self.run_dir / "assets/encounter.json", runtime / "encounter.json")
            result["encounter_profile"] = self.plan["encounter_profile"]
        if "human_goal_clock" in self.plan:
            shutil.copyfile(self.run_dir / "assets/human_goal_clock.json", runtime / "human_goal_clock.json")
            result["human_goal_clock"] = self.plan["human_goal_clock"]
        result.update(capture_schema="dynamuhsim_capture/2", capture_freeze_sha256=sha(self.run_dir / "capture_freeze.json"),
                      experiment="Versioned capture API; native camera and moderate Nav2 treatment",
                      capture_recipe_schema=self.config["schema"],
                      camera_runtime_corrections=json.loads((self.run_dir / "preflight.json").read_text()))

    def configure_world(self, tree):
        import xml.etree.ElementTree as ET
        library = self.repo / "experiments/social_navigation/perception/depth_proxy_ablation/generated/build_jazzy/libdepth_proxy_filter.so"
        ET.SubElement(tree.getroot().find("world"), "plugin", filename=str(library), name="depth_proxy_experiment::Filter")

    def configure_nav(self, config, output):
        result = apply_nav2_treatment(config, self.config["nav2"], navigation=self.config.get("navigation"), controller=self.config.get("controller"))
        dump(output / "config_changes.json", config_changes(config, result))
        return result

    def attach(self, node, output, telemetry):
        from .capture_recording import Recording, Pairs
        extra_topics = ()
        if self.plan.get("planner_diagnostics"):
            from .planner_diagnostics import NativeCostmaps, TOPICS
            self.diagnostics = NativeCostmaps(node, output, telemetry)
            extra_topics = TOPICS
        self.recorder = Recording(node, output, extra_topics=extra_topics)
        self.pairs = Pairs(node, output, lambda: telemetry.phase)
        telemetry.scene_sink = self.recorder.scene

    def start_bridge(self, runtime, output, env, processes, files):
        env.pop("DYNAMUHSIM_CAPTURE_MAP", None)
        if self.config["schema"] in ["dynamuhsim_capture_config/4", "dynamuhsim_capture_config/5", "dynamuhsim_capture_config/6", "dynamuhsim_capture_config/7"]:env["DATASET_ROBOT_URDF"] = str(runtime / "mecanum_aligned.urdf")
        else:env.pop("DATASET_ROBOT_URDF", None)
        log = (output / "pair_bridge.log").open("w")
        files.append(log)
        command = ["ros2", "run", "ros_gz_bridge", "parameter_bridge", "/proxy_check/on/depth@sensor_msgs/msg/Image[gz.msgs.Image"]
        processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=env))

    def ready(self, node, wait, output, result, telemetry):
        if not wait(lambda: self.recorder.ready() and self.pairs.ready(), 30):
            raise RuntimeError("Full recorder/paired depth/calibration not ready")
        additional = None
        if self.diagnostics is not None:
            if not wait(self.diagnostics.ready, 15):raise RuntimeError("Native global/local costmaps missing")
            from .planner_diagnostics import PLANNER_PARAMETERS
            additional = PLANNER_PARAMETERS
        result["live_parameters"] = capture_live_parameters(node, wait, self.config["nav2"], self.config.get("navigation"), additional=additional, controller=self.config.get("controller"))
        dump(output / "live_parameters.json", result["live_parameters"])
        self.recorder.event("ready", round(telemetry.simtime() * 1e9), scenario=self.plan["scenario"], seed=self.plan["seed"])

    def check_health(self):
        if self.recorder is not None and self.recorder.process.poll() is not None:
            raise RuntimeError("Recorder exited during capture; preserve the failed episode")

    def started(self, start):
        self.recorder.event("goal_accepted", round(start * 1e9), goal_world=[-20., 0., 0.])

    def finish(self, wait, telemetry, result):
        self.recorder.event("action_end", round(result["action_end_sim"] * 1e9),
                            navigation_status=result["navigation_status"], actual_goal_reached=result["actual_goal_reached"])
        start = telemetry.simtime()
        telemetry.phase = "label_tail"
        if not wait(lambda: telemetry.simtime() - start >= self.config["label_tail_s"], self.config["tail_wall_budget_s"]):
            raise RuntimeError("Future label tail incomplete")
        self.recorder.event("observation_end", round(telemetry.simtime() * 1e9))
        wait(lambda: False, .5)
        telemetry.phase = "finished"

    def close(self, result):
        if getattr(self, "diagnostics", None) is not None:
            result["planner_diagnostics"] = self.diagnostics.close()
        if self.pairs is not None:
            result["depth_pairs"] = self.pairs.close()
        if self.recorder is not None:
            result["recording"] = self.recorder.close()
            if result["recording"]["returncode"] != 0 or not result["recording"]["metadata_present"]:
                raise RuntimeError("MCAP recorder did not close successfully")

    def stop_server(self, guard, original_stop):
        return stop_paused(guard, original_stop)


def capture_live_parameters(node, wait, treatment, navigation=None, *, additional=None, controller=None):
    from rclpy.parameter_client import AsyncParameterClient
    expected = nav2_parameter_contract(treatment, navigation, controller=controller)
    expected.update(additional or {})
    observed = {}
    for remote, parameters in expected.items():
        client = AsyncParameterClient(node, remote)
        if not wait(client.services_are_ready, 10):
            raise RuntimeError("Parameter service not ready: " + remote)
        future = client.get_parameters(list(parameters))
        if not wait(future.done, 10):
            raise RuntimeError("Parameter request timed out: " + remote)
        values = future.result().values
        observed[remote] = {name: (value.bool_value if isinstance(wanted, bool) else value.string_value if isinstance(wanted, str) else value.integer_value if isinstance(wanted, int) else value.double_value)
                            for (name, wanted), value in zip(parameters.items(), values)}
        if observed[remote] != parameters:
            raise RuntimeError("Live Nav2 parameters differ: " + remote)
    return observed
