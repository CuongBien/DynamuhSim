"""Versioned capture hooks. New runs never execute report snapshots or rewrite source."""
from pathlib import Path, PurePosixPath
from types import SimpleNamespace
import copy
import datetime
import hashlib
import json
import math
import shutil
import subprocess

from .nav2_trial import source_files
from .profiles import PROFILES
from dynamuhsim.simulation.capture import prepare_camera, apply_nav2_treatment, config_changes

CAMERA_SHA256 = "14c15412664eb37dd2f0fb2d61a79b09f98722f204f97d97212f0ec814ad3d9f"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def validate_config(config):
    expected = {"schema", "allowed_seeds", "sim_budget_s", "label_tail_s", "tail_wall_budget_s", "camera_sha256", "nav2"}
    if set(config) != expected or config["schema"] != "dynamuhsim_capture_config/1":
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
    if nav != dict(robot_radius=.25, footprint_padding=.01, inflation_radius=.8, cost_scaling_factor=3.,
                   speed=dict(vx_max=.30, vx_min=-.15, vy_max=.20)):
        raise ValueError("Capture version1 retains the accepted moderate Nav2 treatment")
    return copy.deepcopy(config)


def capture_sources(repo):
    benchmark = repo / "experiments/social_navigation/benchmark"
    files = source_files(repo, PROFILES["socnavgym_group_nav2"], repo / "tools/capture_episode.py")
    files += [repo / "tools/capture_warehouse.launch.py", repo / "configs/capture/storage.yaml", repo / "configs/capture/qos.yaml",
              repo / "src/custom_corridor/models/mecanum_robot/model.sdf", repo / "src/custom_corridor/urdf/mecanum_robot.urdf",
              repo / "src/custom_corridor/launch/corridor_tb3.launch.py", repo / "src/custom_corridor/config/mecanum_bridge.yaml",
              benchmark / "depth_proxy_ablation/depth_proxy_filter.cc", benchmark / "depth_proxy_ablation/CMakeLists.txt",
              benchmark / "depth_proxy_ablation/generated/build_jazzy/libdepth_proxy_filter.so",
              benchmark / "early_avoidance_pilot/check_friction.cc", benchmark / "early_avoidance_pilot/generated/check_friction"]
    return sorted(set(files))


def prepare_run(repo, run_dir, config_path, scenario, seed, gui=False):
    repo, run_dir, config_path = Path(repo).resolve(), Path(run_dir).resolve(), Path(config_path).resolve()
    run_dir.relative_to(repo)
    config = validate_config(json.loads(config_path.read_text()))
    if scenario not in ["standing", "walking"] or type(seed) is not int or seed not in config["allowed_seeds"]:
        raise ValueError("Scenario/seed not allowed by capture config")
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
    checker = repo / "experiments/social_navigation/benchmark/early_avoidance_pilot/generated/check_friction"
    friction = subprocess.run([str(checker), str(assets / "mecanum_camera.sdf")], capture_output=True, text=True, check=True)
    friction = json.loads(friction.stdout)
    if friction["recognized_gz_base_link_frames"] != 4:
        raise RuntimeError("Native SDFormat friction frames were not recognized")
    dump(run_dir / "preflight.json", dict(camera=camera, native_friction=friction))
    dump(run_dir / "plan.json", dict(schema="dynamuhsim_capture_plan/1", config=config, scenario=scenario, seed=seed, gui=bool(gui)))
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
    return manifest


class CaptureContext:
    def __init__(self, repo, run_dir):
        self.repo, self.run_dir = Path(repo).resolve(), Path(run_dir).resolve()
        self.manifest = verify_run(self.repo, self.run_dir)
        self.plan = json.loads((self.run_dir / "plan.json").read_text())
        self.config = validate_config(self.plan["config"])
        self.runtime_dir = self.run_dir / "runtime"
        self.recorder = self.pairs = None

    def source_paths(self):
        return [self.repo / path for path in self.manifest["source_sha256"]] + [self.run_dir / "capture_freeze.json"]

    def prepare(self, runtime, result):
        verify_run(self.repo, self.run_dir)
        shutil.copyfile(self.run_dir / "assets/mecanum_camera.sdf", runtime / "mecanum_camera.sdf")
        result.update(capture_schema="dynamuhsim_capture/2", capture_freeze_sha256=sha(self.run_dir / "capture_freeze.json"),
                      experiment="Versioned capture API; native camera and moderate Nav2 treatment",
                      camera_runtime_corrections=json.loads((self.run_dir / "preflight.json").read_text()))

    def configure_world(self, tree):
        import xml.etree.ElementTree as ET
        library = self.repo / "experiments/social_navigation/benchmark/depth_proxy_ablation/generated/build_jazzy/libdepth_proxy_filter.so"
        ET.SubElement(tree.getroot().find("world"), "plugin", filename=str(library), name="depth_proxy_experiment::Filter")

    def configure_nav(self, config, output):
        result = apply_nav2_treatment(config, self.config["nav2"])
        dump(output / "config_changes.json", config_changes(config, result))
        return result

    def attach(self, node, output, telemetry):
        from .capture_recording import Recording, Pairs
        self.recorder = Recording(node, output)
        self.pairs = Pairs(node, output, lambda: telemetry.phase)
        telemetry.scene_sink = self.recorder.scene

    def start_bridge(self, runtime, output, env, processes, files):
        log = (output / "pair_bridge.log").open("w")
        files.append(log)
        command = ["ros2", "run", "ros_gz_bridge", "parameter_bridge", "/proxy_check/on/depth@sensor_msgs/msg/Image[gz.msgs.Image"]
        processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=env))

    def ready(self, node, wait, output, result, telemetry):
        if not wait(lambda: self.recorder.ready() and self.pairs.ready(), 30):
            raise RuntimeError("Full recorder/paired depth/calibration not ready")
        result["live_parameters"] = capture_live_parameters(node, wait, self.config["nav2"])
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
        if self.pairs is not None:
            result["depth_pairs"] = self.pairs.close()
        if self.recorder is not None:
            result["recording"] = self.recorder.close()
            if result["recording"]["returncode"] != 0 or not result["recording"]["metadata_present"]:
                raise RuntimeError("MCAP recorder did not close successfully")

    def stop_server(self, guard, original_stop):
        return stop_paused(guard, original_stop)


def capture_live_parameters(node, wait, treatment):
    from rclpy.parameter_client import AsyncParameterClient
    expected = {"/controller_server": {"FollowPath." + key: value for key, value in treatment["speed"].items()}}
    expected["/controller_server"]["FollowPath.CostCritic.consider_footprint"] = False
    for name in ["local_costmap", "global_costmap"]:
        expected[f"/{name}/{name}"] = {"robot_radius": treatment["robot_radius"], "footprint_padding": treatment["footprint_padding"],
                                      "inflation_layer.inflation_radius": treatment["inflation_radius"],
                                      "inflation_layer.cost_scaling_factor": treatment["cost_scaling_factor"]}
    observed = {}
    for remote, parameters in expected.items():
        client = AsyncParameterClient(node, remote)
        if not wait(client.services_are_ready, 10):
            raise RuntimeError("Parameter service not ready: " + remote)
        future = client.get_parameters(list(parameters))
        if not wait(future.done, 10):
            raise RuntimeError("Parameter request timed out: " + remote)
        values = future.result().values
        observed[remote] = {name: (value.bool_value if isinstance(wanted, bool) else value.double_value)
                            for (name, wanted), value in zip(parameters.items(), values)}
        if observed[remote] != parameters:
            raise RuntimeError("Live Nav2 parameters differ: " + remote)
    return observed


def stop_paused(guard,original_stop,world='social_warehouse'):
    """Pause is a mitigation, not a GPU fence or proof of root cause."""
    import threading, time
    from gz.transport13 import Node
    from gz.msgs10.world_stats_pb2 import WorldStatistics
    QUIET_SECONDS = .5
    event=threading.Event();stats=[];node=Node()
    def received(msg):
        if msg.paused:
            stats.append(dict(paused=True,sim_sec=msg.sim_time.sec,sim_nsec=msg.sim_time.nsec));event.set()
    topic=f'/world/{world}/stats';subscribed=node.subscribe(WorldStatistics,topic,received)
    request=subprocess.run(['gz','service','-s',f'/world/{world}/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','3000','--req','pause: true'],capture_output=True,text=True,timeout=5)
    acknowledged=bool(subscribed and request.returncode==0 and 'data: true' in request.stdout and event.wait(3))
    settled=False
    if acknowledged:
        time.sleep(QUIET_SECONDS)
        # The statistics stream must confirm unchanged simulation time while paused.
        settled=bool(stats and len({(r['sim_sec'],r['sim_nsec']) for r in stats})==1)
    node.unsubscribe(topic)
    result=original_stop(guard)
    result['quiesce']=dict(pause_acknowledged=acknowledged,simulation_time_stable=settled,
        settle_wall_s=QUIET_SECONDS,stats=stats,request_returncode=request.returncode,
        response=request.stdout.strip(),stderr=request.stderr.strip(),is_gpu_fence=False)
    (guard.runtime/'quiesce_shutdown.json').write_text(json.dumps(result,indent=2)+'\n')
    return result
