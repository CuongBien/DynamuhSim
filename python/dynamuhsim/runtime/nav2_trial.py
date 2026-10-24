"""One Nav2 runner for individual, grouped, and isolated-backend warehouse pilots.

Historical reports/drivers and the camera/WM capture pipeline are immutable inputs
of earlier stages, not templates for this runner.
"""
from pathlib import Path
from types import SimpleNamespace
import fcntl
import hashlib
import importlib.util
import json
import math
import os
import signal
import subprocess
import tempfile
import time
import traceback
import xml.etree.ElementTree as ET

from .navigation import execute_goal, goal_metrics, wait_for
from .profiles import PROFILES
from .recording import TrialRecording, endpoint, yaw
from dynamuhsim.simulation.warehouse import configure_world, runtime_nav2_config


def load_file_module(name, path):
    """Load a support module by explicit path, not a generic `run`/`runtime` import."""
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_dependencies(repo):
    # CLI help and offline unit tests do not import optional ROS/Gazebo modules.
    import yaml
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from nav2_msgs.action import NavigateToPose
    from lifecycle_msgs.srv import GetState
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry, Path as RosPath, OccupancyGrid
    from sensor_msgs.msg import LaserScan, Imu
    from rosgraph_msgs.msg import Clock
    from rcl_interfaces.msg import Log
    from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
    from tf2_ros import Buffer, TransformListener
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.image_pb2 import Image as GzImage

    social = repo / "experiments/social_navigation"
    benchmark = social / "benchmark"
    adapter = load_file_module("_dynamuhsim_gazebo_adapter", social / "gazebo_adapter/run.py")
    gpu = load_file_module("_dynamuhsim_gpu_runtime", benchmark / "gpu_runtime/runtime_guard.py")
    shutdown = load_file_module("_dynamuhsim_ordered_shutdown", benchmark / "shutdown_fix/ordered_stop.py")
    observer = load_file_module("_dynamuhsim_amcl_observer", benchmark / "amcl_observer.py")
    return SimpleNamespace(**locals())


def stop_ros_launch(process, fallback_stop, timeout=15):
    """Let ROS launch signal its children once; retain group cleanup as fallback.

    killpg(SIGINT) hits both launch and every node. Launch then forwards SIGINT
    again, which can interrupt the nodes' own shutdown and report exit -2/255.
    """
    if process.poll() is not None:
        return dict(status="ALREADY_EXITED", returncode=process.returncode)
    try:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=timeout)
        return dict(status="STOPPED", returncode=process.returncode,
                    signal_scope="ROS launch parent; launch owns child shutdown")
    except ProcessLookupError:
        process.wait(timeout=timeout)
        return dict(status="ALREADY_EXITED", returncode=process.returncode)
    except subprocess.TimeoutExpired:
        fallback_stop(process)
        return dict(status="ESCALATED", returncode=process.returncode,
                    signal_scope="process group fallback after launch grace period")


def source_files(repo, profile, entrypoint):
    social = repo / "experiments/social_navigation"
    benchmark = social / "benchmark"
    stage = benchmark / profile.name
    fix = benchmark / "socnavgym_response_fix"
    common = benchmark / "common_response"
    files = [entrypoint, stage / "worker.py", stage / "warehouse.launch.py", stage / "protocol.json",
             fix / "projection.py", fix / "responsive_motion.py", fix / "adapter.cc", fix / "build/libresponse_adapter.so",
             common / "model_worker.py", common / "inputs/crossing_near.json",
             social / "warehouse/nav2_warehouse.yaml", social / "warehouse/warehouse.yaml",
             social / "warehouse/warehouse.pgm", social / "warehouse/warehouse.sdf",
             repo / "src/custom_corridor/launch/nav2_corridor.launch.py",
             repo / "src/custom_corridor/launch/publish_episode_initial_pose.py"]
    if profile.groups:
        files += [stage / name for name in ["fixture.py", "reactive_motion.py", "standing_projection.py", "config.yaml"]]
        files += [benchmark / "socnavgym_warehouse_lifecycle" / name for name in ["kernel.py", "motion.py", "geometry.py", "config.yaml"]]
        files += [benchmark / "socnavgym_lifecycle_fix/continuous_groups.py",
                  social / "vendor/SocNavGym/socnavgym/envs/socnavenv_v2.py",
                  social / "vendor/SocNavGym/socnavgym/envs/utils/human_human.py"]
    if profile.container_worker:
        files.append(stage / "runtime.py")
    files += sorted((repo / "python/dynamuhsim").rglob("*.py"))
    files += [repo / "pyproject.toml", social / "gazebo_adapter/run.py",
              benchmark / "gpu_runtime/runtime_guard.py", benchmark / "shutdown_fix/ordered_stop.py",
              benchmark / "amcl_observer.py"]
    return files


def run_trial(profile, args, repo, entrypoint, *, dependencies=None):
    deps = dependencies if dependencies is not None else load_dependencies(repo)
    social = repo / "experiments/social_navigation"
    benchmark = social / "benchmark"
    stage = benchmark / profile.name
    plan = json.loads((stage / "protocol.json").read_text())
    dt = profile.planning_dt(plan, args)
    out = args.report_dir.resolve()
    # Preserve prior attempts: neither report nor runtime directory is overwritten.
    out.mkdir(parents=True, exist_ok=False)
    runtime = stage / "generated" / out.name
    runtime.parent.mkdir(parents=True, exist_ok=True)
    runtime.mkdir(exist_ok=False)
    recording = TrialRecording()
    result = dict(status="ERROR", condition=args.condition, gui=args.gui,
                  head=subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip(),
                  runtime=str(runtime.relative_to(repo)), robot_start=plan["start"], robot_goal=plan["goal"],
                  sim_budget_s=plan["sim_budget"], no_scripted_robot_velocity_publisher=True,
                  **profile.result_fields(args, dt))
    processes, files, accepted_handles = [], [], []
    node = guard = gui_process = socket_dir = lock = launch = None
    container_name = None
    sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    try:
        lock_path = benchmark / "common_gazebo/generated/domain91.lock"
        lock_path.parent.mkdir(parents=True, exist_ok=True)
        lock = lock_path.open("a")
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        os.environ.update(ROS_DOMAIN_ID="91", GZ_PARTITION=profile.partition_prefix + str(os.getpid()),
                          ROS_LOG_DIR=str(runtime / "ros_logs"), QT_QPA_PLATFORM="xcb",
                          ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST", FASTDDS_BUILTIN_TRANSPORTS="UDPv4")
        guard = deps.gpu.RuntimeGuard(runtime)
        result["source_sha256"] = {str(path.relative_to(repo)): sha(path) for path in source_files(repo, profile, entrypoint)}
        (out / "protocol.json").write_text(json.dumps(result, indent=2) + "\n")
        profile.validate_gates(benchmark)
        result["rendering"] = deps.gpu.gpu_preflight(runtime)
        launch_env, result["shutdown_preload"] = deps.shutdown.launch_environment(runtime)
        socket_dir = tempfile.TemporaryDirectory(prefix="nav2v2_")
        socket_path = str(Path(socket_dir.name) / "worker.sock")
        initial = []
        if args.condition == "people":
            log = (runtime / "worker.log").open("w")
            files.append(log)
            if profile.container_worker:
                backend = load_file_module("_dynamuhsim_shared_backend", stage / "runtime.py")
                command, container_name, image = backend.worker_command(runtime, Path(socket_dir.name), args.model, guard.token)
                result["container_image"] = image
            else:
                command = profile.worker_command(social, runtime, socket_path, args)
            result["worker_command"] = command
            worker = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            processes.append(worker)
            deadline = time.monotonic() + 90
            while not (runtime / "ready.json").exists():
                if worker.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("Worker not ready")
                time.sleep(.1)
            ready = json.loads((runtime / "ready.json").read_text())
            initial = ready["humans"]
            result["ready"] = ready
            assert len(initial) == 6 and ready["requires_start_gate"]
        deps.adapter.make_world(runtime, socket_path, initial, "walking")
        tree = configure_world(ET.parse(runtime / "warehouse.sdf"),
                               benchmark / "socnavgym_response_fix/build/libresponse_adapter.so",
                               args.condition, dt, planning_dt_text=None if profile.container_worker else ".1")
        ET.indent(tree)
        tree.write(runtime / "warehouse.sdf", encoding="unicode")
        config = runtime_nav2_config(deps.yaml.safe_load((social / "warehouse/nav2_warehouse.yaml").read_text()))
        (runtime / "nav2_runtime.yaml").write_text(deps.yaml.safe_dump(config, sort_keys=False))
        result.update(runtime_nav_params_sha256=sha(runtime / "nav2_runtime.yaml"), runtime_world_sha256=sha(runtime / "warehouse.sdf"))
        launch_env.update(NAV2_PILOT_WORLD=str(runtime / "warehouse.sdf"), NAV2_PILOT_PARAMS=str(runtime / "nav2_runtime.yaml"))
        deps.rclpy.init()
        node = deps.Node(profile.name, parameter_overrides=[deps.rclpy.parameter.Parameter("use_sim_time", value=True)])
        gz = deps.GzNode()
        tfbuffer = deps.Buffer()
        tflistener = deps.TransformListener(tfbuffer, node)
        topics = [("gt", "/common/ground_truth", deps.String), ("scan", "/scan", deps.LaserScan),
                  ("imu", "/imu", deps.Imu), ("odom", "/odom", deps.Odometry), ("clock", "/clock", deps.Clock),
                  ("amcl", "/amcl_pose", deps.PoseWithCovarianceStamped), ("cmd", "/cmd_vel", deps.TwistStamped),
                  ("plan", "/plan", deps.RosPath), ("local_costmap", "/local_costmap/costmap", deps.OccupancyGrid),
                  ("log", "/rosout", deps.Log)]
        for key, topic, kind in topics:
            qos = deps.observer.observer_qos("transient_local") if key == "amcl" else deps.qos_profile_sensor_data
            node.create_subscription(kind, topic, lambda msg, key=key: recording.receive(key, msg), qos)
        gz.subscribe(deps.Pose_V, "/world/social_warehouse/pose/info", recording.scene)
        gz.subscribe(deps.GzImage, "/common/observer", recording.camera)
        log = (out / "launch.log").open("w")
        files.append(log)
        gui = False if profile.groups else args.gui
        launch = subprocess.Popen(["ros2", "launch", str(stage / "warehouse.launch.py"), f"gui:={str(gui).lower()}"],
                                  stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=launch_env)
        processes.append(launch)

        def capture_tf():
            if not recording.rows["gazebo_poses"]:
                return
            try:
                transform = tfbuffer.lookup_transform("map", "base_footprint", deps.rclpy.time.Time())
                odom = tfbuffer.lookup_transform("map", "odom", deps.rclpy.time.Time())
                pose = lambda value: dict(x=value.translation.x, y=value.translation.y, yaw=yaw(value.rotation))
                recording.rows["tf"].append(dict(time=recording.simtime(), map_base=pose(transform.transform), map_odom=pose(odom.transform)))
            except Exception:
                pass  # A missing TF sample is handled by the explicit readiness gate below.

        def wait(predicate, seconds):
            return wait_for(predicate, seconds, spin_once=lambda: deps.rclpy.spin_once(node, timeout_sec=.02), processes=processes)

        def robot_pose():
            return recording.rows["gazebo_poses"][-1]["poses"]["mecanum_robot"]

        def sensors_ready():
            poses = recording.rows["gazebo_poses"]
            return (all(recording.counts[key] >= 5 for key in ["scan", "imu", "odom", "clock"])
                    and recording.counts["amcl"] > 0 and poses and "mecanum_robot" in poses[-1]["poses"]
                    and (args.condition == "empty" or len(recording.rows["ground_truth"]) > 5))

        assert wait(sensors_ready, 100), "Sensors/scene/localization missing"
        lifecycle = node.create_client(deps.GetState, "/bt_navigator/get_state")
        client = deps.ActionClient(node, deps.NavigateToPose, "/navigate_to_pose")
        deadline = time.monotonic() + 100
        ready = False
        while time.monotonic() < deadline:
            if lifecycle.service_is_ready() and client.server_is_ready():
                future = lifecycle.call_async(deps.GetState.Request())
                if wait(future.done, 4) and future.result().current_state.id == 3:
                    ready = True
                    break
            wait(lambda: False, .1)
        assert ready, "bt_navigator not active"
        assert wait(lambda: recording.counts["local_costmap"] > 0, 10), "Local costmap missing"
        capture_tf()
        assert recording.rows["tf"], "map->base TF missing"
        if profile.groups and args.gui:
            log = (runtime / "gui.log").open("w")
            files.append(log)
            gui_process = subprocess.Popen(["gz", "sim", "-g"], stdout=log, stderr=subprocess.STDOUT,
                                           start_new_session=True, env=launch_env)
            processes.append(gui_process)
            wait(lambda: False, 3.)
            result["gui_start_mode"] = "Separate GUI after server sensors/AMCL/Nav2 ready"
        result["amcl_publishers"] = deps.observer.publisher_evidence(node)
        publishers = lambda: [dict(node=item.node_name, namespace=item.node_namespace) for item in node.get_publishers_info_by_topic("/cmd_vel")]
        result["cmd_vel_publishers_before_goal"] = publishers()
        assert all(item["node"] in ["controller_server", "behavior_server"] for item in result["cmd_vel_publishers_before_goal"])
        robot = robot_pose()
        assert math.hypot(robot["x"] + 28, robot["y"]) < .02, "Robot moved in warmup"
        if args.condition == "people":
            poses = recording.rows["gazebo_poses"][-1]["poses"]
            assert max(math.hypot(poses[f'common_{human["id"]}_proxy']["x"] - human["x"],
                                  poses[f'common_{human["id"]}_proxy']["y"] - human["y"]) for human in initial) < .01, "Humans moved before start gate"
        goal = deps.NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = recording.latest["clock"].clock
        goal.pose.pose.position.x = -20.
        goal.pose.pose.position.y = 0.
        goal.pose.pose.orientation.w = 1.
        result["before_goal_world_pose"] = robot
        execute_goal(client, goal, recording, runtime, result, wait, capture_tf, on_accepted=accepted_handles.append)
        wait(lambda: False, .4)
        capture_tf()
        recording.phase = "finished"
        result.update(goal_metrics(robot_pose(), result["action_status"]))
        result.update(counts=dict(recording.counts), cmd_vel_publishers_after_goal=publishers(),
                      status="MEASURED", amcl_final_pose=endpoint(recording.latest["amcl"].pose.pose))
    except Exception as error:
        result["error"] = str(error)
        traceback.print_exc()
    finally:
        recording.phase = "finished"
        if result["status"] == "ERROR" and accepted_handles:
            try:
                accepted_handles[-1].cancel_goal_async()
            except Exception:
                pass
        if gui_process is not None:
            deps.adapter.stop(gui_process)
            result["gui_returncode"] = gui_process.returncode
        if guard is not None:
            try:
                result["server_shutdown"] = deps.shutdown.stop_server(guard)
            except Exception as error:
                result["shutdown_error"] = str(error)
        if container_name:
            query = ["docker", "ps", "-aq", "--filter", "name=^/" + container_name + "$"]
            if subprocess.check_output(query, text=True).strip():
                subprocess.run(["docker", "stop", "-t", "3", container_name], capture_output=True, timeout=12)
            result["container_remaining"] = subprocess.check_output(query, text=True).strip()
        for process in reversed(processes):
            if process is launch:
                result["ros_launch_shutdown"] = stop_ros_launch(process, deps.adapter.stop)
            else:
                deps.adapter.stop(process)
        if node is not None:
            try:
                node.destroy_node()
                deps.rclpy.shutdown()
            except Exception as error:
                result["ros_teardown_error"] = str(error)
        if guard is not None:
            result["cleanup"] = guard.cleanup()
        for stream in files:
            stream.close()
        if socket_dir is not None:
            socket_dir.cleanup()
        if lock is not None:
            lock.close()
        launch_log = out / "launch.log"
        result.update(faults=recording.faults,
                      gazebo_clean_exit=launch_log.exists() and "[INFO] [gz-1]: process has finished cleanly" in launch_log.read_text(errors="replace"))
        media = recording.write(out)
        if media is not None:
            result["media"] = media
        (out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
    return 0 if result["status"] == "MEASURED" else 1


def main(experiment, entrypoint, argv=None):
    profile = PROFILES[experiment]
    args = profile.parse_args(argv)
    repo = Path(__file__).resolve().parents[3]
    return run_trial(profile, args, repo, Path(entrypoint).resolve())
