"""Structured camera fixture and Nav2 treatment for new capture runs."""
import copy
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET


def camera_tree(source):
    # Native SDF uses a gz extension without an XML namespace declaration.
    # Supply its namespace as parsing context; do not patch Python or SDF source.
    text = Path(source).read_text().lstrip()
    if text.startswith("<?xml"):
        text = text.split("?>", 1)[1]
    root = ET.fromstring('<context xmlns:gz="urn:gz">' + text + '</context>')
    sdf = root.find("sdf")
    if sdf is None:
        raise ValueError("Expected SDF document")
    sdf.tail = None
    frames = sdf.findall(".//gz_frame_id")
    selected = [node for node in frames if node.text == "camera_link"]
    if len(selected) != 1:
        raise ValueError("Expected exactly one native depth camera frame")
    selected[0].text = "camera_optical_frame"
    link = sdf.find("model/link[@name='camera_link']")
    sensor = link.find("sensor")
    control = copy.deepcopy(sensor)
    control.set("name", "proxy_low_on")
    control.find("topic").text = "/proxy_check/on/depth"
    control.find("camera/camera_info_topic").text = "/proxy_check/on/camera_info"
    sensor.set("name", "proxy_low_off")
    link.append(control)
    friction = sdf.findall("model/link/collision/surface/friction/ode/fdir1")
    if len(friction) != 4 or any(node.get("{urn:gz}expressed_in") != "base_link" for node in friction):
        raise ValueError("Native four-wheel friction frame contract failed")
    ET.register_namespace("gz", "urn:gz")
    tree = ET.ElementTree(sdf)
    ET.indent(tree)
    return tree


def prepare_camera(runtime, repo):
    runtime, repo = Path(runtime), Path(repo)
    source = repo / "src/custom_corridor/models/mecanum_robot/model.sdf"
    target = runtime / "mecanum_camera.sdf"
    camera_tree(source).write(target, encoding="unicode")
    return dict(source=str(source.relative_to(repo)), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                runtime_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
                camera_pose_unchanged=True, literal_gz_expressed_in_count=4,
                off_topic="/camera/depth/image_rect_raw", on_topic="/proxy_check/on/depth")


def apply_nav2_treatment(config, treatment, *, navigation=None, controller=None):
    result = copy.deepcopy(config)
    for name in ["local_costmap", "global_costmap"]:
        params = result[name][name]["ros__parameters"]
        params.pop("footprint", None)
        params.update(robot_radius=treatment["robot_radius"], footprint_padding=treatment["footprint_padding"])
        if navigation is not None and "scan" in navigation:
            params["obstacle_layer"]["scan"].update(navigation["scan"])
        params["inflation_layer"].update(inflation_radius=treatment["inflation_radius"],
                                          cost_scaling_factor=treatment["cost_scaling_factor"])
    follow = result["controller_server"]["ros__parameters"]["FollowPath"]
    follow.update(treatment["speed"])
    follow["CostCritic"]["consider_footprint"] = False
    if controller is not None:
        follow["CostCritic"].update(controller["cost_critic"])
        if "time_steps" in controller:follow["time_steps"] = controller["time_steps"]
        if "regenerate_noises" in controller:follow["regenerate_noises"] = controller["regenerate_noises"]
    if navigation is not None:
        result["controller_server"]["ros__parameters"]["goal_checker"].update(navigation["goal_checker"])
        for critic in ["GoalCritic", "PathFollowCritic"]:
            follow[critic]["threshold_to_consider"] = navigation["goal_critic_distance_m"]
    return result


def config_changes(before, after, prefix=""):
    changes = []
    for key in sorted(set(before) | set(after)):
        a, b = before.get(key), after.get(key)
        path = prefix + "/" + key
        if isinstance(a, dict) and isinstance(b, dict):
            changes.extend(config_changes(a, b, path))
        elif a != b:
            changes.append(dict(path=path, before=a, after=b))
    return changes


def nav2_parameter_contract(treatment, navigation=None, *, controller=None):
    """Expected live values for the declared capture treatment and scan policy."""
    expected = {"/controller_server": {"FollowPath." + key: value for key, value in treatment["speed"].items()}}
    expected["/controller_server"]["FollowPath.CostCritic.consider_footprint"] = False
    if controller is not None:
        expected["/controller_server"].update({"FollowPath.CostCritic."+key:value for key,value in controller["cost_critic"].items()})
        if "time_steps" in controller:
            expected["/controller_server"]["FollowPath.time_steps"] = controller["time_steps"]
            expected["/controller_server"]["FollowPath.model_dt"] = .05
        if "regenerate_noises" in controller:expected["/controller_server"]["FollowPath.regenerate_noises"] = controller["regenerate_noises"]
    if navigation is not None:
        expected["/controller_server"].update({"goal_checker." + key: value for key, value in navigation["goal_checker"].items()})
        for critic in ["GoalCritic", "PathFollowCritic"]:
            expected["/controller_server"]["FollowPath." + critic + ".threshold_to_consider"] = navigation["goal_critic_distance_m"]
    for name in ["local_costmap", "global_costmap"]:
        expected[f"/{name}/{name}"] = {"robot_radius": treatment["robot_radius"], "footprint_padding": treatment["footprint_padding"],
                                      "inflation_layer.inflation_radius": treatment["inflation_radius"],
                                      "inflation_layer.cost_scaling_factor": treatment["cost_scaling_factor"]}
        if navigation is not None and "scan" in navigation:
            expected[f"/{name}/{name}"].update({"obstacle_layer.scan." + key: value for key, value in navigation["scan"].items()})
    return expected
