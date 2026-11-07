"""Checked map/world coordinates and declared AMCL settings for the local pilot.

Ground truth is used for validation and RViz labels only, never for localization.
"""
import copy
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

PROFILE = "warehouse_sync_v1"
INITIAL_COVARIANCE = {"x": .0004, "y": .0004, "yaw": .0001}
AMCL = dict(alpha1=.001, alpha2=.001, alpha3=.001, alpha4=.001, alpha5=.001,
            laser_model_type="likelihood_field_prob", do_beamskip=True,
            beam_skip_distance=.5, beam_skip_threshold=.3, beam_skip_error_threshold=.9)
GOAL_CHECKER = dict(stateful=False, xy_goal_tolerance=.05, yaw_goal_tolerance=.15)


def configure_navigation(config):
    result = copy.deepcopy(config)
    result["amcl"]["ros__parameters"].update(AMCL)
    result["controller_server"]["ros__parameters"]["goal_checker"].update(GOAL_CHECKER)
    return result


def parameter_contract():
    return {"/amcl": AMCL,
            "/controller_server": {"goal_checker." + k: v for k, v in GOAL_CHECKER.items()}}


def check_map_world(world_path, map_path, layout_path):
    """Reject a different map, origin, or static warehouse before any movement."""
    import json
    import numpy as np
    from PIL import Image
    import yaml

    world_path, map_path, layout_path = map(Path, [world_path, map_path, layout_path])
    layout = json.loads(layout_path.read_text())
    meta = yaml.safe_load(map_path.read_text())
    image_path = (map_path.parent / meta["image"]).resolve()
    grid = np.asarray(Image.open(image_path))
    if meta.get("negate", 0) != 0 or meta.get("mode", "trinary") != "trinary" or meta["origin"][2] != 0:
        raise ValueError("Warehouse synchronization requires its native unrotated trinary map")
    resolution = meta["resolution"]
    x = meta["origin"][0] + (np.arange(grid.shape[1]) + .5) * resolution
    y = meta["origin"][1] + (grid.shape[0] - np.arange(grid.shape[0]) - .5) * resolution
    expected = np.zeros(grid.shape, dtype=bool)
    world = ET.parse(world_path).getroot().find("world")
    for box in layout["obstacles"]:
        model = world.find("model[@name='%s']" % box["name"])
        if model is None:
            raise ValueError("Map obstacle absent from Gazebo: " + box["name"])
        pose = [float(v) for v in model.findtext("pose").split()]
        size = [float(v) for v in model.findtext("link/collision/geometry/box/size").split()]
        if any(abs(a - b) > 1e-9 for a, b in zip(pose[:2] + size[:2], [box["x"], box["y"], box["sx"], box["sy"]])) or any(abs(v) > 1e-9 for v in pose[3:]):
            raise ValueError("Map/Gazebo obstacle geometry mismatch: " + box["name"])
        expected |= ((np.abs(x[None, :] - box["x"]) <= box["sx"] / 2)
                     & (np.abs(y[:, None] - box["y"]) <= box["sy"] / 2))
    occupied = (255 - grid.astype(float)) / 255 > meta["occupied_thresh"]
    mismatches = int(np.count_nonzero(occupied != expected))
    if mismatches:
        raise ValueError(f"Map/Gazebo raster mismatch: {mismatches} cells; refusing to start")
    digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    return dict(profile=PROFILE, map_frame="map", gazebo_frame="world",
                world_to_map=dict(x=0., y=0., yaw=0.), occupied_cell_mismatches=mismatches,
                static_obstacles=len(layout["obstacles"]), resolution=resolution, origin=meta["origin"],
                world_sha256=digest(world_path), map_yaml_sha256=digest(map_path),
                map_image_sha256=digest(image_path), ground_truth_used_by_navigation=False)


def synchronized_pose_sample(tfbuffer, rclpy, scene, yaw):
    """Compare TF and Gazebo at the same simulation timestamp, without feedback."""
    sample_time = rclpy.time.Time(nanoseconds=round(scene["time"] * 1e9))
    base = tfbuffer.lookup_transform("map", "base_footprint", sample_time)
    odom = tfbuffer.lookup_transform("map", "odom", sample_time)
    def pose(value):
        return dict(x=value.translation.x, y=value.translation.y, yaw=yaw(value.rotation))
    estimate = pose(base.transform)
    truth = scene["poses"]["mecanum_robot"]
    return dict(time=scene["time"], map_base=estimate, map_odom=pose(odom.transform),
                gazebo_robot=truth, comparison="same_simulation_timestamp",
                xy_error_m=math.hypot(estimate["x"] - truth["x"], estimate["y"] - truth["y"]),
                yaw_error_rad=abs(math.remainder(estimate["yaw"] - truth["yaw"], 2 * math.pi)))
