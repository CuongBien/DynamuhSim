from dataclasses import replace
from hashlib import sha256
from json import loads
from pathlib import Path
from xml.etree import ElementTree

import mujoco
import numpy as np
import pytest

from narrow_nav.core.actions import RobotAction
from narrow_nav.core.errors import ConfigurationError
from narrow_nav.core.seed import derive_seed
from narrow_nav.core.types import Twist2D
from narrow_nav.humans.registry import create_human_backend
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.d435i_noise import D435iNoiseModel
from narrow_nav.simulation.depth_config import depth_from_config
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine
from narrow_nav.simulation.record_depth import record_depth_episode
from narrow_nav.simulation.world_builder import build_world

_SCENE = (
    "robots=mecanum_robot",
    "sensors=d435i",
    "worlds=corridor_090",
    "scenarios=corridor_090",
    "simulation.human_visuals=proxy",
    "experiments.duration_ns=100000000",
)


@pytest.fixture
def egl(monkeypatch):
    module = pytest.importorskip("mujoco.egl", exc_type=ImportError)
    try:
        context = module.GLContext(16, 16)
        context.make_current()
        context.free()
    except (ImportError, RuntimeError) as exc:
        pytest.skip(f"headless MuJoCo EGL context unavailable: {exc}")
    monkeypatch.setattr(mujoco, "GLContext", module.GLContext)
    monkeypatch.setenv("MUJOCO_GL", "egl")


def test_calibration_matches_the_rendered_optical_projection():
    world = build_world(compose_config(overrides=_SCENE))
    calibration = world.depth_camera.calibration
    model = mujoco.MjModel.from_xml_string(world.mjcf)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
    camera.fixedcamid = model.camera("robot/depth_camera").id
    frustum = mujoco.mjv_camera2GLCamera(model, data, camera)
    near = frustum.frustum_near
    fx = near * calibration.width / (2 * frustum.frustum_width)
    fy = near * calibration.height / (frustum.frustum_top - frustum.frustum_bottom)
    cx = (calibration.width - 1) / 2 - frustum.frustum_center * fx / near
    cy = frustum.frustum_top * fy / near - 0.5
    assert (fx, fy, cx, cy) == pytest.approx(
        (calibration.fx_px, calibration.fy_px, calibration.cx_px, calibration.cy_px),
        abs=1e-4,
    )
    assert model.cam_ipd[camera.fixedcamid] == pytest.approx(calibration.baseline_m)


def test_sensor_profile_rejects_invalid_configuration():
    with pytest.raises(ConfigurationError, match="requires robots=mecanum_robot"):
        compose_config(overrides=("sensors=d435i",))
    with pytest.raises(ConfigurationError, match="temporal_correlation"):
        compose_config(overrides=(*_SCENE, "sensors.noise.temporal_correlation=1"))
    with pytest.raises(ConfigurationError, match="divisible by physics_dt_ns"):
        compose_config(overrides=(*_SCENE, "simulation.observation_dt_ns=33000000"))


def test_disparity_noise_grows_with_range_and_keeps_invalid_pixels():
    profile = depth_from_config(compose_config(overrides=_SCENE))
    cfg = replace(profile.noise, hole_probability=0.0, temporal_sigma_px=0.0)
    model = D435iNoiseModel(profile.calibration, cfg)
    shape = (profile.calibration.height, profile.calibration.width)
    rng_seed = 11
    rms = []
    for distance in (1.0, 4.0):
        clean = np.full(shape, distance, dtype=np.float32)
        clean[0, 0:3] = (0, np.nan, cfg.max_depth_m + 1)
        model.reset(rng_seed)
        noisy, mask = model.apply(clean)
        assert not mask[0, :3].any()
        assert (noisy[~mask] == 0).all()
        # The finite right imager loses a disparity-wide strip on the left.
        assert not mask[:, 0].any()
        rms.append(np.sqrt(np.mean((noisy[mask] - distance) ** 2)))
    assert rms[1] > 10 * rms[0]

    clean = np.full(shape, 2.0, dtype=np.float32)
    clean[0, 0] = 0
    ideal = D435iNoiseModel(profile.calibration, replace(cfg, enabled=False))
    measured, mask = ideal.apply(clean)
    np.testing.assert_array_equal(measured, clean)
    np.testing.assert_array_equal(mask, clean > 0)


def test_runtime_replays_seeds_and_keeps_samples_on_the_physics_grid(egl):
    engine = MuJoCoEngine()
    try:
        engine.load(
            compose_config(overrides=(*_SCENE, "simulation.observation_dt_ns=20000000"))
        )

        def capture(seed):
            observation = engine.reset(seed=seed)
            sample = next(
                slot.sample for slot in observation.sensors if slot.sensor_id == "depth"
            )
            assert sample.source == "d435i_noisy"
            assert sample.payload is not engine.get_clean_depth()[0].payload
            samples, truth = [sample], [engine.get_clean_depth()[0]]
            while engine.get_metadata().status == "running":
                result = engine.stop()
                samples.extend(result.sensor_samples)
                truth.extend(engine.get_clean_depth())
                assert (
                    next(
                        slot.sample
                        for slot in result.observation.sensors
                        if slot.sensor_id == "depth"
                    )
                    is samples[-1]
                )
            return samples, truth

        first, clean = capture(42)
        replay, replay_clean = capture(42)
        alternate, alternate_clean = capture(43)
        assert first == replay
        assert clean == replay_clean == alternate_clean
        assert first[0].payload != alternate[0].payload
        assert [frame.capture_time_ns for frame in first] == list(
            range(0, 100000001, 20000000)
        )
        assert [frame.available_time_ns for frame in first] == [
            frame.capture_time_ns for frame in first
        ]

        engine.load(
            compose_config(overrides=(*_SCENE, "simulation.observation_dt_ns=80000000"))
        )
        reset = engine.reset()
        step = engine.stop()
        assert step.observation.sensors == reset.sensors
        assert step.sensor_samples == () and engine.get_clean_depth() == ()
        assert engine.stop().sensor_samples[0].capture_time_ns == 80000000
    finally:
        engine.close()


def test_robot_human_and_depth_share_one_runtime_and_replay(egl):
    config = compose_config(
        overrides=(
            "robots=mecanum_robot",
            "sensors=d435i",
            "humans=scripted",
            "scenarios.humans.0.initial_pose=[0,-0.2,-3.141592653589793]",
            "scenarios.humans.0.goal=[-2,-0.2]",
            "simulation.observation_dt_ns=20000000",
            "experiments.duration_ns=200000000",
        )
    )
    engine = MuJoCoEngine(human_backend=create_human_backend(config.humans.backend))
    try:
        engine.load(config)
        replays = []
        for _ in range(2):
            observation = engine.reset(seed=71)
            start = engine.get_ground_truth()
            metadata = engine.get_metadata()
            stream = metadata.depth_stream
            assert metadata.human_backend_id == "scripted"
            assert metadata.human_backend_seed == derive_seed(71, "human_backend")
            assert stream.seed == derive_seed(71, "sensor", "sensor.depth")
            assert "depth" not in metadata.unavailable_sensors
            assert observation.stamp == start.stamp
            (depth_slot,) = (
                slot for slot in observation.sensors if slot.sensor_id == "depth"
            )
            assert depth_slot.status == "valid"
            samples, clean_frames = [depth_slot.sample], list(engine.get_clean_depth())
            poses = []

            for tick in range(4):
                step = engine.step(
                    RobotAction(
                        start.robot.entity_id, Twist2D(0.3, 0, 0), start.robot.timestamp
                    )
                )
                end = step.ground_truth
                (motion,) = step.human_motion_results
                assert motion.requested.entity_id == start.humans[0].entity_id
                assert motion.requested.timestamp == start.stamp.timestamp
                assert start.stamp.timestamp.time_ns == tick * 50_000_000
                assert step.observation.stamp == end.stamp
                assert end.stamp.timestamp.time_ns == (tick + 1) * 50_000_000
                assert (
                    motion.requested.velocity.linear_x
                    < motion.limited.velocity.linear_x
                    < 0
                )
                assert motion.executed.velocity.linear_x == pytest.approx(
                    motion.limited.velocity.linear_x, abs=1e-9
                )
                assert end.humans[0].pose.x - start.humans[0].pose.x == pytest.approx(
                    motion.limited.velocity.linear_x * 0.05, abs=1e-9
                )
                assert step.human_motion_blocked_substeps == 0
                assert step.clearance.human_wall_m > 0
                assert step.contacts == ()
                (depth_slot,) = (
                    slot
                    for slot in step.observation.sensors
                    if slot.sensor_id == "depth"
                )
                assert depth_slot.sample is step.sensor_samples[-1]
                samples.extend(step.sensor_samples)
                clean_frames.extend(engine.get_clean_depth())
                poses.append((end.robot.pose, end.humans[0].pose))
                start = end

            assert start.robot.pose.x > -3.0
            assert [sample.capture_time_ns for sample in samples] == list(
                range(0, 200_000_001, 20_000_000)
            )
            shape = (
                stream.profile.calibration.height,
                stream.profile.calibration.width,
            )
            for sample, clean in zip(samples, clean_frames, strict=True):
                assert sample.sample_index == clean.sample_index
                assert (
                    sample.capture_time_ns
                    == clean.capture_time_ns
                    == sample.available_time_ns
                )
                assert sample.frame_id == clean.frame_id == stream.frame_id
                assert sample.source == "d435i_noisy"
                assert sample.payload.shape == clean.payload.shape == shape
                assert any(sample.payload.valid_mask) and any(clean.payload.valid_mask)
                assert sample.payload != clean.payload
            assert engine.get_metadata().status == "truncated"
            replays.append((poses, samples, clean_frames))
        assert replays[0] == replays[1]
    finally:
        engine.close()


def test_recorded_dataset_separates_clean_depth_and_matches_policy_input(
    egl, tmp_path: Path
):
    config = compose_config(
        overrides=(*_SCENE, "simulation.observation_dt_ns=20000000")
    )
    run = record_depth_episode(config, tmp_path, report=True)
    manifest = loads((run / "manifest.json").read_text())
    assert manifest["status"] == "truncated"
    assert manifest["depth_frame_count"] == 6
    assert (
        manifest["episode"]["config_sha256"]
        == sha256((run / "resolved.yaml").read_bytes()).hexdigest()
    )
    assert manifest["depth_stream"]["seed"] == derive_seed(42, "sensor", "sensor.depth")
    assert manifest["depth_stream"]["profile"]["noise"]["enabled"]
    assert manifest["depth_stream"]["invalid_value"] == 0

    boundaries = [
        loads(line) for line in (run / "observations.jsonl").read_text().splitlines()
    ]
    assert [entry["new_depth_indices"] for entry in boundaries] == [
        [0],
        [1, 2],
        [3, 4, 5],
    ]
    assert "clean" not in (run / "observations.jsonl").read_text()
    assert {p.name for p in (run / "observations").iterdir()} == {
        f"depth_{index:08d}.npz" for index in range(6)
    }
    report = ElementTree.parse(run / "ground_truth" / "diagnostics.svg")
    assert len(report.findall(".//{http://www.w3.org/2000/svg}image")) == 6

    engine = MuJoCoEngine()
    try:
        engine.load(config)
        policy_observation = engine.reset()
        policy_depth = next(
            slot.sample
            for slot in policy_observation.sensors
            if slot.sensor_id == "depth"
        )
        name = "depth_00000000.npz"
        with (
            np.load(run / "observations" / name, allow_pickle=False) as observed,
            np.load(run / "ground_truth" / name, allow_pickle=False) as clean,
        ):
            assert set(observed.files) == {
                "depth_m",
                "valid_mask",
                "sensor_id",
                "source",
                "frame_id",
                "sample_index",
                "capture_time_ns",
                "available_time_ns",
                "schema_version",
                "episode_id",
            }
            expected = np.asarray(
                policy_depth.payload.values, dtype=np.float32
            ).reshape(240, 424)
            np.testing.assert_array_equal(observed["depth_m"], expected)
            np.testing.assert_array_equal(observed["valid_mask"], expected > 0)
            assert not np.array_equal(observed["depth_m"], clean["depth_m"])
            assert (
                int(observed["capture_time_ns"]) == int(clean["capture_time_ns"]) == 0
            )
            assert 0 < observed["valid_mask"].mean() < clean["valid_mask"].mean()
    finally:
        engine.close()
