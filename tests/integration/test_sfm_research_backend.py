from dataclasses import replace

import pytest

from narrow_nav.core.errors import ConfigurationError
from narrow_nav.humans.registry import create_human_backend
from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.mujoco_engine import MuJoCoEngine

try:
    import jupedsim
except ImportError:
    jupedsim = None

EXPERIMENTAL = jupedsim is not None and hasattr(
    jupedsim.Simulation, "apply_sfm_coupled_state"
)


@pytest.mark.skipif(EXPERIMENTAL, reason="stock interpreter gate")
def test_stock_sfm_cannot_fall_back_to_an_autonomous_proxy():
    with pytest.raises(
        ConfigurationError, match=r"1.4.2\+task04s external/rebase APIs"
    ):
        create_human_backend("sfm_jupedsim")


@pytest.mark.skipif(not EXPERIMENTAL, reason="requires the verified 04S build")
@pytest.mark.parametrize(("period", "native_steps"), [(100, 10), (50, 5)])
def test_native_sfm_registry_runtime_and_reset(period, native_steps):
    config = compose_config(
        overrides=(
            "humans=sfm_jupedsim",
            "simulation.human_visuals=proxy",
            f"simulation.human_control_dt_ns={period * 1_000_000}",
        )
    )
    backend = create_human_backend(config.humans.backend)
    engine = MuJoCoEngine(human_backend=backend)
    runs = []
    try:
        engine.load(config)
        for _ in range(2):
            engine.reset(seed=11)
            assert backend.last_sync is None
            trajectory, callbacks = [], []
            for _ in range(8):
                before = engine.get_ground_truth()
                step = engine.stop()
                sample = backend.last_sync
                if sample.time_ns == before.stamp.timestamp.time_ns:
                    callbacks.append(sample)
                    (pedestrian,) = sample.pedestrians
                    pose = before.humans[0].pose
                    assert pedestrian.position == (pose.x, pose.y)
                    assert (
                        sample.iteration_after - sample.iteration_before == native_steps
                    )
                    assert sample.iteration_before * 10_000_000 == sample.time_ns
                assert step.clearance.human_wall_m > 0
                assert step.human_motion_blocked_substeps == 0
                trajectory.append(step.ground_truth.humans)
            assert len(callbacks) == 400 // period
            assert len({sample.robot_native_id for sample in callbacks}) == 1
            assert (
                len({sample.pedestrians[0].native_identity for sample in callbacks})
                == 1
            )
            assert trajectory[-1][0].pose.x < trajectory[0][0].pose.x
            metadata = engine.get_metadata()
            assert metadata.human_backend_id == "sfm_jupedsim"
            assert metadata.human_backend_version.endswith(backend.build.binary_sha256)
            runs.append((trajectory, callbacks, metadata.episode.episode_id))
        assert runs[0][0] == runs[1][0]
        assert runs[0][2] != runs[1][2]
        assert runs[0][1][0].robot_native_id != runs[1][1][0].robot_native_id
        assert [
            replace(
                sample,
                robot_native_id=0,
                pedestrians=tuple(
                    replace(p, native_identity=(0, 0, 0)) for p in sample.pedestrians
                ),
            )
            for sample in runs[0][1]
        ] == [
            replace(
                sample,
                robot_native_id=0,
                pedestrians=tuple(
                    replace(p, native_identity=(0, 0, 0)) for p in sample.pedestrians
                ),
            )
            for sample in runs[1][1]
        ]
    finally:
        engine.close()


@pytest.mark.skipif(not EXPERIMENTAL, reason="requires the verified 04S build")
def test_narrow_replay_retains_overlap_guard_and_contact_under_research_backend():
    from experiments.social_navigation.evaluate_sfm import run_case

    case = run_case("wall_04s_100")
    assert case["runtime_error"] is None
    assert case["metrics"]["verdict"] == "FAIL"
    assert case["side_by_side_slack_m"] < 0
    assert case["metrics"]["collision_samples_by_type"] == {"robot_human": 610}
    assert case["metrics"]["collision_episodes_by_type"] == {"robot_human": 1}
    assert case["metrics"]["guard_blocked_substeps"] == 822
    assert case["metrics"]["minimum_clearance_m"]["robot_human_m"] == pytest.approx(
        -0.06209206331455408
    )
    assert case["sync"][-1]["iteration_after"] == 300
