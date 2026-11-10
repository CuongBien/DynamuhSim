"""Regression evidence for the native SFM coupling gate, without a registered backend."""

from collections import Counter
from importlib.metadata import version
from math import dist

import pytest

from narrow_nav.core.errors import ConfigurationError
from narrow_nav.simulation.configuration import compose_config

pytest.importorskip("jupedsim")

from experiments.social_navigation.probe_jupedsim_coupling import (
    ACTIONS,
    COMMON_OVERRIDES,
    GATE_STATUS,
    ROOT,
    probe_contact_reference,
    probe_public_api,
    probe_runtime,
)


@pytest.fixture(scope="module")
def native_api():
    return probe_public_api()


@pytest.fixture(scope="module")
def action_runs():
    return {policy: probe_runtime("head_on", policy, 11) for policy in ACTIONS}


@pytest.mark.skipif(
    version("jupedsim") != "1.4.2", reason="04R stock wheel fingerprint"
)
def test_public_surface_and_replacement_preserve_negative_evidence(native_api):
    assert native_api["model"] == "SocialForceModel"
    assert native_api["jupedsim"] == "1.4.2"
    assert native_api["native_pose_setters"] == {
        "position": False,
        "orientation": False,
    }
    assert set(native_api["position_orientation_write_errors"]) == {
        "position",
        "orientation",
    }
    assert native_api["wrapper_velocity_assignment_native_readback"] == (0, 0)
    assert native_api["model_velocity_setter_readback"] == (0.2, -0.1)

    removal = native_api["removal"]
    assert removal["old_visible_before_iterate"]
    assert "too close" in removal["duplicate_error"]
    assert "unknown Agent" in removal["old_lookup_after_iterate_error"]
    assert (removal["before"], removal["after"]) == (0, 1)
    assert removal["native_id_changed"]
    assert "too close" in native_api["insertion_overlap_errors"]["0"]
    assert "too close" in native_api["insertion_overlap_errors"]["0.1"]
    assert native_api["insertion_overlap_errors"]["0.35"] is None

    switch = native_api["geometry_switch"]
    assert switch["static_holes"] == switch["reentry_holes"] == 1
    assert "agents would be outside" in switch["errors"]["agent"]
    assert "stages would be outside" in switch["errors"]["waypoint"]
    assert switch["native_iterations"] == 0


def test_native_sfm_velocity_response_is_overlap_friction_not_anticipation(native_api):
    pairs = native_api["pairs"]
    assert (
        pairs["separated_yield"]["human_velocity"]
        == pairs["separated_stop"]["human_velocity"]
    )
    assert (
        pairs["overlap_reverse"]["human_velocity"]
        == pairs["overlap_stop"]["human_velocity"]
    )
    assert pairs["overlap_yield"]["human_velocity"][1] > 0.5
    assert pairs["separated_stop"]["robot_shadow_error_vs_linear_m"] > 0
    assert pairs["autonomous_drift"]["robot_shadow_error_vs_linear_m"] > 0.009
    assert pairs["autonomous_drift"]["robot_shadow_velocity"] != (0.35, 0)
    continuity = native_api["isolated_reconstruction"]
    assert continuity == {
        "max_xy_error_m": 0,
        "max_velocity_error_mps": 0,
        "continuous_iterations": 100,
        "distinct_rebuild_ids": 10,
    }


def _assert_command_windows(case):
    trace = {tick["start_time_ns"]: tick for tick in case["trace"]}
    for index, sample in enumerate(case["samples"]):
        time_ns = index * 100_000_000
        assert sample["time_ns"] == time_ns
        assert sample["native_iterations_before"] == index * 10
        if "error" in sample:
            assert time_ns not in trace
            continue
        assert sample["local_iteration_before"] == 0
        assert sample["local_iteration_after"] == 10
        assert sample["native_iterations_after"] == (index + 1) * 10
        robot_pose, robot_velocity = sample["robot_inserted_pose_velocity"]
        assert robot_pose == sample["robot_measured_xy"]
        assert robot_velocity == sample["robot_measured_velocity_xy"]
        ticks = [trace[time_ns], trace[time_ns + 50_000_000]]
        for entity_id, predicted in sample["human_predictions"].items():
            first, second = [tick["humans"][entity_id] for tick in ticks]
            assert predicted["initial_xy"] == first["start_xy"]
            assert predicted["executed_xy"] == second["end_xy"]
            assert first["requested"] == second["requested"]
            assert first["limited"] == second["limited"]
            for axis in range(2):
                assert first["requested"][axis] * 0.1 == pytest.approx(
                    predicted["predicted_xy"][axis] - first["start_xy"][axis], abs=1e-12
                )
                assert sum(
                    tick["executed"][axis] * 0.05 for tick in (first, second)
                ) == pytest.approx(
                    second["end_xy"][axis] - first["start_xy"][axis], abs=1e-12
                )


def test_measured_robot_action_and_sfm_motion_are_logged_separately(action_runs):
    reference = action_runs["advance"]
    assert reference["samples"][0]["robot_measured_velocity_xy"] == (0, 0)
    assert reference["robot_ticks"] == 60 and reference["native_iterations"] == 300
    assert reference["metadata"]["clocks"]["physics_steps"] == 1500
    limited_tick = next(
        tick for tick in reference["trace"] if tick["start_time_ns"] == 1_500_000_000
    )
    human = limited_tick["humans"]["human_01"]
    assert dist(human["requested"], human["limited"]) > 0.01
    for policy, case in action_runs.items():
        assert case["status"] == GATE_STATUS
        assert case["failure"] is None
        assert case["trace"][:14] == reference["trace"][:14]
        assert case["trace"][14]["robot_requested_body"] == ACTIONS[policy]
        _assert_command_windows(case)
    stop, advance, yielding, reverse = (
        action_runs[policy]["trace"][-1]["robot_world_velocity"]
        for policy in ("stop", "advance", "yield", "reverse")
    )
    assert abs(stop[0]) < 0.02 and advance[0] > 0.3
    assert yielding[1] < -0.18 and reverse[0] < -0.2
    assert any(
        dist(a["humans"]["human_01"]["requested"], b["humans"]["human_01"]["requested"])
        > 0.1
        for a, b in zip(reference["trace"], action_runs["yield"]["trace"], strict=True)
    )
    assert (
        max(sample["robot_shadow_vs_mujoco_m"] for sample in reference["samples"])
        > 0.02
    )


def test_probe_replay_and_distinct_episode_seeds(action_runs):
    reference = action_runs["advance"]
    same = probe_runtime("head_on", "advance", 11)
    other = probe_runtime("head_on", "advance", 12)
    assert same["trace"] == other["trace"] == reference["trace"]
    metadata = [case["metadata"] for case in (reference, same, other)]
    assert len({entry["episode"]["run_id"] for entry in metadata}) == 3
    assert metadata[0]["human_backend_seed"] == metadata[1]["human_backend_seed"]
    assert metadata[0]["human_backend_seed"] != metadata[2]["human_backend_seed"]


@pytest.mark.parametrize("fixture", ("straight", "crossing", "waypoint"))
def test_other_native_probe_geometries(fixture):
    case = probe_runtime(fixture, "advance", 11)
    _assert_command_windows(case)
    assert case["status"] == GATE_STATUS
    assert case["robot_ticks"] == 60
    assert all(step["clearance"]["human_wall_m"] > 0 for step in case["trace"])
    if fixture == "crossing":
        vx, vy = case["trace"][-1]["robot_world_velocity"]
        assert abs(vx) < 0.001 and vy > 0.3
    if fixture == "waypoint":
        assert not case["samples"][0]["goal_covered_by_robot"]
        assert not case["samples"][-1]["goal_covered_by_robot"]
        assert any(
            sample["goal_covered_by_robot"] == ["human_01"]
            for sample in case["samples"]
        )


def test_overlap_retains_prefix_guard_discrepancy_and_native_contact_reference():
    with pytest.raises(ConfigurationError, match="initial footprints overlap"):
        compose_config(
            ROOT / "configs",
            (*COMMON_OVERRIDES, "scenarios.robot.initial_pose=[0.1,0.25,0]"),
        )
    case = probe_runtime("narrow_wall", "advance", 11)
    _assert_command_windows(case)
    assert case["status"] == GATE_STATUS
    assert case["failure"] == "observed_robot_human_footprint_overlap"
    assert case["metadata"]["end_reason"] == "human_backend_failed"
    assert case["metadata"]["last_valid_tick"] == case["robot_ticks"]
    assert case["metadata"]["failure_tick"] == case["robot_ticks"] + 1
    assert case["samples"][-1]["human_robot_surface_clearance_m"]["human_01"] < 0
    assert "native_iterations_after" not in case["samples"][-1]
    assert any(
        tick["guard_blocked_substeps"] > 0
        and tick["humans"]["human_01"]["limited"]
        != tick["humans"]["human_01"]["executed"]
        for tick in case["trace"]
    )
    assert any(
        sample["human_predictions"]["human_01"]["state_sync_error_m"] > 1e-4
        for sample in case["samples"][:-1]
    )
    assert not any(tick["contacts"] for tick in case["trace"])

    reference = probe_contact_reference()
    assert reference["metadata"]["human_motion_mode"] == "stationary"
    kinds = Counter(event["kind"] for event in reference["native_contacts"])
    assert kinds["robot_human"] > 0
    assert reference["min_robot_human_clearance_m"] < 0
    assert set(reference["metadata"]["not_observable_solver"]) == {
        "human_wall",
        "human_human",
    }
