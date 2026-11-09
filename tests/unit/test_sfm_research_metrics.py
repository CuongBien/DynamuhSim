import pytest

from experiments.social_navigation.sfm_metrics import (
    append_dynamics,
    contact_counts,
    footprint_ttc,
    progress,
)


@pytest.mark.parametrize(
    ("xy", "velocity", "result"),
    [
        ((2.5, 0), (-2, 0), 1.0),
        ((2.5, 0.5), (-2, 0), 1.25),
        ((2.5, 0.7), (-2, 0), None),
        ((2.5, 0), (1, 0), None),
        ((2.5, 0), (0, 0), None),
        ((0.4, 0), (1, 0), 0.0),
    ],
)
def test_ttc_uses_relative_velocity_and_combined_footprint(xy, velocity, result):
    assert footprint_ttc(xy, velocity, 0.5) == result


def test_contact_samples_count_separately_from_contiguous_solver_runs():
    def contact(tick, pair, kind):
        return {
            "time_ns": tick * 2_000_000,
            "first_id": pair[0],
            "second_id": pair[1],
            "kind": kind,
        }

    human, wall = ("human_01", "robot"), ("robot", "wall")
    contacts = [
        contact(1, human, "robot_human"),
        contact(2, human, "robot_human"),
        contact(2, wall, "robot_wall"),
        contact(3, wall, "robot_wall"),
        contact(4, human, "robot_human"),
    ]
    samples, events = contact_counts(contacts, 2_000_000)
    assert samples == {"robot_human": 3, "robot_wall": 2}
    assert events == {"robot_human": 2, "robot_wall": 1}


def test_failure_to_progress_counts_streaks_exceeding_five_seconds():
    assert progress([{"robot_goal_distance_m": 2.0}] * 101, 0.05) == (5.0, 0)
    frozen = [{"robot_goal_distance_m": 2.0}] * 103
    closer = [{"robot_goal_distance_m": 1.0}] * 105
    assert progress(frozen + closer, 0.05) == (5.2, 2)


def test_jerk_measures_changes_in_speed_even_when_velocity_turns():
    previous = None
    for velocity, expected in (
        ((1, 0), None),
        ((0, 1), None),
        ((-1, 0), 0.0),
        ((-2, 0), 4.0),
    ):
        row = {
            f"{actor}_{field}": value
            for actor in ("robot", "human")
            for field, value in (
                ("velocity", velocity),
                ("xy", (0, 0)),
                ("executed", (0, 0)),
            )
        }
        append_dynamics(row, previous, 0.5)
        assert row["robot_jerk_mps3"] == row["human_jerk_mps3"] == expected
        previous = row
