from dataclasses import replace

import pytest

from narrow_nav.core.errors import ContractError
from narrow_nav.core.metadata import SnapshotMetadata
from narrow_nav.core.protocols import HumanBackendConfig
from narrow_nav.core.states import HumanContext, HumanState
from narrow_nav.core.types import Point2D, Pose2D, Timestamp, Velocity2D
from narrow_nav.humans.scripted import ScriptedHumanBackend


def test_scripted_goal_hold_and_reset_preserve_order():
    stamp = SnapshotMetadata("fixture:0", 42, Timestamp(0, 0))
    moving = HumanState(
        "human_a", Pose2D(0, 0, 0), Velocity2D(0, 0), Point2D(3, 4), stamp.timestamp
    )
    at_goal = HumanState(
        "human_b", Pose2D(2, 1, 0), Velocity2D(0, 0), Point2D(2, 1), stamp.timestamp
    )
    context = HumanContext(
        stamp,
        "robot",
        Pose2D(-2, 0, 0),
        Velocity2D(0, 0),
        0.25,
        0.25,
        (moving, at_goal),
    )
    backend = ScriptedHumanBackend()
    settings = HumanBackendConfig({"speed_mps": 1}, 100_000_000, 0.1, 1.5)
    with pytest.raises(ContractError, match="reset"):
        backend.propose(context)
    with pytest.raises(ContractError, match="sorted"):
        backend.reset(settings, 42, ("human_b", "human_a"))
    backend.reset(settings, 42, ("human_a", "human_b"))
    first, second = backend.propose(context)
    assert (first.entity_id, second.entity_id) == ("human_a", "human_b")
    assert first.velocity.linear_x == pytest.approx(0.6)
    assert first.velocity.linear_y == pytest.approx(0.8)
    assert second.velocity == Velocity2D(0, 0)
    moved = replace(context, humans=(moving, replace(at_goal, pose=Pose2D(1, 1, 0))))
    assert backend.propose(moved)[1].velocity == Velocity2D(0, 0)
    backend.reset(settings, 42, ("human_a", "human_b"))
    assert backend.propose(moved)[1].velocity.linear_x > 0
