"""Read a native prediction and compare it with the next MuJoCo snapshot."""

from collections.abc import Mapping
from dataclasses import dataclass
from math import hypot

from narrow_nav.core.actions import HumanMotion
from narrow_nav.core.errors import ContractError
from narrow_nav.core.states import HumanContext
from narrow_nav.core.types import NS_PER_SECOND, Point2D, Velocity2D
from narrow_nav.core.validation import positive, unique

from .clock import BridgeClock


@dataclass(frozen=True, slots=True)
class StateSyncError:
    entity_id: str
    predicted_xy_m: tuple[float, float]
    executed_xy_m: tuple[float, float]

    @property
    def position_error_m(self) -> float:
        (px, py), (ex, ey) = self.predicted_xy_m, self.executed_xy_m
        return hypot(px - ex, py - ey)


class StateSync:
    """One episode's prediction stream. Read native XY; never write physical XY."""

    def __init__(
        self,
        simulation,
        clock: BridgeClock,
        agent_ids: Mapping[str, int],
        initial: HumanContext,
        *,
        max_position_error_m: float,
    ) -> None:
        self._simulation = simulation
        self._clock = clock
        self._episode_id = initial.stamp.episode_id
        self._episode_seed = initial.stamp.episode_seed
        self._agent_ids = dict(agent_ids)
        unique(self._agent_ids.values(), "native agent IDs")
        self._max_error = positive(max_position_error_m, "max_position_error_m")
        if (
            initial.stamp.timestamp.time_ns != 0
            or simulation.iteration_count() != 0
            or simulation.delta_time() != clock.internal_dt_s
        ):
            raise ContractError(
                "state sync requires a fresh simulation at the configured dt"
            )
        if set(self._agent_ids.values()) != {agent.id for agent in simulation.agents()}:
            raise ContractError(
                "native simulation must contain only the mapped human agents"
            )
        self.last_sync: tuple[StateSyncError, ...] = ()
        self.compare(initial)
        if any(sample.position_error_m > 1e-9 for sample in self.last_sync):
            raise ContractError(
                "native initial positions must match the measured spawns"
            )

    def compare(self, context: HumanContext) -> tuple[StateSyncError, ...]:
        """Compare paired states at one boundary without advancing either engine."""
        if (context.stamp.episode_id, context.stamp.episode_seed) != (
            self._episode_id,
            self._episode_seed,
        ):
            raise ContractError("reset the bridge before changing episodes")
        if tuple(human.entity_id for human in context.humans) != tuple(
            sorted(self._agent_ids)
        ):
            raise ContractError("human roster must match the episode's native agents")
        time_ns = context.stamp.timestamp.time_ns
        if self._simulation.iteration_count() != self._clock.iterations_at(time_ns):
            raise ContractError(
                "JuPedSim iteration count does not match the snapshot boundary"
            )
        self.last_sync = tuple(
            StateSyncError(
                human.entity_id,
                self._position(human.entity_id),
                (human.pose.x, human.pose.y),
            )
            for human in context.humans
        )
        for sample in self.last_sync:
            if sample.position_error_m > self._max_error:
                raise ContractError(
                    f"state_sync_error for {sample.entity_id} at {time_ns} ns: "
                    f"{sample.position_error_m:.6g} m exceeds {self._max_error:g} m"
                )
        return self.last_sync

    def _position(self, entity_id: str) -> tuple[float, float]:
        # Point2D validation ensures NaN/Inf never pass the sync tolerance.
        point = Point2D(*self._simulation.agent(self._agent_ids[entity_id]).position)
        return point.x, point.y

    def propose(self, context: HumanContext) -> tuple[HumanMotion, ...]:
        before = self.compare(context)
        stamp = context.stamp.timestamp
        self._simulation.iterate(self._clock.iterations_per_callback)
        expected = self._clock.iterations_at(stamp.time_ns + self._clock.human_dt_ns)
        if self._simulation.iteration_count() != expected:
            raise ContractError("JuPedSim advanced an unexpected number of iterations")
        dt = self._clock.human_dt_ns / NS_PER_SECOND
        motions = []
        for sample in before:
            px, py = sample.predicted_xy_m
            x, y = self._position(sample.entity_id)
            motions.append(
                HumanMotion(
                    sample.entity_id, Velocity2D((x - px) / dt, (y - py) / dt), stamp
                )
            )
        return tuple(motions)
