"""ROS-independent episode lifecycle; runtime managers are injected later."""
from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Protocol

from .episode_loader import EpisodeLoader, EpisodeValidationError
from .nav2_manager import Nav2Error
from .episode_types import (
    EpisodeContext, EpisodeHooks, EpisodeResult, EpisodeState,
    EpisodeStatus, EpisodeTermination,
)


class InvalidStateTransition(RuntimeError):
    pass


class SimulatorManager(Protocol):
    def reset(self, context: EpisodeContext) -> None: ...
    def wait_ready(self, context: EpisodeContext) -> None: ...
    def finalize(self, context: EpisodeContext) -> None: ...


class HuNavManager(Protocol):
    def reset(self, context: EpisodeContext) -> None: ...
    def load_scenario(self, context: EpisodeContext) -> None: ...
    def wait_ready(self, context: EpisodeContext) -> None: ...
    def finalize(self, context: EpisodeContext) -> None: ...


class Nav2Manager(Protocol):
    def wait_ready(self, context: EpisodeContext) -> None: ...
    def send_goal(self, context: EpisodeContext) -> None: ...
    def poll(self, context: EpisodeContext): ...
    def cancel_goal(self, context: EpisodeContext) -> None: ...
    def finalize(self, context: EpisodeContext) -> None: ...


class EpisodeMonitor(Protocol):
    def run(self, context: EpisodeContext) -> EpisodeTermination: ...


_NEXT = {
    EpisodeState.PREPARE: EpisodeState.RESET,
    EpisodeState.RESET: EpisodeState.LOAD_SCENARIO,
    EpisodeState.LOAD_SCENARIO: EpisodeState.WAIT_READY,
    EpisodeState.WAIT_READY: EpisodeState.START,
    EpisodeState.START: EpisodeState.RUNNING,
    EpisodeState.RUNNING: EpisodeState.FINALIZE,
}


class EpisodeRunner:
    """Run one episode through fixed states; managers own all ROS operations.

    A manager's wait_ready method must return only when ready or raise on a
    bounded timeout. The runner never treats an absent manager as readiness.
    """

    def __init__(
        self, *, simulator_manager: SimulatorManager | None = None,
        hunav_manager: HuNavManager | None = None,
        nav2_manager: Nav2Manager | None = None,
        monitor: EpisodeMonitor | None = None,
        loader: EpisodeLoader | None = None,
        hooks: EpisodeHooks | None = None,
        logger: logging.Logger | None = None,
    ) -> None:
        self.simulator_manager = simulator_manager
        self.hunav_manager = hunav_manager
        self.nav2_manager = nav2_manager
        self.monitor = monitor
        self.loader = loader or EpisodeLoader()
        self.hooks = hooks or EpisodeHooks()
        self.logger = logger or logging.getLogger(__name__)
        self.state: EpisodeState | None = None
        self.transitions: list[EpisodeState] = []

    def transition(self, next_state: EpisodeState) -> None:
        if not isinstance(next_state, EpisodeState):
            raise InvalidStateTransition(f"Unknown state: {next_state}")
        allowed = (
            {EpisodeState.PREPARE} if self.state is None else
            {_NEXT[self.state], EpisodeState.FINALIZE} if self.state in _NEXT else set()
        )
        if next_state not in allowed:
            raise InvalidStateTransition(f"{self.state} -> {next_state} is not allowed")
        self.logger.info("[%s] -> [%s]", self.state.name if self.state else "IDLE",
                         next_state.name)
        self.state = next_state
        self.transitions.append(next_state)

    def _configured(self) -> None:
        missing = [name for name in (
            "simulator_manager", "hunav_manager", "nav2_manager", "monitor"
        ) if getattr(self, name) is None]
        if missing:
            raise RuntimeError(f"Runtime managers not configured: {', '.join(missing)}")

    def run(self, episode_dir: Path) -> EpisodeResult:
        """Return a result for validation/runtime failures, then finalize once."""
        self.state = None
        self.transitions = []
        context: EpisodeContext | None = None
        status = EpisodeStatus.SIM_FAILURE
        reason = "episode did not start"
        started = time.monotonic()
        self.transition(EpisodeState.PREPARE)
        try:
            context = self.loader.load(episode_dir)
            self.logger.info(
                "episode=%s family=%s seed=%s density=%s start=%s goal=%s humans=%s",
                context.episode_id, context.scenario_family, context.seed,
                context.metadata.get("density"), context.start_pose,
                context.goal_node, context.metadata["humans"]["total"],
            )
            self.hooks.on_episode_prepare(context)
            self._configured()

            self.transition(EpisodeState.RESET)
            self.simulator_manager.reset(context)

            self.transition(EpisodeState.LOAD_SCENARIO)
            self.hunav_manager.load_scenario(context)

            self.transition(EpisodeState.WAIT_READY)
            self.simulator_manager.wait_ready(context)
            self.hunav_manager.wait_ready(context)
            self.nav2_manager.wait_ready(context)

            self.transition(EpisodeState.START)
            self.nav2_manager.send_goal(context)
            self.hooks.on_episode_start(context)

            self.transition(EpisodeState.RUNNING)
            termination = self.monitor.run(context)
            if not isinstance(termination, EpisodeTermination) or not isinstance(
                termination.status, EpisodeStatus
            ) or not termination.reason:
                raise RuntimeError("Monitor returned an invalid termination")
            status, reason = termination.status, termination.reason
        except EpisodeValidationError as exc:
            status, reason = EpisodeStatus.INVALID_EPISODE, str(exc)
            self.logger.error("Invalid episode: %s", exc)
        except Nav2Error as exc:
            status = (EpisodeStatus.NAV_FAILURE if exc.code in
                      {"goal_rejected", "goal_send_failed", "goal_aborted"}
                      else EpisodeStatus.SIM_FAILURE)
            reason = str(exc)
            self.logger.error("Nav2 failed: %s", exc)
        except Exception as exc:
            status, reason = EpisodeStatus.SIM_FAILURE, f"{type(exc).__name__}: {exc}"
            self.logger.exception("Episode infrastructure failed")
        finally:
            self.transition(EpisodeState.FINALIZE)
            if context is not None:
                for name in ("nav2_manager", "hunav_manager", "simulator_manager"):
                    manager = getattr(self, name)
                    if manager is None:
                        continue
                    try:
                        manager.finalize(context)
                    except Exception as exc:
                        self.logger.exception("%s finalize failed", name)
                        if status is EpisodeStatus.SUCCESS:
                            status = EpisodeStatus.SIM_FAILURE
                            reason = f"{name} finalize failed: {exc}"
            result = EpisodeResult(
                episode_id=context.episode_id if context else Path(episode_dir).name,
                status=status, termination_reason=reason,
                duration_sec=time.monotonic() - started,
                transitions=tuple(self.transitions), context=context,
            )
            try:
                self.hooks.on_episode_end(context, result)
            except Exception as exc:
                self.logger.exception("on_episode_end failed")
                if status is EpisodeStatus.SUCCESS:
                    result = EpisodeResult(
                        episode_id=result.episode_id, status=EpisodeStatus.SIM_FAILURE,
                        termination_reason=f"on_episode_end failed: {exc}",
                        duration_sec=result.duration_sec,
                        transitions=result.transitions, context=context,
                    )
            self.logger.info("[TERMINATED: %s] %s", result.status.value.upper(),
                             result.termination_reason)
        return result
