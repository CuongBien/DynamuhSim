"""Exact integer schedule for one JuPedSim prediction per human callback."""

from dataclasses import dataclass

from narrow_nav.core.errors import ContractError
from narrow_nav.core.types import NS_PER_SECOND
from narrow_nav.core.validation import integer

INTERNAL_DT_NS = 10_000_000


@dataclass(frozen=True, slots=True)
class BridgeClock:
    human_dt_ns: int
    internal_dt_ns: int = INTERNAL_DT_NS

    def __post_init__(self) -> None:
        integer(self.human_dt_ns, "human_dt_ns", minimum=1)
        integer(self.internal_dt_ns, "internal_dt_ns", minimum=1)
        if self.human_dt_ns % self.internal_dt_ns:
            raise ContractError("internal_dt_ns must divide human_dt_ns")

    @property
    def iterations_per_callback(self) -> int:
        return self.human_dt_ns // self.internal_dt_ns

    @property
    def internal_dt_s(self) -> float:
        return self.internal_dt_ns / NS_PER_SECOND

    def iterations_at(self, time_ns: int) -> int:
        integer(time_ns, "bridge time_ns")
        if time_ns % self.human_dt_ns:
            raise ContractError("bridge time must lie on the human callback grid")
        return time_ns // self.internal_dt_ns
