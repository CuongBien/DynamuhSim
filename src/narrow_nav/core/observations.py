"""Time-stamped sensor measurements exposed to robot policies."""

from dataclasses import dataclass
from math import prod
from typing import Literal

from .errors import ContractError
from .metadata import SnapshotMetadata
from .validation import finite, identifier, integer, ordered_ids, require_type

SensorStatus = Literal["valid", "invalid"]
SlotStatus = Literal["valid", "missing", "invalid", "stale"]


@dataclass(frozen=True, slots=True)
class SensorPayload:
    """Flat row-major values and optional mask in the source's declared units."""

    shape: tuple[int, ...]
    values: tuple[float, ...]
    valid_mask: tuple[bool, ...] | None = None

    def __post_init__(self) -> None:
        require_type(self.shape, tuple, "shape")
        require_type(self.values, tuple, "values")
        if not self.shape:
            raise ContractError("sensor payload must have at least one dimension")
        for dimension in self.shape:
            integer(dimension, "shape dimension", minimum=1)
        if prod(self.shape) != len(self.values):
            raise ContractError("sensor values must match shape")
        for value in self.values:
            finite(value, "sensor value")
        if self.valid_mask is not None:
            require_type(self.valid_mask, tuple, "valid_mask")
            if len(self.valid_mask) != len(self.values):
                raise ContractError("valid_mask must match shape")
            for valid in self.valid_mask:
                require_type(valid, bool, "mask value")


@dataclass(frozen=True, slots=True)
class SensorObservation:
    sensor_id: str
    source: str
    frame_id: str
    sample_index: int
    capture_time_ns: int
    available_time_ns: int
    status: SensorStatus = "valid"
    payload: SensorPayload | None = None
    reason: str | None = None

    def __post_init__(self) -> None:
        identifier(self.sensor_id, "sensor_id")
        identifier(self.source, "source")
        identifier(self.frame_id, "frame_id")
        integer(self.sample_index, "sample_index")
        integer(self.capture_time_ns, "capture_time_ns")
        integer(
            self.available_time_ns, "available_time_ns", minimum=self.capture_time_ns
        )
        if self.status == "valid":
            require_type(self.payload, SensorPayload, "payload")
            if self.reason is not None:
                raise ContractError("valid sensor sample cannot have a reason")
        elif self.status == "invalid" and self.payload is None:
            identifier(self.reason, "reason")
        else:
            raise ContractError(
                "sensor sample must be valid or invalid without payload"
            )


@dataclass(frozen=True, slots=True)
class SensorSlot:
    sensor_id: str
    status: SlotStatus
    sample: SensorObservation | None = None
    reason: str | None = None

    def __post_init__(self) -> None:
        identifier(self.sensor_id, "sensor_id")
        if self.status == "valid":
            sample = require_type(self.sample, SensorObservation, "sample")
            if sample.status != "valid" or sample.sensor_id != self.sensor_id:
                raise ContractError(
                    "slot must contain a valid sample from the same sensor"
                )
            if self.reason is not None:
                raise ContractError("valid slot cannot have a reason")
        elif self.status in ("missing", "invalid", "stale") and self.sample is None:
            identifier(self.reason, "reason")
        else:
            raise ContractError(
                "missing, invalid and stale slots cannot contain a sample"
            )


@dataclass(frozen=True, slots=True)
class Observation:
    """Robot-visible data only, with no privileged physical state."""

    stamp: SnapshotMetadata
    sensors: tuple[SensorSlot, ...] = ()

    def __post_init__(self) -> None:
        require_type(self.stamp, SnapshotMetadata, "stamp")
        require_type(self.sensors, tuple, "sensors")
        for slot in self.sensors:
            require_type(slot, SensorSlot, "slot")
            if (
                slot.sample is not None
                and slot.sample.available_time_ns > self.stamp.timestamp.time_ns
            ):
                raise ContractError("observation cannot contain a future sensor sample")
        ordered_ids(tuple(slot.sensor_id for slot in self.sensors), "sensors")
