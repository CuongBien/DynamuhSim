"""Shared checks for values crossing core interfaces."""

import math
import re
from collections.abc import Iterable
from typing import TypeVar

from .errors import ContractError

T = TypeVar("T")
_IDENTIFIER = re.compile(r"[a-z][a-z0-9_]*")
_RUN_ID = re.compile(r"[a-z0-9][a-z0-9_-]*")
_EPISODE_ID = re.compile(r"[a-z0-9][a-z0-9_-]*:(?:0|[1-9][0-9]*)")


def require_type(value: object, expected: type[T], field: str) -> T:
    if not isinstance(value, expected):
        raise ContractError(f"{field} must be {expected.__name__}")
    return value


def identifier(value: object, field: str) -> str:
    if not isinstance(value, str) or not _IDENTIFIER.fullmatch(value):
        raise ContractError(f"{field} must match {_IDENTIFIER.pattern}")
    return value


def run_id(value: object) -> None:
    if not isinstance(value, str) or not _RUN_ID.fullmatch(value):
        raise ContractError("run_id must be a lowercase slug")


def episode_id(value: object) -> None:
    if not isinstance(value, str) or not _EPISODE_ID.fullmatch(value):
        raise ContractError("episode_id must be <run_id>:<episode_ordinal>")


def finite(value: object, field: str) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise ContractError(f"{field} must be a finite number")
    try:
        result = float(value)
    except OverflowError:
        raise ContractError(f"{field} must be a finite number") from None
    if not math.isfinite(result):
        raise ContractError(f"{field} must be a finite number")
    return result


def positive(value: object, field: str) -> float:
    result = finite(value, field)
    if result <= 0:
        raise ContractError(f"{field} must be positive")
    return result


def integer(value: object, field: str, *, minimum: int = 0) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < minimum:
        raise ContractError(f"{field} must be an integer >= {minimum}")
    return value


def uint64(value: object, field: str = "episode_seed") -> int:
    result = integer(value, field)
    if result >= 2**64:
        raise ContractError(f"{field} must fit in uint64")
    return result


def frame_is(actual: str, expected: str, field: str) -> None:
    if actual != expected:
        raise ContractError(f"{field} must use frame {expected!r}")


def unique(values: Iterable[object], field: str) -> None:
    seen: set[object] = set()
    for value in values:
        if value in seen:
            raise ContractError(f"{field} contains duplicate {value!r}")
        seen.add(value)


def ordered_ids(ids: tuple[str, ...], field: str) -> None:
    unique(ids, field)
    if ids != tuple(sorted(ids)):
        raise ContractError(f"{field} must be sorted by ID")
