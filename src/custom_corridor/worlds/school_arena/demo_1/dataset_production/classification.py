"""Separate research outcomes from failures of recording or runtime infrastructure."""
from __future__ import annotations

from dataclasses import dataclass

from dataset_runner.episode_types import EpisodeResult, EpisodeStatus

VALID_OUTCOMES = {
    EpisodeStatus.SUCCESS,
    EpisodeStatus.COLLISION,
    EpisodeStatus.TIMEOUT,
    EpisodeStatus.STUCK,
    EpisodeStatus.NAV_FAILURE,
}


@dataclass(frozen=True)
class ProductionDecision:
    status: str
    outcome: str
    reason: str


def classify(result: EpisodeResult, *, recording_ok: bool,
             structured_ok: bool, qa_pass: bool, reason: str = "") -> ProductionDecision:
    if result.status not in VALID_OUTCOMES:
        return ProductionDecision("FAIL_INFRASTRUCTURE", "", result.termination_reason)
    if not recording_ok:
        return ProductionDecision("FAIL_INFRASTRUCTURE", "", reason or "raw bag missing/corrupt")
    if not structured_ok:
        return ProductionDecision("FAIL_INFRASTRUCTURE", "", reason or "structured data missing/corrupt")
    if not qa_pass:
        return ProductionDecision("FAIL_INFRASTRUCTURE", "", reason or "QA failed")
    return ProductionDecision("PASS", result.status.value, result.termination_reason)
