"""DynamuhSim Evaluation Framework Package.

Modular evaluation engine for multi-sensor perception and proactive yielding
in narrow, dynamic corridor environments.
"""

from evaluation.types import (
    FailureType,
    SocialSpace,
    Encounter,
    YieldingMetrics,
    PSIMetrics,
    MotionMetrics,
    TrialMetrics,
)
from evaluation.analyzer import CorridorTrialAnalyzer

__all__ = [
    "FailureType",
    "SocialSpace",
    "Encounter",
    "YieldingMetrics",
    "PSIMetrics",
    "MotionMetrics",
    "TrialMetrics",
    "CorridorTrialAnalyzer",
]

__version__ = "2.0.0"
