"""Data models and type definitions for the evaluation framework."""
from __future__ import annotations

import math
import statistics
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Sequence, Tuple


class FailureType(str, Enum):
    """Failure classification taxonomy based on NavigationAnalyzer / Arena benchmarks."""
    SUCCESS = "SUCCESS"
    DEADLOCK_NARROW = "DEADLOCK_NARROW"
    LOCAL_MINIMA_OSCILLATION = "LOCAL_MINIMA_OSCILLATION"
    COLLISION_OBSTACLE = "COLLISION_OBSTACLE"
    TIMEOUT = "TIMEOUT"
    GOAL_NOT_REACHED = "GOAL_NOT_REACHED"
    UNAVAILABLE_NO_GOAL = "UNAVAILABLE_GOAL_NOT_CONFIGURED"


class SocialSpace(str, Enum):
    """Hall's Proxemic Zones (Hall, 1966)."""
    INTIMATE = "INTIMATE"       # < 0.45m
    PERSONAL = "PERSONAL"       # 0.45m - 1.20m
    SOCIAL = "SOCIAL"           # 1.20m - 3.60m
    PUBLIC = "PUBLIC"           # >= 3.60m


@dataclass
class Encounter:
    """Represents a close interaction episode between robot and obstacle."""
    id: int
    encounter_time_s: float
    encounter_end_s: float
    duration_s: float
    reaction_search_start_s: float
    reaction_search_end_s: float
    closest_approach_time_s: float
    minimum_center_distance_m: float
    minimum_estimated_clearance_m: float
    reaction_onset_s: Optional[float] = None
    reaction_classification: str = "NO_DETECTED_RESPONSE"
    reaction_label: str = "NO DETECTED RESPONSE"
    reaction_modes: List[str] = field(default_factory=list)
    lead_time_s: Optional[float] = None
    reaction_delay_s: Optional[float] = None
    distance_at_reaction_m: Optional[float] = None
    reaction_linear_x_mps: Optional[float] = None
    reaction_angular_z_radps: Optional[float] = None
    linear_x_before_reaction: Optional[Dict[str, Any]] = None
    angular_z_before_reaction: Optional[Dict[str, Any]] = None
    abs_angular_z_before_reaction: Optional[Dict[str, Any]] = None
    linear_x_after_reaction: Optional[Dict[str, Any]] = None
    angular_z_after_reaction: Optional[Dict[str, Any]] = None
    abs_angular_z_after_reaction: Optional[Dict[str, Any]] = None
    command_baseline_window_s: Optional[List[float]] = None
    baseline_linear_x_median_mps: Optional[float] = None
    baseline_linear_x_mad_mps: Optional[float] = None
    baseline_abs_angular_z_median_radps: Optional[float] = None
    baseline_abs_angular_z_mad_radps: Optional[float] = None
    stop_total_s_after_reaction_within_response_window: Optional[float] = None
    longest_continuous_stop_s_after_reaction_within_response_window: Optional[float] = None


@dataclass
class PSIMetrics:
    """Personal Space Intrusion (PSI) metrics from SEAN / Arena-Bench."""
    intimate_threshold_m: float = 0.45
    personal_threshold_m: float = 1.20
    total_intimate_time_s: float = 0.0
    intimate_time_ratio: float = 0.0
    total_personal_time_s: float = 0.0
    personal_time_ratio: float = 0.0
    minimum_distance_m: Optional[float] = None
    minimum_clearance_m: Optional[float] = None
    time_at_min_distance_s: Optional[float] = None
    intimate_intrusions_count: int = 0


@dataclass
class YieldingMetrics:
    """Proactive yielding and alcove evasion metrics."""
    yielding_detected: bool = False
    entered_alcove: bool = False
    alcove_entry_time_s: Optional[float] = None
    alcove_exit_time_s: Optional[float] = None
    alcove_duration_s: float = 0.0
    yielding_initiation_time_s: Optional[float] = None
    distance_to_obstacle_at_yielding_m: Optional[float] = None
    yielding_lead_time_s: Optional[float] = None
    max_alcove_depth_reached_m: float = 0.0


@dataclass
class MotionMetrics:
    """Trajectory smoothness, efficiency, and jerk metrics."""
    total_path_length_m: float = 0.0
    path_efficiency_ratio: Optional[float] = None
    linear_jerk_rms_mps3: Optional[float] = None
    angular_jerk_rms_radps3: Optional[float] = None
    normalized_jerk_cost: Optional[float] = None
    linear_acceleration_stats: Dict[str, Optional[float]] = field(default_factory=dict)
    angular_acceleration_stats: Dict[str, Optional[float]] = field(default_factory=dict)


@dataclass
class TrialMetrics:
    """Complete aggregated metrics for a single trial."""
    failure_type: FailureType
    failure_reason: str
    goal_reached: bool
    travel_time_s: float
    path_length_m: float
    collision_contact: bool
    psi: PSIMetrics
    yielding: YieldingMetrics
    motion: MotionMetrics
    encounters: List[Encounter] = field(default_factory=list)


def finite(values: Sequence[Optional[float]]) -> List[float]:
    """Filter out None and non-finite values."""
    return [float(x) for x in values if x is not None and math.isfinite(float(x))]


def percentile(values: Sequence[float], p: float) -> Optional[float]:
    """Compute p-th percentile from a list of numbers."""
    cleaned = sorted(finite(values))
    if not cleaned:
        return None
    if len(cleaned) == 1:
        return cleaned[0]
    pos = (len(cleaned) - 1) * p / 100.0
    lo, hi = int(math.floor(pos)), int(math.ceil(pos))
    return cleaned[lo] + (cleaned[hi] - cleaned[lo]) * (pos - lo)


def compute_stats(values: Sequence[float]) -> Dict[str, Optional[float]]:
    """Compute common summary statistics (min, mean, median, p05, p50, p95, max)."""
    cleaned = finite(values)
    if not cleaned:
        return {
            "count": 0,
            "min": None,
            "mean": None,
            "median": None,
            "p05": None,
            "p50": None,
            "p95": None,
            "max": None,
        }
    return {
        "count": len(cleaned),
        "min": min(cleaned),
        "mean": statistics.fmean(cleaned),
        "median": statistics.median(cleaned),
        "p05": percentile(cleaned, 5),
        "p50": percentile(cleaned, 50),
        "p95": percentile(cleaned, 95),
        "max": max(cleaned),
    }


def format_num(val: Any, digits: int = 3) -> str:
    """Format floating point numbers or None consistently."""
    if val is None:
        return "n/a"
    if isinstance(val, float):
        return f"{val:.{digits}f}"
    return str(val)
