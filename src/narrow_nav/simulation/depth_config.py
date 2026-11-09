"""Resolved camera geometry and model parameters, without renderer dependencies."""

from dataclasses import dataclass

from omegaconf import DictConfig, OmegaConf

from narrow_nav.core.errors import ConfigurationError
from narrow_nav.core.validation import finite, integer, positive, require_type

DEPTH_SENSOR_ID = "depth"
DEPTH_FRAME_ID = "depth_optical"
DEPTH_MODEL_VERSION = "d435i_disparity_v1"


@dataclass(frozen=True, slots=True)
class DepthCalibration:
    source: str
    width: int
    height: int
    fx_px: float
    fy_px: float
    cx_px: float
    cy_px: float
    baseline_m: float

    def __post_init__(self) -> None:
        if not require_type(self.source, str, "calibration.source").strip():
            raise ConfigurationError("calibration.source must identify the calibration")
        for name in ("width", "height"):
            integer(getattr(self, name), name, minimum=2)
        for name in ("fx_px", "fy_px", "baseline_m"):
            positive(getattr(self, name), name)
        if not (
            0 <= finite(self.cx_px, "cx_px") < self.width
            and 0 <= finite(self.cy_px, "cy_px") < self.height
        ):
            raise ConfigurationError("principal point must lie inside the depth image")


@dataclass(frozen=True, slots=True)
class D435iNoiseConfig:
    enabled: bool
    min_depth_m: float
    max_depth_m: float
    disparity_step_px: float
    depth_unit_m: float
    disparity_sigma_px: float
    edge_threshold_px: float
    edge_dropout_probability: float
    edge_mismatch_probability: float
    hole_tile_px: int
    hole_probability: float
    temporal_sigma_px: float
    temporal_correlation: float

    def __post_init__(self) -> None:
        require_type(self.enabled, bool, "noise.enabled")
        for name in (
            "min_depth_m",
            "max_depth_m",
            "disparity_step_px",
            "depth_unit_m",
            "edge_threshold_px",
        ):
            positive(getattr(self, name), name)
        if (
            not self.depth_unit_m
            <= self.min_depth_m
            < self.max_depth_m
            <= 65535 * self.depth_unit_m
        ):
            raise ConfigurationError("depth range must fit nonzero Z16 depth units")
        for name in ("disparity_sigma_px", "temporal_sigma_px"):
            if finite(getattr(self, name), name) < 0:
                raise ConfigurationError(f"{name} must be nonnegative")
        for name in (
            "edge_dropout_probability",
            "edge_mismatch_probability",
            "hole_probability",
        ):
            if not 0 <= finite(getattr(self, name), name) <= 1:
                raise ConfigurationError(f"{name} must be between 0 and 1")
        if self.edge_dropout_probability + self.edge_mismatch_probability > 1:
            raise ConfigurationError("edge probabilities must sum to at most 1")
        if not 0 <= finite(self.temporal_correlation, "temporal_correlation") < 1:
            raise ConfigurationError("temporal_correlation must be in [0, 1)")
        integer(self.hole_tile_px, "hole_tile_px", minimum=1)


@dataclass(frozen=True, slots=True)
class DepthCameraConfig:
    schema_version: int
    id: str
    calibration: DepthCalibration
    noise: D435iNoiseConfig
    period_ns: int

    def __post_init__(self) -> None:
        if (
            integer(self.schema_version, "sensors.schema_version") != 1
            or self.id != "d435i"
        ):
            raise ConfigurationError("expected D435i sensor profile schema version 1")
        integer(self.period_ns, "depth capture period", minimum=1)


def depth_from_config(config: DictConfig) -> DepthCameraConfig | None:
    node = config.get("sensors")
    if node is None:
        return None
    try:
        fields = OmegaConf.to_container(
            require_type(node, DictConfig, "sensors"),
            resolve=True,
            throw_on_missing=True,
        )
        camera = DepthCameraConfig(
            **{
                **fields,
                "calibration": DepthCalibration(**fields["calibration"]),
                "noise": D435iNoiseConfig(**fields["noise"]),
                "period_ns": config.simulation.get(
                    "observation_dt_ns", config.simulation.control_dt_ns
                ),
            }
        )
    except (KeyError, TypeError) as exc:
        raise ConfigurationError(f"invalid D435i profile: {exc}") from exc
    if config.robots.get("id") != "mecanum_robot":
        raise ConfigurationError("sensors=d435i requires robots=mecanum_robot")
    if DEPTH_SENSOR_ID not in config.simulation.get("observation_sensors", ()):
        raise ConfigurationError("sensors=d435i requires depth in observation_sensors")
    return camera
