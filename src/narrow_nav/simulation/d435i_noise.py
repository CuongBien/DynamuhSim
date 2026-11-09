"""Parametric rectified stereo depth noise for online capture."""

from math import sqrt

import numpy as np

from .depth_config import D435iNoiseConfig, DepthCalibration


class D435iNoiseModel:
    """Map axial metric depth to a sensor measurement, with zero for invalid pixels."""

    def __init__(self, calibration: DepthCalibration, config: D435iNoiseConfig) -> None:
        self.calibration = calibration
        self.config = config
        self._focal_baseline = calibration.fx_px * calibration.baseline_m
        self._shape = (calibration.height, calibration.width)
        self._columns = np.arange(calibration.width)
        self._rows = np.arange(calibration.height)[:, None]
        self._temporal: np.ndarray | None = None
        self._rng: np.random.Generator | None = None

    def reset(self, seed: int) -> None:
        self._rng = np.random.Generator(np.random.PCG64(seed))
        self._temporal = None

    def apply(self, clean_m: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if clean_m.shape != self._shape:
            raise ValueError(f"expected depth shape {self._shape}")
        cfg = self.config
        valid = (
            np.isfinite(clean_m)
            & (clean_m >= cfg.min_depth_m)
            & (clean_m <= cfg.max_depth_m)
        )
        if not cfg.enabled:
            return np.where(valid, clean_m, 0).astype(np.float32), valid
        rng = self._rng
        if rng is None:
            raise RuntimeError("reset depth noise before capture")

        disparity = np.divide(
            self._focal_baseline, clean_m, out=np.zeros(self._shape), where=valid
        )
        # Reproject the left image onto the rectified right scanlines. At a
        # collision, a point with larger disparity is closer to the right imager.
        right_x = np.rint(self._columns - disparity).astype(np.intp)
        matched = valid & (right_x >= 0) & (right_x < self.calibration.width)
        right_x = right_x.clip(0, self.calibration.width - 1)
        right_surface = np.zeros(self._shape)
        np.maximum.at(
            right_surface, (self._rows, right_x), np.where(matched, disparity, 0)
        )
        valid &= matched & (
            right_surface[self._rows, right_x] - disparity <= cfg.edge_threshold_px
        )

        # SRU-style local disparity discontinuities and pseudo-matches.
        # A synthetic mismatch copies a horizontal neighbour from the other side.
        padded = np.pad(disparity, ((0, 0), (1, 1)), mode="edge")
        left, right = padded[:, :-2], padded[:, 2:]
        left_jump = np.where(left > 0, np.abs(left - disparity), 0)
        right_jump = np.where(right > 0, np.abs(right - disparity), 0)
        edge = np.maximum(left_jump, right_jump) > cfg.edge_threshold_px
        draw = rng.random(self._shape)
        valid &= ~(edge & (draw < cfg.edge_dropout_probability))
        mismatch = edge & (
            draw < cfg.edge_dropout_probability + cfg.edge_mismatch_probability
        )
        disparity = np.where(
            mismatch, np.where(left_jump >= right_jump, left, right), disparity
        )

        tile = cfg.hole_tile_px
        holes = rng.random(tuple((size + tile - 1) // tile for size in self._shape))
        valid &= (
            holes[self._rows // tile, self._columns // tile] >= cfg.hole_probability
        )

        disparity += rng.normal(0, cfg.disparity_sigma_px, self._shape)
        if cfg.temporal_sigma_px:
            innovation = rng.normal(0, cfg.temporal_sigma_px, self._shape)
            rho = cfg.temporal_correlation
            self._temporal = (
                innovation
                if self._temporal is None
                else rho * self._temporal + sqrt(1 - rho**2) * innovation
            )
            disparity += self._temporal

        disparity = np.rint(disparity / cfg.disparity_step_px) * cfg.disparity_step_px
        valid &= disparity > 0
        depth = np.divide(
            self._focal_baseline, disparity, out=np.zeros(self._shape), where=valid
        )
        depth = np.rint(depth / cfg.depth_unit_m) * cfg.depth_unit_m
        valid &= (depth >= cfg.min_depth_m) & (depth <= cfg.max_depth_m)
        return np.where(valid, depth, 0).astype(np.float32), valid
