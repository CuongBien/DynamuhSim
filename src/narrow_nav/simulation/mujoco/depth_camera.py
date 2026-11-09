"""Shared MuJoCo depth renderer and online D435i capture."""

import mujoco
import numpy as np

from narrow_nav.core.observations import SensorObservation, SensorPayload
from narrow_nav.core.states import CleanDepthFrame

from ..backend import SimulationError
from ..d435i_noise import D435iNoiseModel
from ..depth_config import (
    DEPTH_FRAME_ID,
    DEPTH_SENSOR_ID,
    DepthCameraConfig,
)


def depth_to_metres(raw: np.ndarray, near_m: float, far_m: float) -> np.ndarray:
    """Convert a reversed-Z perspective depth buffer read with ZEROFAR to metres.

    Matches ``mujoco.Renderer``: raw 1 is the near plane and raw 0 the far plane.
    """
    znear, zfar = np.float32(near_m), np.float32(far_m)
    c_coef = -(zfar + znear) / (zfar - znear)
    d_coef = -(np.float32(2) * zfar * znear) / (zfar - znear)
    c_coef = np.float32(-0.5) * c_coef - np.float32(0.5)
    d_coef = np.float32(-0.5) * d_coef
    return (d_coef / (raw.astype(np.float64) + c_coef)).astype(np.float32)


class DepthCamera:
    """Offscreen RGB and metric depth capture from a fixed MJCF camera."""

    def __init__(self, model: mujoco.MjModel, camera_name: str) -> None:
        camera_id = model.camera(camera_name).id
        width, height = map(int, model.cam_resolution[camera_id])
        if width > model.vis.global_.offwidth or height > model.vis.global_.offheight:
            raise SimulationError(
                f"{camera_name} resolution {width}x{height} exceeds the offscreen "
                "buffer; raise <visual><global offwidth/offheight>"
            )
        self.camera = mujoco.MjvCamera()
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
        self.camera.fixedcamid = camera_id
        self.width, self.height = width, height
        self.scene = mujoco.MjvScene(model, maxgeom=max(10_000, 2 * model.ngeom))
        self._rect = mujoco.MjrRect(0, 0, width, height)
        self._rgb = np.empty((height, width, 3), dtype=np.uint8)
        self._raw = np.empty((height, width), dtype=np.float32)
        extent = model.stat.extent
        self.near_m = float(model.vis.map.znear * extent)
        self.far_m = float(model.vis.map.zfar * extent)

    def capture(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        option: mujoco.MjvOption,
        context: mujoco.MjrContext,
        *,
        rgb: bool = True,
    ) -> tuple[np.ndarray | None, np.ndarray]:
        """Return RGB and depth in metres, rows ordered bottom-up like OpenGL."""
        mujoco.mjv_updateScene(
            model,
            data,
            option,
            None,
            self.camera,
            mujoco.mjtCatBit.mjCAT_ALL,
            self.scene,
        )
        buffer, depth_map = context.currentBuffer, context.readDepthMap
        flags = self.scene.flags.copy()
        try:
            mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, context)
            context.readDepthMap = mujoco.mjtDepthMap.mjDEPTH_ZEROFAR
            if rgb:
                mujoco.mjr_render(self._rect, self.scene, context)
                mujoco.mjr_readPixels(self._rgb, None, self._rect, context)
            # Flat ID colours avoid lighting effects in the depth pass.
            self.scene.flags[mujoco.mjtRndFlag.mjRND_SEGMENT] = True
            self.scene.flags[mujoco.mjtRndFlag.mjRND_IDCOLOR] = True
            mujoco.mjr_render(self._rect, self.scene, context)
            mujoco.mjr_readPixels(None, self._raw, self._rect, context)
        finally:
            self.scene.flags[:] = flags
            context.readDepthMap = depth_map
            mujoco.mjr_setBuffer(buffer, context)
        depth = depth_to_metres(self._raw, self.near_m, self.far_m)
        depth[self._raw <= 0] = 0  # The reversed-Z clear value denotes no hit.
        return self._rgb.copy() if rgb else None, depth


def _payload(depth_m: np.ndarray, valid: np.ndarray) -> SensorPayload:
    return SensorPayload(
        depth_m.shape, tuple(depth_m.ravel().tolist()), tuple(valid.ravel().tolist())
    )


class D435iCapture:
    """Capture with a borrowed viewer context or an owned headless context."""

    def __init__(
        self,
        model: mujoco.MjModel,
        camera_name: str,
        profile: DepthCameraConfig,
        *,
        context: mujoco.MjrContext | None = None,
    ) -> None:
        # Keep render clipping below the configured sensor Min-Z on large scenes.
        near_m = min(0.02, profile.noise.min_depth_m / 2)
        model.vis.map.znear = min(model.vis.map.znear, near_m / model.stat.extent)
        self.camera = DepthCamera(model, camera_name)
        self.profile = profile
        self.noise = D435iNoiseModel(profile.calibration, profile.noise)
        self.option = mujoco.MjvOption()
        self.option.geomgroup[3] = (
            0  # Hide human collision proxies for the skinned actors.
        )
        self.option.geomgroup[5] = 1
        self._gl = None
        if context is not None:
            self._context = context
            return
        self._gl = mujoco.GLContext(self.camera.width, self.camera.height)
        try:
            self._gl.make_current()
            self._context = mujoco.MjrContext(
                model, mujoco.mjtFontScale.mjFONTSCALE_100
            )
        except BaseException:
            self._gl.free()
            raise

    def capture(
        self, model: mujoco.MjModel, data: mujoco.MjData, time_ns: int
    ) -> tuple[SensorObservation, CleanDepthFrame]:
        if self._gl is not None:
            self._gl.make_current()
        _, rendered = self.camera.capture(
            model, data, self.option, self._context, rgb=False
        )
        # The renderer reads bottom-up. SDK and dataset coordinates are top-down.
        rendered = rendered[::-1]
        clean_valid = np.isfinite(rendered) & (rendered > 0)
        clean = np.where(clean_valid, rendered, 0)
        measured, valid = self.noise.apply(clean)
        index = time_ns // self.profile.period_ns
        sample = SensorObservation(
            sensor_id=DEPTH_SENSOR_ID,
            source="d435i_noisy" if self.profile.noise.enabled else "d435i_ideal",
            frame_id=DEPTH_FRAME_ID,
            sample_index=index,
            capture_time_ns=time_ns,
            available_time_ns=time_ns,
            payload=_payload(measured, valid),
        )
        return sample, CleanDepthFrame(
            index, time_ns, DEPTH_FRAME_ID, _payload(clean, clean_valid)
        )

    def close(self) -> None:
        if self._gl is None:
            return
        self._gl.make_current()
        try:
            self._context.free()
        finally:
            self._gl.free()
