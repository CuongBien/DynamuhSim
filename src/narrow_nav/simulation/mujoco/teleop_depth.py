"""Drive the robot from the keyboard and watch its depth camera live.

The window shows the scene and the latest scheduled D435i measurement
with its clean reference, validity mask or error map. Commands use the
official ``SimulationLifecycle.step(RobotAction)`` path.

Select ``sensors=d435i`` for the live observation. Without a sensor profile,
the window shows the direct depth and RGB preview.

Example (from the repository root, in a desktop session)::

    .venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth \\
        sensors=d435i worlds=hospital_easy scenarios=hospital_easy

Keys: W/S or Up/Down drive, A/D strafe (mecanum), Q/E or Left/Right turn,
Space hold-to-stop, -/= speed, R reset, C camera mode, V diagnostics, P pause, H help, Esc quit.
Mouse on the scene: left drag rotate, right drag pan, scroll zoom.
"""

from argparse import ArgumentParser
from collections.abc import Iterable
from math import degrees
from pathlib import Path
from time import perf_counter

import glfw
import mujoco
import numpy as np
from omegaconf import OmegaConf

from narrow_nav.core.actions import RobotAction
from narrow_nav.core.errors import ContractError
from narrow_nav.core.observations import Observation
from narrow_nav.core.types import NS_PER_SECOND, Twist2D

from ..backend import SimulationError
from ..configuration import compose_config
from ..depth_config import DEPTH_SENSOR_ID
from ..depth_report import colorize_depth_error
from ..lifecycle import SimulationLifecycle
from ..world_builder import MJCF_ROOT
from .adapter import MuJoCoAdapter
from .depth_camera import DepthCamera

# A concrete environment when no override is given. User overrides of the same
# key replace these. A long budget keeps the session from truncating at 20 s.
DEFAULT_OVERRIDES = (
    "worlds=corridor_090",
    "scenarios=corridor_090",
    "robots=mecanum_robot",
    "experiments.duration_ns=600000000000",
)
CAMERA_NAME = "depth_camera"
CAMERA_MODES = ("chase", "track", "free")
DIAGNOSTIC_PANELS = ("Clean depth", "Validity mask", "Error (+/-100 mm)")

_FORWARD = (glfw.KEY_W, glfw.KEY_UP)
_BACKWARD = (glfw.KEY_S, glfw.KEY_DOWN)
_STRAFE_LEFT = (glfw.KEY_A,)
_STRAFE_RIGHT = (glfw.KEY_D,)
_TURN_LEFT = (glfw.KEY_Q, glfw.KEY_LEFT)
_TURN_RIGHT = (glfw.KEY_E, glfw.KEY_RIGHT)
_STOP_KEYS = (glfw.KEY_SPACE,)

# Near (warm) to far (cool) stops for the depth colour map.
_DEPTH_STOPS = np.linspace(0.0, 1.0, 5)
_DEPTH_COLOURS = np.array(
    [(255, 40, 30), (255, 200, 0), (60, 220, 90), (0, 150, 255), (40, 30, 160)],
    dtype=np.float64,
)

HELP = (
    "W/S  Up/Down\nA/D\nQ/E  Left/Right\nSpace\n- / =\nR\nC\nV\nP\nH\nEsc",
    (
        "forward / back\nstrafe left / right (mecanum)\nturn left / right\n"
        "hold to stop\nspeed down / up\nreset episode\ncamera chase/track/free\n"
        "clean / mask / error (D435i)\npause\nhide help\nquit"
    ),
)


def merge_overrides(defaults: Iterable[str], overrides: Iterable[str]) -> list[str]:
    """Return ``overrides`` plus every default whose key the user did not set."""
    overrides = list(overrides)
    given = {item.split("=", 1)[0].lstrip("+~") for item in overrides}
    return [
        *(item for item in defaults if item.split("=", 1)[0] not in given),
        *overrides,
    ]


def keyboard_twist(
    pressed: Iterable[int],
    linear_mps: float,
    angular_radps: float,
    *,
    holonomic: bool,
) -> Twist2D:
    """Map held keys to a requested robot-frame twist."""
    held = set(pressed)
    if held.intersection(_STOP_KEYS):
        return Twist2D(0.0, 0.0, 0.0)

    def axis(positive: tuple[int, ...], negative: tuple[int, ...]) -> float:
        return float(bool(held.intersection(positive))) - float(
            bool(held.intersection(negative))
        )

    turn = axis(_TURN_LEFT, _TURN_RIGHT)
    strafe = axis(_STRAFE_LEFT, _STRAFE_RIGHT)
    if not holonomic:
        # Differential drive rejects linear_y, so A/D turn instead.
        turn, strafe = max(-1.0, min(1.0, turn + strafe)), 0.0
    return Twist2D(
        axis(_FORWARD, _BACKWARD) * linear_mps,
        strafe * linear_mps,
        turn * angular_radps,
    )


def colorize_depth(depth_m: np.ndarray, min_m: float, max_m: float) -> np.ndarray:
    """Colour valid depths near-warm to far-cool; out-of-range pixels are black."""
    valid = (depth_m >= min_m) & (depth_m <= max_m)
    t = np.clip((depth_m - min_m) / (max_m - min_m), 0.0, 1.0)
    image = np.stack(
        [np.interp(t, _DEPTH_STOPS, _DEPTH_COLOURS[:, c]) for c in range(3)], axis=-1
    )
    image[~valid] = 0
    return image.astype(np.uint8)


def _resize_rgb_buffer(image: np.ndarray, width: int, height: int) -> np.ndarray:
    """Resize by nearest neighbour and pack a flat RGB buffer for mjr_drawPixels."""
    rows = np.arange(height) * image.shape[0] // height
    cols = np.arange(width) * image.shape[1] // width
    return image[rows[:, None], cols[None, :]].ravel()


class _ViewerAdapter(MuJoCoAdapter):
    """Official adapter that also lends its compiled model to this viewer.

    The native handles stay inside ``simulation/mujoco`` and are only used to
    render frames in this process; they never cross a ``core`` interface.
    """

    def native_for_rendering(self) -> tuple[mujoco.MjModel, mujoco.MjData]:
        return self._native()

    def use_viewer_context(self, context: mujoco.MjrContext) -> None:
        self._depth_context = context


class TeleopDepthViewer:
    """GLFW window, keyboard/mouse input and the real-time control loop."""

    def __init__(
        self,
        engine: SimulationLifecycle,
        adapter: _ViewerAdapter,
        *,
        title: str,
        camera_name: str,
        linear_mps: float,
        angular_radps: float,
        holonomic: bool,
        depth_range_m: tuple[float, float],
        speed_scale: float,
        top_down: bool = False,
    ) -> None:
        self.engine, self.adapter = engine, adapter
        self.model, self.data = adapter.native_for_rendering()
        self.linear_mps, self.angular_radps = linear_mps, angular_radps
        self.holonomic = holonomic
        self.depth_min_m, self.depth_max_m = depth_range_m
        self.speed_scale = speed_scale
        self.top_down = top_down
        self.pressed: set[int] = set()
        self.paused = self.show_help = False
        self.camera_mode = 0
        self.azimuth_offset = 0.0
        self.lookat_offset = np.zeros(3)
        self.command = Twist2D(0.0, 0.0, 0.0)
        self.status = "running"
        self.contacts = 0
        self.real_time_factor = 0.0
        self._buttons: set[int] = set()
        self.diagnostic_panel = 0
        self._sensor_views: tuple[np.ndarray, ...] = ()
        self._sensor_labels = ("", "")

        model = self.model
        # Close-up chase views need a near plane in centimetres on large worlds.
        model.vis.map.znear = min(model.vis.map.znear, 0.02 / model.stat.extent)
        model.vis.headlight.active = 1
        model.vis.headlight.ambient[:] = (0.65, 0.65, 0.65)
        model.vis.headlight.diffuse[:] = (0.55, 0.55, 0.55)

        if not glfw.init():
            raise SimulationError("GLFW could not start; run in a desktop session")
        glfw.window_hint(glfw.SAMPLES, 4)
        self.window = glfw.create_window(1440, 780, title, None, None)
        if not self.window:
            glfw.terminate()
            raise SimulationError("GLFW could not open a window")
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)

        self.option = mujoco.MjvOption()
        self.option.geomgroup[3] = 0  # Human collision footprints.
        self.option.geomgroup[5] = 1  # Actor visual meshes and bodies.
        self.scene = mujoco.MjvScene(model, maxgeom=max(10_000, 2 * model.ngeom))
        self.context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
        adapter.use_viewer_context(self.context)
        self.depth_camera = DepthCamera(model, camera_name)
        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(model, self.camera)
        # A steeper view keeps the chase camera inside the default corridor at spawn.
        self.camera.distance, self.camera.elevation = 2.4, -45.0

        # Use motion events to preserve the start position of queued drags.
        self._cursor = glfw.get_cursor_pos(self.window)
        glfw.set_key_callback(self.window, self._on_key)
        glfw.set_window_focus_callback(self.window, self._on_focus)
        glfw.set_mouse_button_callback(self.window, self._on_button)
        glfw.set_cursor_pos_callback(self.window, self._on_cursor)
        glfw.set_scroll_callback(self.window, self._on_scroll)

    # Input -----------------------------------------------------------------
    def _on_key(self, window, key: int, scancode: int, action: int, mods: int) -> None:
        if action == glfw.RELEASE:
            self.pressed.discard(key)
            return
        if action != glfw.PRESS:
            return
        self.pressed.add(key)
        if key == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(window, True)
        elif key == glfw.KEY_R:
            self._reset()
        elif key == glfw.KEY_P:
            self.paused = not self.paused
        elif key == glfw.KEY_H:
            self.show_help = not self.show_help
        elif key == glfw.KEY_C:
            self.camera_mode = (self.camera_mode + 1) % len(CAMERA_MODES)
        elif key == glfw.KEY_V:
            self.diagnostic_panel = (self.diagnostic_panel + 1) % len(DIAGNOSTIC_PANELS)
        elif key in (glfw.KEY_MINUS, glfw.KEY_KP_SUBTRACT):
            self.speed_scale = max(0.1, round(self.speed_scale - 0.1, 2))
        elif key in (glfw.KEY_EQUAL, glfw.KEY_KP_ADD):
            self.speed_scale = min(1.0, round(self.speed_scale + 0.1, 2))

    def _on_focus(self, window, focused: int) -> None:
        if not focused:
            self.pressed.clear()  # Avoid a stuck drive key after alt-tab.
            self._buttons.clear()

    def _cursor_in_scene(self, window, x: float, y: float) -> bool:
        window_width, window_height = glfw.get_window_size(window)
        width, height = glfw.get_framebuffer_size(window)
        if min(window_width, window_height, width, height) <= 0:
            return False
        panel_width, _ = self._panel_size(width, height)
        scene_right = (width - panel_width) * window_width / width
        return 0 <= x < scene_right and 0 <= y < window_height

    def _on_button(self, window, button: int, action: int, mods: int) -> None:
        if action == glfw.RELEASE:
            self._buttons.discard(button)
        elif (
            action == glfw.PRESS
            and button
            in (
                glfw.MOUSE_BUTTON_LEFT,
                glfw.MOUSE_BUTTON_RIGHT,
                glfw.MOUSE_BUTTON_MIDDLE,
            )
            and self._cursor_in_scene(window, *self._cursor)
        ):
            self._buttons.add(button)

    def _on_cursor(self, window, x: float, y: float) -> None:
        dx, dy = x - self._cursor[0], y - self._cursor[1]
        self._cursor = (x, y)
        if not self._buttons:
            return
        shift = (
            glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
            or glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
        )
        if glfw.MOUSE_BUTTON_RIGHT in self._buttons:
            action = (
                mujoco.mjtMouse.mjMOUSE_MOVE_H
                if shift
                else mujoco.mjtMouse.mjMOUSE_MOVE_V
            )
        elif glfw.MOUSE_BUTTON_LEFT in self._buttons:
            action = (
                mujoco.mjtMouse.mjMOUSE_ROTATE_H
                if shift
                else mujoco.mjtMouse.mjMOUSE_ROTATE_V
            )
        else:
            action = mujoco.mjtMouse.mjMOUSE_ZOOM
        height = max(1, glfw.get_window_size(window)[1])
        lookat_before = self.camera.lookat.copy()
        azimuth_before = self.camera.azimuth
        mujoco.mjv_moveCamera(self.model, action, dx / height, dy / height, self.camera)
        mode = CAMERA_MODES[self.camera_mode]
        if mode != "free":
            self.lookat_offset += self.camera.lookat - lookat_before
        if mode == "chase":
            self.azimuth_offset += self.camera.azimuth - azimuth_before

    def _on_scroll(self, window, x_offset: float, y_offset: float) -> None:
        if not self._cursor_in_scene(window, *self._cursor):
            return
        mujoco.mjv_moveCamera(
            self.model,
            mujoco.mjtMouse.mjMOUSE_ZOOM,
            0.0,
            0.05 * y_offset,
            self.camera,
        )

    # Simulation ------------------------------------------------------------
    def _reset(self) -> None:
        self._update_sensor_preview(self.engine.reset())
        self.status, self.contacts = "running", 0
        self.command = Twist2D(0.0, 0.0, 0.0)

    def _step(self) -> None:
        truth = self.engine.get_ground_truth()
        self.command = keyboard_twist(
            self.pressed,
            self.speed_scale * self.linear_mps,
            self.speed_scale * self.angular_radps,
            holonomic=self.holonomic,
        )
        try:
            result = self.engine.step(
                RobotAction(truth.robot.entity_id, self.command, truth.stamp.timestamp)
            )
        except (ContractError, SimulationError) as exc:
            # Keep the window alive; R starts a new episode.
            self.status = f"failed: {type(exc).__name__}"
            return
        self.contacts += len(result.contacts)
        self._update_sensor_preview(result.observation)
        if result.terminated or result.truncated:
            self.status = f"{result.end_reason} (R to reset)"

    def _follow_robot(self) -> None:
        mode = CAMERA_MODES[self.camera_mode]
        if mode == "free":
            return
        pose = self.engine.get_ground_truth().robot.pose
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.camera.lookat[:] = self.lookat_offset + (pose.x, pose.y, 0.30)
        if mode == "chase":
            self.camera.azimuth = degrees(pose.yaw) + self.azimuth_offset

    # Rendering -------------------------------------------------------------
    def _nearest_at_centre(self, depth: np.ndarray) -> str:
        h, w = depth.shape
        centre = depth[h * 2 // 5 : h * 3 // 5, w * 2 // 5 : w * 3 // 5]
        centre = centre[(centre >= self.depth_min_m) & (centre <= self.depth_max_m)]
        return f"{centre.min():.2f} m" if centre.size else "out of range"

    def _update_sensor_preview(self, observation: Observation) -> None:
        # Pair frames at their publication boundary. A later step can reuse
        # the observation without publishing any new clean frames.
        frames = self.engine.get_clean_depth()
        if not frames:
            return
        sample = next(
            slot.sample
            for slot in observation.sensors
            if slot.sensor_id == DEPTH_SENSOR_ID
        )
        clean = frames[-1].payload
        measured = sample.payload
        depth = np.asarray(measured.values, dtype=np.float32).reshape(measured.shape)
        valid = np.asarray(measured.valid_mask, dtype=np.bool_).reshape(measured.shape)
        reference = np.asarray(clean.values, dtype=np.float32).reshape(clean.shape)
        reference_valid = np.asarray(clean.valid_mask, dtype=np.bool_).reshape(
            clean.shape
        )
        depth = np.where(valid, depth, 0)
        reference = np.where(reference_valid, reference, 0)
        self._sensor_views = (
            colorize_depth(depth, self.depth_min_m, self.depth_max_m)[::-1],
            colorize_depth(reference, self.depth_min_m, self.depth_max_m)[::-1],
            np.repeat(valid[::-1, :, None], 3, axis=-1).astype(np.uint8) * 255,
            colorize_depth_error(depth - reference, valid & reference_valid, 0.1)[::-1],
        )
        mode = "noisy" if sample.source == "d435i_noisy" else "ideal"
        capture_s = sample.capture_time_ns / NS_PER_SECOND
        summary = f"{valid.mean():.1%} / {self._nearest_at_centre(depth)}"
        self._sensor_labels = (
            f"D435i {mode}\nFill / centre",
            f"#{sample.sample_index} @ {capture_s:.3f} s\n{summary}",
        )

    def _hud(self) -> tuple[str, str]:
        truth = self.engine.get_ground_truth()
        robot, metadata = truth.robot, self.engine.get_metadata()
        clearance = self.engine.get_clearance()
        human = (
            "-"
            if clearance.robot_human_m is None
            else f"{clearance.robot_human_m:.2f} m"
        )
        status = "paused" if self.paused and self.status == "running" else self.status
        c = self.command
        labels = (
            "Scenario\nStatus\nTime\nCommand\nRobot twist\nPose\nClearance\n"
            "Speed\nCamera\nSim / real"
        )
        values = (
            f"{metadata.episode.config.scenario_id}\n{status}\n"
            f"{robot.timestamp.time_s:.2f} s (tick {robot.timestamp.tick})\n"
            f"vx {c.linear_x:+.2f}  vy {c.linear_y:+.2f}  wz {c.angular_z:+.2f}\n"
            f"vx {robot.velocity.linear_x:+.2f}  vy {robot.velocity.linear_y:+.2f}"
            f"  wz {robot.velocity.angular_z:+.2f}\n"
            f"x {robot.pose.x:+.2f}  y {robot.pose.y:+.2f}  "
            f"yaw {degrees(robot.pose.yaw):+.0f} deg\n"
            f"wall {clearance.robot_wall_m:.2f} m  human {human}\n"
            f"{self.speed_scale:.0%}  (contacts {self.contacts})\n"
            f"{CAMERA_MODES[self.camera_mode]}  (H help)\n"
            f"{self.real_time_factor:.2f}x"
        )
        return labels, values

    def _panel_size(self, width: int, height: int) -> tuple[int, int]:
        cam = self.depth_camera
        panel_w = max(1, int(0.36 * width))
        panel_h = max(1, panel_w * cam.height // cam.width)
        if 2 * panel_h > height:
            panel_h = max(1, height // 2)
            panel_w = max(1, panel_h * cam.width // cam.height)
        return panel_w, panel_h

    def _render(self) -> None:
        width, height = glfw.get_framebuffer_size(self.window)
        if width <= 0 or height <= 0:
            return
        cam = self.depth_camera
        panel_w, panel_h = self._panel_size(width, height)
        main = mujoco.MjrRect(0, 0, width - panel_w, height)
        depth_rect = mujoco.MjrRect(width - panel_w, height - panel_h, panel_w, panel_h)
        lower_rect = mujoco.MjrRect(
            width - panel_w, height - 2 * panel_h, panel_w, panel_h
        )

        if self._sensor_views:
            top = self._sensor_views[0]
            lower = self._sensor_views[1 + self.diagnostic_panel]
            top_labels = self._sensor_labels
            lower_labels = (DIAGNOSTIC_PANELS[self.diagnostic_panel], "V to switch")
        else:
            lower, depth = cam.capture(self.model, self.data, self.option, self.context)
            top = colorize_depth(depth, self.depth_min_m, self.depth_max_m)
            top_labels = (
                f"Depth {self.depth_min_m:.2f}-{self.depth_max_m:.2f} m",
                f"centre {self._nearest_at_centre(depth)}",
            )
            lower_labels = ("RGB", CAMERA_NAME)
        mujoco.mjr_rectangle(
            mujoco.MjrRect(width - panel_w, 0, panel_w, height), 0.08, 0.08, 0.1, 1
        )
        self._follow_robot()
        mujoco.mjv_updateScene(
            self.model,
            self.data,
            self.option,
            None,
            self.camera,
            mujoco.mjtCatBit.mjCAT_ALL,
            self.scene,
        )
        mujoco.mjr_render(main, self.scene, self.context)

        mujoco.mjr_drawPixels(
            _resize_rgb_buffer(top, panel_w, panel_h),
            None,
            depth_rect,
            self.context,
        )
        mujoco.mjr_drawPixels(
            _resize_rgb_buffer(lower, panel_w, panel_h), None, lower_rect, self.context
        )
        font = mujoco.mjtFontScale.mjFONTSCALE_150
        grid = mujoco.mjtGridPos
        mujoco.mjr_overlay(
            font,
            grid.mjGRID_TOPLEFT,
            depth_rect,
            *top_labels,
            self.context,
        )
        mujoco.mjr_overlay(
            font, grid.mjGRID_TOPLEFT, lower_rect, *lower_labels, self.context
        )
        mujoco.mjr_overlay(
            font, grid.mjGRID_BOTTOMLEFT, main, *self._hud(), self.context
        )
        if self.show_help:
            mujoco.mjr_overlay(font, grid.mjGRID_TOPLEFT, main, *HELP, self.context)
        glfw.swap_buffers(self.window)

    def run(self) -> None:
        self._reset()
        self._follow_robot()
        if self.top_down:
            self.camera_mode = CAMERA_MODES.index("free")
            self.camera.azimuth, self.camera.elevation = 0.0, -90.0
            self.camera.distance = 1.8
        control_dt_s = (
            self.engine.get_metadata().timing.robot_control_dt_ns / NS_PER_SECOND
        )
        last = perf_counter()
        budget = sim_elapsed = wall_elapsed = 0.0
        while not glfw.window_should_close(self.window):
            glfw.poll_events()
            now = perf_counter()
            frame_dt, last = min(now - last, 0.25), now
            if self.paused or self.status != "running":
                budget = 0.0
            else:
                budget += frame_dt
                steps = 0
                while budget >= control_dt_s and steps < 4:
                    self._step()
                    budget -= control_dt_s
                    steps += 1
                    if self.status != "running":
                        break
                budget = min(budget, control_dt_s)  # Drop lag, stay live.
                sim_elapsed += steps * control_dt_s
                wall_elapsed += frame_dt
                if wall_elapsed >= 1.0:
                    self.real_time_factor = sim_elapsed / wall_elapsed
                    sim_elapsed = wall_elapsed = 0.0
            self._render()

    def close(self) -> None:
        self.context.free()
        glfw.destroy_window(self.window)
        glfw.terminate()


def main() -> None:
    parser = ArgumentParser(
        description="Keyboard teleoperation with a live depth camera view."
    )
    parser.add_argument(
        "--config-dir", type=Path, default=MJCF_ROOT.parents[1] / "configs"
    )
    parser.add_argument("--speed", type=float, default=0.5, help="start speed 0.1-1")
    parser.add_argument(
        "--top-down", action="store_true", help="start with a stationary overhead view"
    )
    parser.add_argument("--depth-min", type=float, help="display minimum in metres")
    parser.add_argument("--depth-max", type=float, help="display maximum in metres")
    parser.add_argument(
        "overrides",
        nargs="*",
        help="Hydra overrides; defaults: " + " ".join(DEFAULT_OVERRIDES),
    )
    args = parser.parse_args()
    if not 0.1 <= args.speed <= 1.0:
        parser.error("--speed must be between 0.1 and 1")
    overrides = merge_overrides(DEFAULT_OVERRIDES, args.overrides)
    config = compose_config(args.config_dir, overrides)
    depth_min = (
        args.depth_min
        if args.depth_min is not None
        else OmegaConf.select(config, "sensors.noise.min_depth_m", default=0.2)
    )
    depth_max = (
        args.depth_max
        if args.depth_max is not None
        else OmegaConf.select(config, "sensors.noise.max_depth_m", default=5.0)
    )
    if not 0 < depth_min < depth_max:
        parser.error("require 0 < --depth-min < --depth-max")
    adapter = _ViewerAdapter()
    engine = SimulationLifecycle(adapter)
    viewer = None
    try:
        engine.load(config)
        model, _ = adapter.native_for_rendering()
        camera_name = f"{config.scenarios.robot.id}/{CAMERA_NAME}"
        try:
            model.camera(camera_name)
        except KeyError:
            parser.error(
                f"robot profile {config.robots.id!r} has no {CAMERA_NAME}; "
                "use robots=mecanum_robot"
            )
        viewer = TeleopDepthViewer(
            engine,
            adapter,
            title="NarrowNav teleop - " + " ".join(overrides),
            camera_name=camera_name,
            linear_mps=float(OmegaConf.select(config, "robots.max_linear_speed_mps")),
            angular_radps=float(
                OmegaConf.select(config, "robots.max_angular_speed_radps")
            ),
            holonomic=config.robots.drive == "mecanum",
            depth_range_m=(depth_min, depth_max),
            speed_scale=args.speed,
            top_down=args.top_down,
        )
        viewer.run()
    finally:
        try:
            engine.close()
        finally:
            if viewer is not None:
                viewer.close()


if __name__ == "__main__":
    main()
