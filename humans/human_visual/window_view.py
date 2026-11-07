"""Main-thread GLFW window backed by MuJoCo's native renderer."""

from contextlib import contextmanager

import glfw
import mujoco


class PreviewWindow:
    def __init__(self, window, model, context, orbit_camera):
        self._window = window
        self._model = model
        self._context = context
        self._scene = mujoco.MjvScene(
            model, maxgeom=max(256, model.nskin + model.ngeom)
        )
        self._options = mujoco.MjvOption()
        self.paused = False
        if orbit_camera is not None:
            self._camera = orbit_camera
            self._home = (
                orbit_camera.lookat.copy(),
                orbit_camera.azimuth,
                orbit_camera.elevation,
                orbit_camera.distance,
            )
            self._cursor = glfw.get_cursor_pos(window)
            glfw.set_cursor_pos_callback(window, self._drag)
            glfw.set_scroll_callback(window, self._scroll)
            glfw.set_key_callback(window, self._key)

    def _drag(self, window, x, y):
        last_x, last_y = self._cursor
        self._cursor = (x, y)
        _, height = glfw.get_window_size(window)
        if height <= 0:
            return
        if glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS:
            action = mujoco.mjtMouse.mjMOUSE_ROTATE_V
        elif glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS:
            action = mujoco.mjtMouse.mjMOUSE_MOVE_V
        else:
            return
        mujoco.mjv_moveCamera(
            self._model,
            action,
            (x - last_x) / height,
            (y - last_y) / height,
            self._camera,
        )

    def _scroll(self, _window, _xoffset, yoffset):
        mujoco.mjv_moveCamera(
            self._model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0, 0.05 * yoffset, self._camera
        )

    def _key(self, _window, key, _scancode, action, _mods):
        if action != glfw.PRESS:
            return
        if key == glfw.KEY_SPACE:
            self.paused = not self.paused
        elif key == glfw.KEY_R:
            lookat, azimuth, elevation, distance = self._home
            self._camera.lookat[:] = lookat
            self._camera.azimuth = azimuth
            self._camera.elevation = elevation
            self._camera.distance = distance

    def is_running(self):
        glfw.poll_events()
        return (
            not glfw.window_should_close(self._window)
            and glfw.get_key(self._window, glfw.KEY_ESCAPE) != glfw.PRESS
        )

    def draw(self, model, data, camera):
        width, height = glfw.get_framebuffer_size(self._window)
        if width <= 0 or height <= 0:
            return
        mujoco.mjv_updateScene(
            model,
            data,
            self._options,
            None,
            camera,
            mujoco.mjtCatBit.mjCAT_ALL,
            self._scene,
        )
        mujoco.mjr_render(
            mujoco.MjrRect(0, 0, width, height), self._scene, self._context
        )
        glfw.swap_buffers(self._window)


@contextmanager
def open_window(model, width, height, *, orbit_camera=None):
    if not glfw.init():
        raise RuntimeError(
            "GLFW cannot open a desktop display; use --check-only or EGL recording"
        )
    window = context = None
    try:
        title = (
            "MuJoCo Corridor | Drag to orbit | Space: pause | R: reset view"
            if orbit_camera is not None
            else "Quaternius Human Animation - MuJoCo"
        )
        window = glfw.create_window(width, height, title, None, None)
        if not window:
            raise RuntimeError("GLFW could not create the MuJoCo preview window")
        glfw.make_context_current(window)
        glfw.swap_interval(1)
        context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_100)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_WINDOW, context)
        yield PreviewWindow(window, model, context, orbit_camera)
    finally:
        if context is not None:
            context.free()
        if window:
            glfw.destroy_window(window)
        glfw.terminate()
