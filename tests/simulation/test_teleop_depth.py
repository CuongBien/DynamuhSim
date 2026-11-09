from types import SimpleNamespace

import glfw
import mujoco
import numpy as np
import pytest

from narrow_nav.core.observations import SensorObservation, SensorPayload, SensorSlot
from narrow_nav.core.states import CleanDepthFrame
from narrow_nav.simulation.mujoco.depth_camera import depth_to_metres
from narrow_nav.simulation.mujoco.teleop_depth import (
    CAMERA_MODES,
    TeleopDepthViewer,
    _resize_rgb_buffer,
    colorize_depth,
    keyboard_twist,
    merge_overrides,
)


def test_user_overrides_replace_defaults_by_key():
    merged = merge_overrides(
        ("worlds=corridor_090", "robots=mecanum_robot"), ("worlds=hospital_easy",)
    )
    assert merged == ["robots=mecanum_robot", "worlds=hospital_easy"]


def test_keyboard_twist_respects_drive_and_stop_key():
    keys = {glfw.KEY_W, glfw.KEY_A, glfw.KEY_E}
    mecanum = keyboard_twist(keys, 0.3, 0.9, holonomic=True)
    assert (mecanum.linear_x, mecanum.linear_y, mecanum.angular_z) == (0.3, 0.3, -0.9)
    differential = keyboard_twist({glfw.KEY_W, glfw.KEY_A}, 0.3, 0.9, holonomic=False)
    assert (differential.linear_y, differential.angular_z) == (0.0, 0.9)
    stopped = keyboard_twist(keys | {glfw.KEY_SPACE}, 0.3, 0.9, holonomic=True)
    assert (stopped.linear_x, stopped.linear_y, stopped.angular_z) == (0, 0, 0)


def test_depth_buffer_maps_reversed_z_planes_to_metres():
    near, far = 0.02, 1500.0
    # Reversed-Z buffer value for a point at z metres: n * (f - z) / (z * (f - n)).
    depths = np.array([near, 0.5, 2.0, 4.0])
    raw = (near * (far - depths) / (depths * (far - near))).astype(np.float32)
    assert depth_to_metres(raw, near, far) == pytest.approx(depths, rel=1e-4)


def test_colorize_blacks_out_of_range_pixels():
    image = colorize_depth(np.array([[0.1, 1.0, 9.0]], dtype=np.float32), 0.2, 5.0)
    assert image.dtype == np.uint8
    assert image[0, 0].sum() == 0 and image[0, 2].sum() == 0
    assert image[0, 1].sum() > 0


def test_resize_rgb_buffer_packs_pixels_for_mujoco():
    image = np.arange(12, dtype=np.uint8).reshape(2, 2, 3)
    buffer = _resize_rgb_buffer(image, width=4, height=2)

    assert buffer.shape == (24,)
    assert buffer.dtype == np.uint8
    assert buffer.flags.c_contiguous
    np.testing.assert_array_equal(buffer.reshape(2, 4, 3), np.repeat(image, 2, axis=1))


def test_d435i_preview_uses_observation_and_retains_its_clean_pair():
    viewer = TeleopDepthViewer.__new__(TeleopDepthViewer)
    viewer.depth_min_m, viewer.depth_max_m = 0.1, 6.0
    measured = SensorPayload((2, 2), (1.0, 0.0, 2.0, 3.0), (True, False, True, True))
    clean = SensorPayload((2, 2), (1.5, 1.0, 2.0, 2.5), (True,) * 4)
    sample = SensorObservation(
        "depth",
        "d435i_noisy",
        "depth_optical",
        2,
        40_000_000,
        40_000_000,
        payload=measured,
    )
    frames = [CleanDepthFrame(2, 40_000_000, "depth_optical", clean)]
    viewer.engine = SimpleNamespace(get_clean_depth=lambda: tuple(frames))
    observation = SimpleNamespace(sensors=(SensorSlot("depth", "valid", sample),))

    viewer._update_sensor_preview(observation)
    preview = viewer._sensor_views
    expected = colorize_depth(np.asarray(measured.values).reshape(2, 2), 0.1, 6.0)
    np.testing.assert_array_equal(preview[0][::-1], expected)
    assert not np.array_equal(preview[0], preview[1])
    assert preview[2][-1, 1].sum() == 0
    assert "#2 @ 0.040 s" in viewer._sensor_labels[1]
    assert "75.0%" in viewer._sensor_labels[1]

    frames.clear()
    viewer._update_sensor_preview(observation)
    assert viewer._sensor_views is preview


@pytest.fixture
def camera_viewer(monkeypatch):
    viewer = TeleopDepthViewer.__new__(TeleopDepthViewer)
    viewer.model = mujoco.MjModel.from_xml_string("<mujoco/>")
    viewer.camera = mujoco.MjvCamera()
    viewer.camera.distance = 2.4
    viewer.camera.elevation = -45.0
    viewer.camera_mode = CAMERA_MODES.index("chase")
    viewer.azimuth_offset = 0.0
    viewer.lookat_offset = np.zeros(3)
    viewer.depth_camera = SimpleNamespace(width=424, height=240)
    viewer.pressed = set()
    viewer._buttons = set()
    viewer._cursor = (100.0, 100.0)
    pose = SimpleNamespace(x=1.0, y=2.0, yaw=0.0)
    truth = SimpleNamespace(robot=SimpleNamespace(pose=pose))
    viewer.engine = SimpleNamespace(get_ground_truth=lambda: truth)
    monkeypatch.setattr(glfw, "get_window_size", lambda _: (1440, 780))
    monkeypatch.setattr(glfw, "get_framebuffer_size", lambda _: (2880, 1560))
    monkeypatch.setattr(glfw, "get_key", lambda *_: glfw.RELEASE)
    viewer._follow_robot()
    return viewer, pose


def test_camera_drag_and_scroll_use_native_binding(camera_viewer):
    viewer, _ = camera_viewer
    viewer._on_button(None, glfw.MOUSE_BUTTON_LEFT, glfw.PRESS, 0)
    viewer._on_cursor(None, 160, 140)
    azimuth = viewer.camera.azimuth
    assert azimuth != 0.0
    viewer._follow_robot()
    assert viewer.camera.azimuth == pytest.approx(azimuth)

    viewer._on_scroll(None, 0, 1)
    zoomed_in = viewer.camera.distance
    assert zoomed_in < 2.4
    viewer._on_scroll(None, 0, -1)
    zoomed_out = viewer.camera.distance
    assert zoomed_out > zoomed_in
    viewer._on_focus(None, 0)
    viewer._on_cursor(None, 250, 200)
    assert viewer.camera.azimuth == pytest.approx(azimuth)

    viewer._on_cursor(None, 1000, 100)
    viewer._on_scroll(None, 0, 1)
    viewer._on_button(None, glfw.MOUSE_BUTTON_LEFT, glfw.PRESS, 0)
    viewer._on_cursor(None, 850, 120)
    assert viewer.camera.distance == zoomed_out
    assert viewer.camera.azimuth == pytest.approx(azimuth)


@pytest.mark.parametrize("mode", ("chase", "track", "free"))
def test_camera_pan_survives_follow_updates(camera_viewer, mode):
    viewer, pose = camera_viewer
    viewer.camera_mode = CAMERA_MODES.index(mode)
    start = viewer.camera.lookat.copy()
    viewer._on_button(None, glfw.MOUSE_BUTTON_RIGHT, glfw.PRESS, 0)
    viewer._on_cursor(None, 160, 140)
    panned = viewer.camera.lookat.copy()
    assert not np.allclose(start, panned)

    pose.x += 1.0
    viewer._follow_robot()
    expected = panned if mode == "free" else panned + (1.0, 0.0, 0.0)
    np.testing.assert_allclose(viewer.camera.lookat, expected)
