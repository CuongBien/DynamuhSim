import json
from pathlib import Path

import mujoco
import numpy as np
import pytest
from human_visual.animation import HumanAnimator, forward_kinematics, slerp
from human_visual.bundle import AssetError, file_sha256, load_character
from human_visual.demo import run_demo
from human_visual.diagnostics import FootDiagnostics
from human_visual.mujoco_skin import build_preview, sole_probes
from human_visual.replay import CorridorReplay, Replay
from human_visual.window_view import PreviewWindow, glfw

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "assets/generated"
SPEC = json.loads((ROOT / "replay.json").read_text())


@pytest.fixture(scope="module")
def characters():
    try:
        return {
            gender: load_character(ASSETS / f"casual_{gender}")
            for gender in ("male", "female")
        }
    except (AssetError, FileNotFoundError) as error:
        if any(
            not (ASSETS / f"casual_{gender}" / "manifest.json").is_file()
            for gender in ("male", "female")
        ):
            pytest.skip(f"BLOCKED (assets): {error}")
        raise


def test_shortest_arc_slerp_and_antipodal_input():
    start = np.array([[1.0, 0.0, 0.0, 0.0]])
    end = np.array([[0.0, 0.0, 0.0, -1.0]])
    halfway = slerp(start, end, 0.5)[0]
    np.testing.assert_allclose(halfway, [2**-0.5, 0, 0, -(2**-0.5)])
    np.testing.assert_allclose(slerp(start, -start, 0.5), start)


def test_missing_assets_have_an_explicit_error():
    with pytest.raises(AssetError, match="missing character bundle"):
        load_character(ASSETS / "missing_bundle")


def test_verified_clothed_sources_and_weighted_bind_poses(characters):
    for gender, character in characters.items():
        manifest = character.metadata
        source = manifest["source"]
        assert manifest["gender"] == gender and source["license"] == "CC0-1.0"
        assert {"Shirt", "Pants", "Hair", "Eyes"} <= {
            part["name"] for part in manifest["parts"]
        }
        assert manifest["rig_version"] == "quaternius_animated_people_2019"
        source_file = ROOT / "assets/source" / source["file"]
        license_file = ROOT / "assets/source" / source["license_file"]
        if not (source_file.is_file() and license_file.is_file()):
            pytest.skip(
                "BLOCKED (assets): original FBX and license are needed to recheck provenance"
            )
        assert file_sha256(source_file) == source["sha256"]
        assert file_sha256(license_file) == source["license_sha256"]
        assert "CC0 1.0 Universal" in license_file.read_text()
        assert len(character.parents) == 42
        vertices = np.concatenate([part.vertices for part in character.parts])
        assert vertices[:, 2].min() == pytest.approx(0, abs=1e-6)
        assert vertices[:, 2].max() == pytest.approx(manifest["height_m"], abs=1e-6)
        for part in character.parts:
            np.testing.assert_allclose(part.weights.sum(axis=1), 1, rtol=0, atol=1e-6)


def test_real_mjcf_skin_mapping_and_deformation_at_fixed_root(characters):
    scene = build_preview(characters)
    model, data = scene.model, scene.data
    assert model.nskin == sum(len(char.parts) for char in characters.values())
    assert model.ngeom == 1 and model.nmocap == 84
    assert np.all(model.body_mocapid[model.skin_bonebodyid] >= 0)

    def native_vertices():
        scene.forward()
        visual = mujoco.MjvScene(model, maxgeom=128)
        mujoco.mjv_updateScene(
            model,
            data,
            mujoco.MjvOption(),
            None,
            mujoco.MjvCamera(),
            mujoco.mjtCatBit.mjCAT_ALL,
            visual,
        )
        return visual.skinvert.copy()

    for name in characters:
        scene.update(name, 1, 2, 0, 1.15, "walk", 0)
    before = native_vertices()
    for name in characters:
        scene.update(name, 1, 2, 0, 1.15, "walk", 0.30)
    after = native_vertices()
    assert before.shape == after.shape and before.size > 10000
    assert np.isfinite(after).all()
    assert np.max(np.abs(after - before)) > 0.05


def test_native_clip_timing_and_loop_continuity(characters):
    for character in characters.values():
        assert set(character.clips) == {"idle", "walk", "run"}
        for name, clip in character.clips.items():
            meta = character.metadata["clips"][name]
            assert meta["fps"] == 24
            np.testing.assert_allclose(np.diff(clip.times), 1 / 24, rtol=0, atol=1e-12)
            assert meta["duration_s"] == clip.duration
            p_before, q_before = clip.sample(clip.duration - 1e-7)
            p_after, q_after = clip.sample(clip.duration + 1e-7)
            np.testing.assert_allclose(p_before, p_after, atol=2e-5)
            np.testing.assert_allclose(
                np.abs(np.sum(q_before * q_after, axis=1)), 1, atol=2e-5
            )


def test_crossfade_continuity_root_pose_and_heading(characters):
    animator = HumanAnimator({"person": characters["male"]})
    before = animator.update("person", 0, 0, 0, 0, "idle", 0.2)
    changed = animator.update("person", 0, 0, 0, 1.15, "walk", 0)
    np.testing.assert_allclose(before.bone_positions, changed.bone_positions, atol=1e-9)
    np.testing.assert_allclose(
        before.bone_quaternions, changed.bone_quaternions, atol=1e-9
    )
    moved = animator.update("person", 3, -4, np.pi / 2, 1.15, "walk", 0)
    assert moved.root_xyz == (3, -4, 0)
    expected = np.column_stack(
        (
            -before.bone_positions[:, 1],
            before.bone_positions[:, 0],
            before.bone_positions[:, 2],
        )
    )
    np.testing.assert_allclose(moved.bone_positions - (3, -4, 0), expected, atol=1e-8)
    walked = animator.update("person", 3, -4, np.pi / 2, 1.15, "walk", 0.25)
    assert np.max(np.abs(walked.bone_positions - moved.bone_positions)) > 0.05


def test_actual_sole_probes_and_stance_units(characters):
    for character in characters.values():
        probes = sole_probes(character)
        assert min(probe.size for probe in probes) > 100
        p, q = forward_kinematics(
            character.rest_p,
            character.rest_q,
            character.parents,
            character.levels,
            (0, 0, 0),
            0,
        )
        pose = type("Pose", (), {"bone_positions": p, "bone_quaternions": q})()
        for probe in probes:
            assert 0 <= probe.position(pose)[:, 2].min() < 0.002
    diagnostic = FootDiagnostics()
    diagnostic.update("a", 0, np.array([[0, 0, -0.01]]), 1.0, 0.0, (0, 0), 0)
    diagnostic.update("a", 0, np.array([[0.1, 0, -0.01]]), 1.0, 0.2, (0.15, 0), 0)
    result = diagnostic.summary()
    assert result["max_penetration_m"] == 0.01
    assert result["stance_sliding_median_mps"] == 0.5


def test_replay_has_all_five_stages_and_reproducible_seeding():
    a, b = (Replay(SPEC, 150, 6, 42) for _ in range(2))
    for second in range(151):
        dt = 0 if not second else 1
        left, right = a.update(second, dt), b.update(second, dt)
        assert left == right
        assert len(left) == (2 if second < 120 else 6)
        assert a.stage == min(4, second // 30)


def test_full_150_second_headless_model_and_animator(characters):
    result = run_demo(SPEC, ASSETS, 150, 6, 42, 30, check_only=True)
    assert result["completed"] and result["simulated_s"] == 150
    assert result["frames"] == 4501 and len(result["stage_frames"]) == 5
    assert result["skins"] == 42 and result["mocap_bodies"] == 252
    assert result["stance_samples"] > 1000
    assert np.isfinite(result["min_sole_z_m"])


def test_corridor_paths_have_continuous_motion_and_heading():
    replay = CorridorReplay(10, 42)
    duplicate = CorridorReplay(10, 42)
    dt = 1 / 60
    before = replay.update(0, 0)
    assert len(before) == 10
    assert before == duplicate.update(0, 0)
    states = set()
    for frame in range(1, 1201):
        after = replay.update(frame * dt, dt)
        xs, ys, headings = np.array([(cmd.x, cmd.y, cmd.heading) for cmd in after]).T
        assert np.all(np.abs(xs) <= 4.75 + 1e-9)
        assert np.all(np.abs(ys) <= 0.95 + 1e-9)
        delta = np.column_stack((xs, ys)) - [(cmd.x, cmd.y) for cmd in before]
        tangent = np.column_stack((np.cos(headings), np.sin(headings)))
        assert np.allclose(delta / dt, replay.speed_mps * tangent, atol=0.012)
        states.update(cmd.state for cmd in after)
        before = after
    assert states == {"walk", "turn"}


def test_corridor_compiles_and_activates_six_actors_from_first_frame(characters):
    result = run_demo(
        SPEC, ASSETS, 0.2, 6, 42, 30, scene_name="corridor", check_only=True
    )
    assert result["completed"] and result["stage_frames"] == {"corridor": 7}
    assert result["active_agents_at_start"] == 6
    assert result["skins"] == 3 * sum(
        len(character.parts) for character in characters.values()
    )
    assert result["visible_geom_count_excluding_skin"] == 14


def test_corridor_camera_controls_while_paused(monkeypatch):
    model = mujoco.MjModel.from_xml_string("<mujoco/>")
    camera = mujoco.MjvCamera()
    camera.azimuth, camera.elevation, camera.distance = 114, -31, 13.8
    callbacks = {}
    for name in ("cursor_pos", "scroll", "key"):
        monkeypatch.setattr(
            glfw,
            f"set_{name}_callback",
            lambda _window, callback, name=name: callbacks.update({name: callback}),
        )
    monkeypatch.setattr(glfw, "get_cursor_pos", lambda _window: (0, 0))
    monkeypatch.setattr(glfw, "get_window_size", lambda _window: (800, 600))
    pressed = glfw.MOUSE_BUTTON_LEFT
    monkeypatch.setattr(
        glfw, "get_mouse_button", lambda _window, button: int(button == pressed)
    )
    preview = PreviewWindow(None, model, None, camera)

    callbacks["key"](None, glfw.KEY_SPACE, 0, glfw.PRESS, 0)
    assert preview.paused
    callbacks["cursor_pos"](None, 100, 40)
    assert camera.azimuth != 114 and camera.elevation != -31
    pressed = glfw.MOUSE_BUTTON_RIGHT
    callbacks["cursor_pos"](None, 120, 60)
    assert np.linalg.norm(camera.lookat) > 0
    callbacks["scroll"](None, 0, 1)
    assert camera.distance < 13.8
    callbacks["key"](None, glfw.KEY_R, 0, glfw.PRESS, 0)
    assert (camera.azimuth, camera.elevation, camera.distance) == (114, -31, 13.8)
    np.testing.assert_array_equal(camera.lookat, np.zeros(3))
    callbacks["key"](None, glfw.KEY_SPACE, 0, glfw.PRESS, 0)
    assert not preview.paused
