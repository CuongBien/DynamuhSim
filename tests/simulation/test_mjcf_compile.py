from dataclasses import astuple

import mujoco
import pytest

from narrow_nav.simulation.configuration import compose_config
from narrow_nav.simulation.geometry_inspector import inspect_world
from narrow_nav.simulation.world_builder import MJCF_ROOT, build_world


@pytest.mark.parametrize(
    ("world", "scenario"),
    [
        ("corridor", "head_on"),
        ("blind_corner", "blind_corner"),
        ("passing_bay", "passing_bay"),
    ],
)
def test_default_and_parametric_scenes_compile_headless_from_another_directory(
    world, scenario, tmp_path, monkeypatch
):
    monkeypatch.chdir(tmp_path)
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
    config = compose_config(
        MJCF_ROOT.parents[1] / "configs",
        [f"worlds={world}", f"scenarios={scenario}"],
    )
    built = build_world(config)
    reference_path = MJCF_ROOT / "worlds" / world / "main.xml"
    reference = inspect_world(reference_path)
    generated = inspect_world(built)
    for expected, actual in (
        (reference.floors, generated.floors),
        (reference.walls, generated.walls),
    ):
        for baseline_box, parametric_box in zip(expected, actual, strict=True):
            assert astuple(parametric_box) == pytest.approx(
                astuple(baseline_box), abs=1e-9
            )

    model = mujoco.MjModel.from_xml_string(built.mjcf, dict(built.visual_assets))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert model.nu == 2 and model.nskin > 0
    assert model.nmocap == 1 + sum(
        len(actor.character.parents) for actor in built.skin_actors
    )
    for actor in (built.scene.robot, *built.scene.humans):
        assert generated.contains(actor.pose.x, actor.pose.y, actor.radius_m)
        assert generated.contains(actor.goal.x, actor.goal.y, actor.radius_m)
        assert tuple(data.body(actor.entity_id).xpos[:2]) == pytest.approx(
            (actor.pose.x, actor.pose.y)
        )
        assert tuple(data.site(f"{actor.entity_id}/goal").xpos[:2]) == pytest.approx(
            (actor.goal.x, actor.goal.y)
        )
    for contact in data.contact:
        assert 1 not in (
            model.geom_group[contact.geom1],
            model.geom_group[contact.geom2],
        )
