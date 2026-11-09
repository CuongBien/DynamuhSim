import json
from argparse import Namespace
from collections import Counter
from copy import deepcopy
from math import pi
from pathlib import Path

import pytest

from narrow_nav.evaluation.aggregate import (
    condition_summary,
    paired_comparisons,
    wilson,
)
from narrow_nav.evaluation.metrics import contacts_by_kind, evaluate, ttc
from narrow_nav.evaluation.qa import scenario_label, schedule
from narrow_nav.evaluation.reporting import report_run
from narrow_nav.evaluation.runner import write_json
from narrow_nav.evaluation.scenario_cards import CARD_IDS, compile_plan
from scripts.benchmarks.plan import _manifest, _reuse, _select

ROOT = Path(__file__).resolve().parents[2]
DT = 50_000_000
TIMING = {"robot_control_dt_ns": DT, "physics_dt_ns": 2_000_000}
GOALS = {"robot": (2, 0), "human": (4, 0)}
RADII = {"robot": 0.18, "human": 0.25}
UNOBSERVABLE = ("human_wall", "human_human")


def test_phase_slots_are_cumulative_and_blocked_not_adjudicated():
    plans = [compile_plan(ROOT, phase) for phase in range(3)]
    ids = [{s["slot_id"] for s in p["slots"]} for p in plans]
    assert [len(s) for s in ids] == [48, 540, 900]
    assert ids[0] < ids[1] < ids[2]
    assert [sum(s["is_new"] for s in p["slots"]) for p in plans] == [48, 492, 360]
    assert len({p["plan_id"] for p in plans}) == 1
    assert all(s["blocked"] and s["binding"] is None for p in plans for s in p["slots"])
    assert Counter(s["branch"] for s in plans[1]["slots"]) == {"a": 360, "b": 180}
    assert all(s["level"] is None for s in plans[1]["slots"] if s["branch"] == "b")
    assert {s["seed"] for s in plans[0]["slots"]} == {1000, 1001}
    assert {s["seed"] for s in plans[2]["slots"] if s["level"] == "critical"} == set(
        range(1000, 1020)
    )
    assert all(s["level"] == "critical" for s in plans[2]["slots"] if s["is_new"])
    assert compile_plan(ROOT, 2) == plans[2]


def test_binding_controls_and_explicit_branch():
    card = CARD_IDS[0]
    advance = {
        "approval": "unit-test only",
        "density": 1,
        "human_policy": "neutral",
        "width_m": 1.2,
        "width_probe_xy_m": [0, 0],
        "overrides": ["worlds.width_m=1.2", "scenarios.id=fa_corridor_passing"],
        "commands": [{"at_ns": 0, "body": [0.35, 0, 0]}],
    }
    stop = {
        **advance,
        "branch_window_ns": [4_000_000_000, 6_000_000_000],
        "commands": [
            *advance["commands"],
            {"at_ns": 4_000_000_000, "body": [0, 0, 0], "intentional_wait": True},
            {"at_ns": 6_000_000_000, "body": [0.35, 0, 0]},
        ],
    }
    bindings = {
        "schema_version": 1,
        "branch_b_levels": {card: "critical"},
        "conditions": {
            f"{card}/critical/advance": advance,
            f"{card}/critical/stop": stop,
        },
    }
    plan = compile_plan(ROOT, 1, bindings)
    enabled = [s for s in plan["slots"] if not s["blocked"]]
    assert len(enabled) == 40
    assert all(
        s["advance_slot_id"] in {r["slot_id"] for r in enabled}
        for s in enabled
        if s["branch"] == "b"
    )
    mismatch = deepcopy(bindings)
    mismatch["conditions"][f"{card}/critical/stop"]["commands"][-1]["body"][0] = 0.3
    with pytest.raises(ValueError, match="outside intervention"):
        compile_plan(ROOT, 1, mismatch)
    mismatch = deepcopy(bindings)
    mismatch["conditions"][f"{card}/critical/stop"]["overrides"] = ["humans=orca"]
    with pytest.raises(ValueError, match="only override"):
        compile_plan(ROOT, 1, mismatch)


def test_manifest_tampering_and_filter_are_explicit(tmp_path):
    path = tmp_path / "plan.json"
    plan = compile_plan(ROOT, 0)
    write_json(path, plan)
    assert _manifest(path) == plan
    assert (
        len(
            _select(
                plan, Namespace(episode=None, backend="orca", scenario=None, seed=1000)
            )
        )
        == 6
    )
    plan["slots"][0]["seed"] = 0
    write_json(path, plan)
    with pytest.raises(ValueError, match="deterministic compiler"):
        _manifest(path)


def actor(x, velocity=0, yaw=0):
    return {"xy": (x, 0), "velocity": (velocity, 0), "yaw": yaw}


def row(index, x, speed=0, human=4, wait=False):
    return {
        "time_ns": index * DT,
        "robot": actor(x, speed),
        "humans": {"human": actor(human, yaw=pi)},
        "clearance": {
            "robot_wall_m": 1,
            "human_wall_m": 1,
            "robot_human_m": abs(x - human) - 0.43,
            "human_human_m": None,
        },
        "intentional_wait": wait,
        "guard_substeps": 0,
    }


def metrics(rows, events=(), **changes):
    args = {
        "timing": TIMING,
        "end_reason": "goal_reached",
        "infra_valid": True,
        "goals": GOALS,
        "radii": RADII,
        "goal_tolerance": 0.1,
        "solver_unobservable": UNOBSERVABLE,
        "shortest_path_m": 2.0,
        "label_valid": True,
        **changes,
    }
    return evaluate(rows, events, **args)


def test_motion_formula_and_unavailable_values():
    result = metrics([row(0, 0), row(1, 1, 0.5), row(2, 2, 1.25)])
    assert result["episode_task_pass"] is True
    quality = result["quality"]
    assert quality["spl"] == 1 and quality["robot"]["path_length_m"] == 2
    assert quality["robot"]["jerk_mps3"]["max"] == pytest.approx(100)
    assert quality["human_goal_s"] == {"human": 0.0}
    assert result["solver_contact_episodes"]["human_wall"] is None
    assert (
        metrics([row(0, 0), row(1, 2)], shortest_path_m=None)["quality"]["spl"] is None
    )
    assert ttc((1.5, 0), (-1, 0), 0.5) == pytest.approx(1)
    assert ttc((1.5, 0), (1, 0), 0.5) is None
    assert ttc((0.4, 0), (0, 0), 0.5) == 0


def test_safety_timeout_and_valid_intentional_wait():
    rows = [row(i, 0, wait=i <= 20) for i in range(122)]
    result = metrics(rows, end_reason="time_budget")
    assert result["hard"] == {"S": False, "C": 0, "TO": True, "FP": 1}
    assert result["outcome"] == "TIMEOUT" and result["episode_task_pass"] is False
    assert result["quality"]["longest_robot_progress_freeze_s"] == pytest.approx(5.05)
    assert metrics(rows[:120], end_reason="time_budget")["hard"]["FP"] == 0
    bad = metrics([], end_reason=None, infra_valid=False)
    assert bad["hard"] == dict.fromkeys(("S", "C", "TO", "FP"))
    assert bad["outcome"] == "INVALID_INFRA" and bad["quality"] is None
    near = metrics(
        [row(0, 0, human=0.49), row(1, 0, human=0.48), row(2, 1, human=0.48)]
    )
    assert near["quality"]["encounters"] == 1
    assert near["quality"]["psc_fraction"] == 0.5


def test_solver_episodes_are_not_substep_samples():
    contact = {"kind": "robot_human", "first_id": "robot", "second_id": "human"}
    events = [{**contact, "time_ns": t} for t in (2_000_000, 4_000_000, 8_000_000)]
    samples, episodes = contacts_by_kind(events, 2_000_000)
    assert samples["robot_human"] == 3 and episodes["robot_human"] == 2
    actual = metrics([row(0, 0), row(1, 1)], events, end_reason="robot_contact")
    assert actual["outcome"] == "SAFETY_FAILURE"
    assert actual["hard"]["C"] == 2


def test_label_and_schedule_distinguish_measurement_from_missing():
    cards = compile_plan(ROOT, 0)["cards"]
    assert scenario_label(cards[CARD_IDS[0]], {}, [row(0, 0, human=6)]) is True
    assert scenario_label(cards[CARD_IDS[0]], {}, [row(0, 0, human=4)]) is False
    assert scenario_label(cards[CARD_IDS[1]], {}, [row(0, 0, human=4)]) is None
    door = {
        "label_context": {"center_xy_m": [0, 0], "crossing_axis": 0},
        "width_m": 1.2,
    }
    assert (
        scenario_label(
            cards[CARD_IDS[1]], door, [row(0, -1, human=1), row(1, 1, human=-1)]
        )
        is True
    )
    with pytest.raises(ValueError, match="resume"):
        schedule(
            [{"at_ns": 0, "body": [0, 0, 0], "intentional_wait": True}], DT, 10 * DT
        )


def test_attempts_are_retained_and_reuse_fails_closed(tmp_path):
    root = tmp_path / "run"
    root.mkdir()
    plan = compile_plan(ROOT, "smoke")
    write_json(root / "run.json", {"plan_id": plan["plan_id"]})
    write_json(root / "phase_smoke.json", plan)
    attempt = root / "attempts/benchmark_smoke/attempt-0001"
    retry = attempt.with_name("attempt-0002")
    attempt.mkdir(parents=True)
    retry.mkdir()
    write_json(attempt / "intent.json", {"execution_key": "stable"})
    assert not _reuse([attempt], "stable")
    write_json(retry / "intent.json", {"execution_key": "stable"})
    write_json(retry / "manifest.json", {"paired_key": "smoke", "episode_id": "test:0"})
    write_json(
        retry / "validity.json",
        metrics([row(0, 0), row(1, 1)], end_reason="time_budget"),
    )
    assert _reuse([attempt, retry], "stable")
    with pytest.raises(ValueError, match="fingerprint"):
        _reuse([attempt, retry], "changed")
    report = report_run(root, tmp_path / "report")
    assert report["attempt_count"] == 2 and report["outcomes"] == {"TIMEOUT": 1}
    assert "INTERRUPTED" in (tmp_path / "report/qa.csv").read_text()


def test_report_denominators_and_paired_clusters():
    def result(backend, seed, distance, **extra):
        return {
            "backend": backend,
            "paired_key": str(seed),
            "card_id": CARD_IDS[0],
            "seed": seed,
            "attempts": 1,
            "infrastructure_valid": True,
            "scenario_label_valid": True,
            "episode_task_pass": True,
            "S": True,
            "C": 0,
            "path_length_m": distance,
            **extra,
        }

    records = [
        result("a", 1, 4),
        result("b", 1, 3),
        result("a", 2, 5),
        result("b", 2, 4),
        result("b", 3, 4),
    ]
    pairs = paired_comparisons(records, ["a", "b"])[0]
    assert pairs["eligible_pairs"] == 2 and pairs["excluded_pairs"] == 1
    assert pairs["path_length_delta_m"]["ci95"] == (1, 1)
    assert condition_summary(records)["sr_n"] == 5
    records[0]["scenario_label_valid"] = None
    assert condition_summary(records)["sr_n"] == 5
    assert condition_summary(records)["labeled_sr_n"] == 4
    assert wilson(0, 0) is None


def test_known_behavior_evidence_stays_separate_from_infrastructure():
    path = ROOT / "tests/fixtures/benchmark/behavior_regressions.json"
    data = json.loads(path.read_text())
    crossing = data["avm_crossing"]
    assert crossing["pre_goal_freeze_s"] > 5 and crossing["guard_substeps"] > 0
    assert crossing["end_reason"] == "time_budget" and crossing["contacts"] == 0
    assert (
        crossing["final_native_prediction_separation_m"]
        < 0.5
        < crossing["final_gt_separation_m"]
    )
    assert (
        data["csm_contested"]["sequential_clearance_m"]
        > 0
        > data["csm_contested"]["side_by_side_slack_m"]
    )
    assert data["csm_contested"]["failures"] == ["human_goal_not_reached"]
    assert (
        data["native_velocity_delta_m"]["avm"]
        > 0
        == data["native_velocity_delta_m"]["csm"]
    )
