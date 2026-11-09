"""Versioned source cards, nominal phase slots, and approved execution bindings."""

import hashlib
import json
from pathlib import Path

from omegaconf import OmegaConf

from .qa import command_at, schedule

LEVELS = ("easy", "critical", "hard")
CARD_IDS = (
    "FA_Corridor_Passing",
    "ND_Doorway_Passing",
    "BC_Corner_Turn",
    "IN_Crossing_Paths",
    "OT_Same_Direction",
    "MG_Small_Group",
)
COUNTS = (48, 540, 900)


def fingerprint(data):
    encoded = json.dumps(data, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(encoded.encode()).hexdigest()


def _yaml(path):
    data = OmegaConf.to_container(OmegaConf.load(path), resolve=True)
    if not isinstance(data, dict) or data.get("schema_version") != 1:
        raise ValueError(f"invalid schema version in {path}")
    return data


def _catalog(repo):
    directory = repo / "configs/benchmarks"
    matrix = _yaml(directory / "matrix.yaml")
    source = repo / "docs/research/HUMAN_MODEL_FINAL.md"
    if hashlib.sha256(source.read_bytes()).hexdigest() != matrix["source_sha256"]:
        raise ValueError("research source fingerprint changed")
    cards = {}
    for path in sorted((directory / "scenarios").glob("*.yaml")):
        card = _yaml(path)
        key = card["id"]
        if key in cards or set(card["widths_m"]) != set(LEVELS):
            raise ValueError(f"duplicate or incomplete card {key}")
        cards[key] = card
    if set(cards) != set(CARD_IDS):
        raise ValueError("research card roster changed")
    if set(matrix["backends"]) != {
        "orca",
        "sfm_jupedsim",
        "avm_jupedsim",
        "csm_jupedsim",
    }:
        raise ValueError("research backend roster changed")
    return matrix, cards


def _slots(matrix):
    for backend in sorted(matrix["backends"]):
        for card in CARD_IDS:
            for level in LEVELS:
                for index in range(20 if level == "critical" else 5):
                    first = (
                        2 if index >= 5 else 0 if level == "easy" and index < 2 else 1
                    )
                    yield backend, card, "a", level, "advance", index, first
            if card in matrix["branch_b_cards"]:
                for action in matrix["branch_b_actions"]:
                    for index in range(5):
                        yield backend, card, "b", None, action, index, 1


def _blockers(card, branch, level, action):
    pending = list(card["blockers"])
    if branch == "b":
        pending.append("branch_b_width_and_advance_reference")
    if (
        card["id"] in ("ND_Doorway_Passing", "IN_Crossing_Paths")
        and action == "reverse"
    ):
        pending.append("hm_spec_004")
    if card["id"] == "ND_Doorway_Passing" and level == "easy":
        pending.append("hm_spec_001")
    if card["id"] == "MG_Small_Group":
        pending.append("hm_spec_005")
    if card["id"] == "OT_Same_Direction" and level == "critical":
        pending.append("hm_spec_006")
    return pending


def _execution(card, level, action, binding):
    if (
        not isinstance(binding, dict)
        or not isinstance(binding.get("approval"), str)
        or not binding["approval"].strip()
    ):
        raise ValueError("execution binding needs an explicit approval reference")
    width = binding["width_m"]
    options = card["widths_m"][level]
    if (options and width not in options) or (
        not options
        and not card["width_range_m"][0] <= width <= card["width_range_m"][1]
    ):
        raise ValueError(f"binding width disagrees with {card['id']}/{level}")
    if action not in card["robot_actions"] and not binding.get("action_deviation"):
        raise ValueError(f"unsupported action {action} needs a deviation reference")
    if binding["density"] not in card["density"]["counts"]:
        raise ValueError("density disagrees with card")
    if (
        binding["human_policy"]
        not in {"neutral", "independent"} & card["human_policies"].keys()
    ):
        raise ValueError(
            "native profiles only support an explicitly approved neutral condition"
        )
    if card.get("subtypes") and binding.get("subtype") not in card["subtypes"]:
        raise ValueError("select an explicit overtaking subtype")
    allowed = ("worlds=", "worlds.", "scenarios=", "scenarios.", "corridor_width=")
    if (
        not isinstance(binding["overrides"], list)
        or not binding["overrides"]
        or any(
            not isinstance(item, str) or not item.startswith(allowed)
            for item in binding["overrides"]
        )
    ):
        raise ValueError("bindings may only override scene geometry and actors")
    if "width_probe_xy_m" not in binding or "commands" not in binding:
        raise ValueError(
            "binding requires measured width point and robot command schedule"
        )
    return binding


def _branch_schedule(control, branch, action):
    start, end = branch["branch_window_ns"]
    if not (
        all(type(tick) is int and tick % 50_000_000 == 0 for tick in (start, end))
        and 0 < start < end < 30_000_000_000
    ):
        raise ValueError("branch requires a finite intervention on robot ticks")
    advance = schedule(control["commands"], 50_000_000, 30_000_000_000)
    alternative = schedule(branch["commands"], 50_000_000, 30_000_000_000)
    points = sorted({*advance[0], *alternative[0], start, end})
    if any(
        command_at(advance, tick) != command_at(alternative, tick)
        for tick in points
        if not start <= tick < end
    ):
        raise ValueError("branch command differs from baseline outside intervention")
    interval = [command_at(alternative, tick) for tick in points if start <= tick < end]
    if action == "stop" and any(any(body) or not wait for body, wait in interval):
        raise ValueError("Stop needs an explicitly logged zero-command wait")
    if action == "reverse" and not any(body[0] < 0 for body, _ in interval):
        raise ValueError("Reverse needs a negative body-X command")
    if all(
        command_at(advance, tick) == command_at(alternative, tick) for tick in points
    ):
        raise ValueError("branch must change the baseline schedule")


def compile_plan(repo: Path, phase: int | str, bindings=None):
    matrix, cards = _catalog(repo)
    bindings = bindings or {}
    if phase == "smoke":
        if bindings:
            raise ValueError("smoke cannot take research bindings")
        smoke = _yaml(repo / "configs/benchmarks/smoke.yaml")
        slots = [
            {
                "slot_id": "benchmark_smoke",
                "backend": smoke["backend"],
                "card_id": None,
                "seed": smoke["seed"],
                "action": "advance",
                "level": None,
                "introduced": None,
                "blocked": [],
                "binding": smoke,
            }
        ]
        payload = {
            "schema_version": 1,
            "phase": phase,
            "matrix": matrix,
            "cards": cards,
            "bindings": {},
            "slots": slots,
        }
    else:
        if phase not in (0, 1, 2):
            raise ValueError("phase must be 0, 1, 2 or smoke")
        by_condition = bindings.get("conditions", {})
        selected = bindings.get("branch_b_levels", {})
        if bindings and bindings.get("schema_version") != 1:
            raise ValueError("binding schema_version must be 1")
        if any(
            card not in matrix["branch_b_cards"] or level not in ("critical", "hard")
            for card, level in selected.items()
        ):
            raise ValueError("invalid branch B geometry selection")
        legal_keys = {
            f"{card}/{level}/{action}"
            for _, card, branch, nominal, action, _, _ in _slots(matrix)
            if (level := selected.get(card) if branch == "b" else nominal) is not None
        }
        if not by_condition.keys() <= legal_keys:
            raise ValueError("binding contains unknown or unselected conditions")
        slots = []
        for backend, card_id, branch, level, action, index, first in _slots(matrix):
            if first > phase:
                continue
            chosen = selected.get(card_id) if branch == "b" else level
            card = cards[card_id]
            binding = by_condition.get(f"{card_id}/{chosen}/{action}")
            blocked = _blockers(card, branch, level, action)
            if binding is not None:
                binding = _execution(card, chosen, action, binding)
                if not {item for item in blocked if item.startswith("hm_spec_")} <= set(
                    binding.get("adjudicated", [])
                ):
                    raise ValueError(
                        "binding must name the adjudicated source conflicts"
                    )
                if branch == "b":
                    control = by_condition.get(f"{card_id}/{chosen}/advance")
                    keys = (
                        "width_m",
                        "width_probe_xy_m",
                        "density",
                        "human_policy",
                        "subtype",
                        "overrides",
                        "shortest_path_m",
                        "shortest_path_source",
                        "label_context",
                    )
                    if control is None or any(
                        control.get(key) != binding.get(key) for key in keys
                    ):
                        raise ValueError(
                            f"branch B {card_id}/{action} does not match branch A baseline"
                        )
                    _branch_schedule(control, binding, action)
                blocked = []
            identity = (backend, card_id, branch, level, action, index)
            slots.append(
                {
                    "slot_id": fingerprint(identity),
                    "backend": backend,
                    "card_id": card_id,
                    "branch": branch,
                    "level": chosen,
                    "action": action,
                    "seed_id": f"seed_{index:03}",
                    "seed": matrix["seed_start"] + index,
                    "seed_split": "evaluation",
                    "advance_slot_id": (
                        fingerprint((backend, card_id, "a", chosen, "advance", index))
                        if branch == "b" and chosen
                        else None
                    ),
                    "introduced": first,
                    "is_new": first == phase,
                    "reused_from_phase": first if first < phase else None,
                    "phase_membership": list(range(first, 3)),
                    "blocked": blocked,
                    "binding": binding,
                }
            )
        if len(slots) != COUNTS[phase] or len(
            {slot["slot_id"] for slot in slots}
        ) != len(slots):
            raise ValueError("nominal phase cardinality is inconsistent")
        payload = {
            "schema_version": 1,
            "phase": phase,
            "matrix": matrix,
            "cards": cards,
            "bindings": bindings,
            "slots": slots,
        }
    payload["plan_id"] = fingerprint(
        (matrix, cards, bindings, smoke if phase == "smoke" else None)
    )
    payload["manifest_sha256"] = fingerprint(payload)
    return payload
