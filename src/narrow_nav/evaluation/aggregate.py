"""Denominators and paired uncertainty over completed, identifiable records."""

from collections import defaultdict
from math import sqrt
from random import Random
from statistics import fmean

Z95 = 1.959963984540054


def wilson(successes, total):
    if not total:
        return None
    if not 0 <= successes <= total:
        raise ValueError("invalid binomial counts")
    p = successes / total
    d = 1 + Z95**2 / total
    center = (p + Z95**2 / (2 * total)) / d
    half = Z95 * sqrt((p * (1 - p) + Z95**2 / (4 * total)) / total) / d
    return max(0.0, center - half), min(1.0, center + half)


def paired_interval(clusters, *, draws=2000, seed=812):
    """Equal-weight cluster bootstrap. One cluster cannot support an interval."""
    if not clusters:
        return None
    means = [fmean(clusters[key]) for key in sorted(clusters)]
    if len(means) < 2:
        return {"mean_delta": means[0], "ci95": None, "clusters": 1}
    rng = Random(seed)
    values = sorted(fmean(rng.choices(means, k=len(means))) for _ in range(draws))
    return {
        "mean_delta": fmean(means),
        "clusters": len(means),
        "ci95": (values[int(0.025 * (draws - 1))], values[int(0.975 * (draws - 1))]),
    }


def condition_summary(records):
    planned = len(records)
    evaluated = [r for r in records if r.get("infrastructure_valid") is True]
    eligible = [r for r in evaluated if r.get("scenario_label_valid") is True]
    research = [r for r in evaluated if r.get("card_id") is not None]
    tasks = [r["S"] for r in research if r["S"] is not None]
    collisions = [r["C"] > 0 for r in research if r["C"] is not None]
    labeled_tasks = [r["S"] for r in eligible if r["S"] is not None]
    return {
        "planned": planned,
        "attempted_slots": sum(r["attempts"] > 0 for r in records),
        "infrastructure_valid": len(evaluated),
        "invalid_or_interrupted": sum(
            r["attempts"] > 0 and r.get("infrastructure_valid") is not True
            for r in records
        ),
        "label_eligible": len(eligible),
        "pass": sum(r.get("episode_task_pass") is True for r in eligible),
        "labeled_sr": sum(labeled_tasks) / len(labeled_tasks)
        if labeled_tasks
        else None,
        "labeled_sr_n": len(labeled_tasks),
        "labeled_sr_ci95": wilson(sum(labeled_tasks), len(labeled_tasks)),
        "sr": sum(tasks) / len(tasks) if tasks else None,
        "sr_n": len(tasks),
        "sr_ci95": wilson(sum(tasks), len(tasks)),
        "cr": sum(collisions) / len(collisions) if collisions else None,
        "cr_n": len(collisions),
        "cr_ci95": wilson(sum(collisions), len(collisions)),
        "timeouts": sum(r.get("TO") is True for r in evaluated),
    }


def paired_comparisons(records, backends):
    grouped = defaultdict(dict)
    for row in records:
        if row["paired_key"]:
            group = grouped[row["paired_key"]]
            if row["backend"] in group:
                raise ValueError("duplicate paired key within one backend")
            group[row["backend"]] = row
    comparisons = []
    for left, right in (
        (a, b) for i, a in enumerate(backends) for b in backends[i + 1 :]
    ):
        eligible = 0
        metrics = (
            "path_length_m",
            "spl",
            "psc_fraction",
            "min_ttc_s",
            "S",
            "min_human_center_distance_m",
            "jerk_max_mps3",
        )
        clusters = {metric: defaultdict(list) for metric in metrics}
        opportunities = sum(
            bool({left, right} & group.keys()) for group in grouped.values()
        )
        for group in grouped.values():
            if not {left, right} <= group.keys():
                continue
            a, b = group[left], group[right]
            if any(
                not row.get("infrastructure_valid")
                or row.get("scenario_label_valid") is not True
                for row in (a, b)
            ):
                continue
            eligible += 1
            for metric in metrics:
                if (
                    a.get(metric) is not None
                    and b.get(metric) is not None
                    and (
                        metric == "S"
                        or all(row.get("episode_task_pass") is True for row in (a, b))
                    )
                ):
                    clusters[metric][(a["card_id"], a["seed"])].append(
                        a[metric] - b[metric]
                    )
        comparisons.append(
            {
                "left": left,
                "right": right,
                "eligible_pairs": eligible,
                "matched_or_one_sided_keys": opportunities,
                "excluded_pairs": opportunities - eligible,
                "path_length_delta_m": paired_interval(clusters["path_length_m"]),
                "spl_delta": paired_interval(clusters["spl"]),
                "psc_fraction_delta": paired_interval(clusters["psc_fraction"]),
                "min_ttc_delta_s": paired_interval(clusters["min_ttc_s"]),
                "min_human_center_distance_delta_m": paired_interval(
                    clusters["min_human_center_distance_m"]
                ),
                "jerk_max_delta_mps3": paired_interval(clusters["jerk_max_mps3"]),
                "success_delta": paired_interval(clusters["S"]),
            }
        )
    return comparisons
