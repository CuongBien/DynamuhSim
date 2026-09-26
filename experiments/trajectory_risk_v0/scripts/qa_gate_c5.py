#!/usr/bin/env python3
import argparse
import csv
import json
import math
from pathlib import Path
import sys

BASE = Path.home() / "nav_ws/experiments/trajectory_risk_v0"
DATA = BASE / "data"

FEATURES = DATA / "candidate_features_gate_c5.csv"
OUTCOMES = DATA / "rollout_outcomes_gate_c5.csv"
TRACES = DATA / "rollout_traces_gate_c5.csv"

CANDIDATES = ("C0", "C1", "C2", "C3", "C4")
HUMANS = tuple(f"human_{i:02d}" for i in range(1, 5))


def read_episode(path, episode_id):
    if not path.exists():
        raise FileNotFoundError(path)

    with path.open(newline="") as f:
        rows = [
            row
            for row in csv.DictReader(f)
            if int(row["episode_id"]) == int(episode_id)
        ]

    return rows


def finite(x):
    return math.isfinite(float(x))


def angle_error(a, b):
    d = float(a) - float(b)
    return abs(math.atan2(math.sin(d), math.cos(d)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episode", type=int, required=True)
    ap.add_argument("--path-ratio-min", type=float, default=0.85)
    ap.add_argument("--path-ratio-max", type=float, default=1.10)
    ap.add_argument("--pos-tol", type=float, default=0.01)
    ap.add_argument("--z-tol", type=float, default=0.01)
    ap.add_argument("--yaw-tol", type=float, default=0.02)
    args = ap.parse_args()

    features = read_episode(FEATURES, args.episode)
    outcomes = read_episode(OUTCOMES, args.episode)
    traces = read_episode(TRACES, args.episode)

    result = {
        "episode_id": args.episode,
        "pass": True,
        "checks": {},
        "candidates": {},
    }
    errors = []

    fmap = {r["candidate_id"]: r for r in features}
    omap = {r["candidate_id"]: r for r in outcomes}

    missing = [
        c for c in CANDIDATES
        if c not in fmap or c not in omap
    ]
    if missing:
        errors.append(f"Missing candidates: {missing}")

    # 1) LiDAR finite.
    lidar_ok = True
    for c in CANDIDATES:
        if c not in omap:
            continue
        o = omap[c]
        vals = {
            k: float(o[k])
            for k in (
                "rollout_front_min",
                "rollout_left_min",
                "rollout_right_min",
            )
        }
        scans = int(float(o.get("rollout_scan_samples", 0)))
        ok = all(math.isfinite(v) and v >= 0 for v in vals.values()) and scans > 0
        lidar_ok &= ok
        result["candidates"].setdefault(c, {})["lidar"] = {
            **vals,
            "scan_samples": scans,
            "pass": ok,
        }
        if not ok:
            errors.append(f"{c}: rollout LiDAR invalid: {vals}, scans={scans}")

    result["checks"]["lidar_finite"] = lidar_ok

    # 2) Path length close to commanded path.
    path_ok = True
    ratio_consistency_ok = True
    for c in CANDIDATES:
        if c not in omap:
            continue
        o = omap[c]
        actual = float(o["actual_world_path_length"])
        expected = float(o["expected_distance"])
        saved_ratio = float(o["progress_ratio"])
        computed = actual / expected if expected > 0 else float("nan")

        ok = (
            math.isfinite(actual)
            and math.isfinite(expected)
            and expected > 0
            and args.path_ratio_min <= computed <= args.path_ratio_max
        )
        path_ok &= ok

        ratio_ok = (
            math.isfinite(saved_ratio)
            and abs(saved_ratio - computed) <= 1e-6
        )
        ratio_consistency_ok &= ratio_ok

        result["candidates"].setdefault(c, {})["path"] = {
            "actual_m": actual,
            "expected_m": expected,
            "ratio": computed,
            "saved_progress_ratio": saved_ratio,
            "path_pass": ok,
            "ratio_consistent": ratio_ok,
        }

        if not ok:
            errors.append(
                f"{c}: path ratio={computed:.4f} outside "
                f"[{args.path_ratio_min}, {args.path_ratio_max}]"
            )
        if not ratio_ok:
            errors.append(
                f"{c}: progress_ratio={saved_ratio:.9f} != "
                f"actual/expected={computed:.9f}"
            )

    result["checks"]["path_length"] = path_ok
    result["checks"]["progress_ratio_consistent"] = ratio_consistency_ok

    # 3) Same initial state C0..C4.
    state_ok = True
    if "C0" in fmap:
        ref = fmap["C0"]
        max_xy = 0.0
        max_z = 0.0
        max_yaw = 0.0

        prefixes = ("robot",) + HUMANS

        for c in CANDIDATES[1:]:
            if c not in fmap:
                continue
            row = fmap[c]

            for prefix in prefixes:
                for axis in ("x", "y"):
                    key = f"{prefix}_world_{axis}0"
                    e = abs(float(row[key]) - float(ref[key]))
                    max_xy = max(max_xy, e)
                    if e > args.pos_tol:
                        state_ok = False
                        errors.append(
                            f"{c}: {key} error={e:.5f}m > {args.pos_tol:.5f}m"
                        )

                zkey = f"{prefix}_world_z0"
                e = abs(float(row[zkey]) - float(ref[zkey]))
                max_z = max(max_z, e)
                if e > args.z_tol:
                    state_ok = False
                    errors.append(
                        f"{c}: {zkey} error={e:.5f}m > {args.z_tol:.5f}m"
                    )

                ykey = f"{prefix}_world_yaw0"
                e = angle_error(row[ykey], ref[ykey])
                max_yaw = max(max_yaw, e)
                if e > args.yaw_tol:
                    state_ok = False
                    errors.append(
                        f"{c}: {ykey} error={e:.5f}rad > {args.yaw_tol:.5f}rad"
                    )

        result["checks"]["same_initial_state"] = {
            "pass": state_ok,
            "max_xy_error_m": max_xy,
            "max_z_error_m": max_z,
            "max_yaw_error_rad": max_yaw,
        }
    else:
        state_ok = False
        result["checks"]["same_initial_state"] = {"pass": False}
        errors.append("C0 feature row missing.")

    # 4) Trace completeness sanity.
    trace_counts = {}
    trace_ok = True
    for c in CANDIDATES:
        count = sum(1 for r in traces if r["candidate_id"] == c)
        trace_counts[c] = count
        if count < 25:
            trace_ok = False
            errors.append(f"{c}: only {count} trace rows (<25).")

    result["checks"]["trace_rows"] = {
        "pass": trace_ok,
        "counts": trace_counts,
    }

    result["pass"] = (
        not missing
        and lidar_ok
        and path_ok
        and ratio_consistency_ok
        and state_ok
        and trace_ok
    )
    result["errors"] = errors

    report = DATA / f"gate_c5_episode_{args.episode:03d}.json"
    report.write_text(
        json.dumps(result, indent=2),
        encoding="utf-8",
    )

    print("\n=== GATE C.5 OFFLINE QA ===")
    print(f"Episode                    : {args.episode}")
    print(f"LiDAR finite               : {'PASS' if lidar_ok else 'FAIL'}")
    print(f"Path length near commanded : {'PASS' if path_ok else 'FAIL'}")
    print(f"progress_ratio consistent  : {'PASS' if ratio_consistency_ok else 'FAIL'}")
    print(f"Same initial state         : {'PASS' if state_ok else 'FAIL'}")
    print(f"Trace completeness         : {'PASS' if trace_ok else 'FAIL'}")

    print("\nCandidate path QA:")
    print(f"{'Cand':<6}{'actual':>10}{'expect':>10}{'ratio':>10}{'LiDAR':>10}")
    for c in CANDIDATES:
        if c not in omap:
            continue
        p = result["candidates"][c]["path"]
        l = result["candidates"][c]["lidar"]
        print(
            f"{c:<6}"
            f"{p['actual_m']:>10.3f}"
            f"{p['expected_m']:>10.3f}"
            f"{p['ratio']:>10.3f}"
            f"{('PASS' if l['pass'] else 'FAIL'):>10}"
        )

    print(f"\nReport: {report}")

    if result["pass"]:
        print("\n[GATE C.5 PASS]")
        return 0

    print("\n[GATE C.5 FAIL]")
    for e in errors:
        print(f"  - {e}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
