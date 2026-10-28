"""Explicit differences between the three existing Nav2 pilot entry points."""
from dataclasses import dataclass
from pathlib import Path
from dynamuhsim.workspace import stage_directory
import argparse
import json


@dataclass(frozen=True)
class TrialProfile:
    name: str
    partition_prefix: str
    groups: bool = False
    container_worker: bool = False

    def parser(self):
        parser = argparse.ArgumentParser(description=f"Bounded Nav2 trial: {self.name}")
        if self.groups:
            parser.add_argument("--scenario", choices=["walking", "standing"], required=True)
            parser.add_argument("--seed", type=int, choices=[42, 43], required=True)
        elif self.container_worker:
            parser.add_argument("--condition", choices=["people"], default="people")
            parser.add_argument("--model", choices=["crowd_orca_plus", "complex_geometry_mixed"], required=True)
        else:
            parser.add_argument("--condition", choices=["empty", "people"], required=True)
        parser.add_argument("--report-dir", type=Path, required=True)
        parser.add_argument("--gui", action="store_true")
        return parser

    def parse_args(self, argv=None):
        args = self.parser().parse_args(argv)
        if self.groups:
            args.condition = "people"
        return args

    def planning_dt(self, plan, args):
        return plan["native_dt"][args.model] if self.container_worker else plan["native_dt"]

    def result_fields(self, args, dt):
        if self.groups:
            return dict(scenario=args.scenario, seed=args.seed,
                        model="socnavgym_initial_groups_v2", native_dt=dt,
                        response_variant="bounded_reactive_standing_v3" if args.scenario == "standing" else "walking_response_v2")
        if self.container_worker:
            return dict(model=args.model, native_dt=dt,
                        response_variant=args.model + "_frozen_until_goal_accepted")
        return dict(response_variant="v2_frozen_until_goal_accepted")

    def validate_gates(self, benchmark):
        benchmark = Path(benchmark)
        if benchmark.parent.name == "social_navigation":
            benchmark = stage_directory(benchmark.parents[2], self.name).parent
        stage = benchmark / self.name
        gates = [(stage / "reports/gate01/validation.json", "status", "PASS")]
        if self.groups:
            gates = [(stage / p, "status", "PASS") for p in
                     ["reports/reaction_gate.json", "reports/gate03/validation.json", "reports/offline05/validation.json"]]
        if self.groups or self.container_worker:
            gates.append((benchmark / "socnavgym_nav2_pilot/reports/empty01/validation.json", "actual_goal_reached", True))
        for path, key, expected in gates:
            value = json.loads(path.read_text())[key]
            if value != expected:
                raise RuntimeError(f"Prerequisite gate failed: {path}: {key}={value!r}")

    def worker_command(self, social, runtime, socket_path, args):
        stage = stage_directory(social.parents[1], self.name)
        command = [str(social / ".venv-socnavgym/bin/python"), str(stage / "worker.py")]
        if self.groups:
            return command + ["--runtime", str(runtime), "--socket", socket_path,
                              "--scenario", args.scenario, "--seed", str(args.seed)]
        if self.container_worker:
            raise ValueError("Container command must come from the pinned backend runtime")
        return command + ["--common", str(social / "benchmark/common_response"), "--runtime", str(runtime),
                          "--socket", socket_path, "--model", "soc_guarded_orca", "--case", "crossing_near"]


PROFILES = {
    "socnavgym_nav2_pilot": TrialProfile("socnavgym_nav2_pilot", "dynamuhsim_nav2_v2_"),
    "socnavgym_group_nav2": TrialProfile("socnavgym_group_nav2", "dynamuhsim_group_nav2_", groups=True),
    "shared_nav2_pilot": TrialProfile("shared_nav2_pilot", "dynamuhsim_shared_nav2_", container_worker=True),
}
