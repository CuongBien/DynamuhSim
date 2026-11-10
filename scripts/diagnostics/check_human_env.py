"""Run small native smoke tests for the four human model dependencies."""

import argparse
import inspect
import json
import math
import platform
import sys
import sysconfig
from functools import partial
from importlib import import_module, metadata
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
MJCF = Path("sim/mujoco/worlds/corridor/main.xml")
DT = 0.01


def signatures(**objects):
    result = {}
    for name, obj in objects.items():
        try:
            result[name] = str(inspect.signature(obj))
        except (TypeError, ValueError):
            result[name] = "unavailable through inspect.signature"
    return result


def smoke_mujoco(module):
    model = module.MjModel.from_xml_path(str(REPO_ROOT / MJCF))
    data = module.MjData(model)
    module.mj_step(model, data)
    if model.ngeom <= 0 or not math.isclose(data.time, model.opt.timestep):
        raise RuntimeError(
            "MJCF geometry or first physics step did not match expectations"
        )
    return {
        "mjcf": MJCF.as_posix(),
        "runtime_version": module.mj_versionString(),
        "geoms": model.ngeom,
        "elapsed_s": data.time,
    }


def smoke_jupedsim(module, model_name, **model_options):
    parameters_name = f"{model_name}AgentParameters"
    model_type = getattr(module, model_name)
    parameters_type = getattr(module, parameters_name)
    simulation = module.Simulation(
        model=model_type(**model_options),
        geometry=[(-2, -2), (4, -2), (4, 2), (-2, 2)],
        dt=DT,
    )
    stage = simulation.add_waypoint_stage((2.0, 0.0), 0.1)
    journey = simulation.add_journey(module.JourneyDescription([stage]))
    agent = simulation.add_agent(
        parameters_type(position=(0.0, 0.0), journey_id=journey, stage_id=stage)
    )
    simulation.iterate(5)
    position = tuple(simulation.agent(agent).position)
    elapsed = simulation.elapsed_time()
    if not (
        math.isclose(elapsed, 5 * DT)
        and all(map(math.isfinite, position))
        and position[0] > 0
    ):
        raise RuntimeError(
            f"{model_name} did not advance toward the waypoint: {position}, {elapsed=}"
        )
    return {
        "position_m": position,
        "elapsed_s": elapsed,
        "api": signatures(
            **{model_name: model_type, parameters_name: parameters_type},
            Simulation=module.Simulation,
            add_agent=module.Simulation.add_agent,
            iterate=module.Simulation.iterate,
        ),
    }


def smoke_rvo2(module):
    simulator = module.PyRVOSimulator(DT, 2.0, 5, 1.5, 2.0, 0.2, 1.0)
    agent = simulator.addAgent((0.0, 0.0))
    simulator.setAgentPrefVelocity(agent, (1.0, 0.0))
    simulator.doStep()
    position = simulator.getAgentPosition(agent)
    elapsed = simulator.getGlobalTime()
    # RVO2 exposes single-precision floats.
    if not (
        math.isclose(elapsed, DT, rel_tol=1e-6)
        and math.isclose(position[0], DT, rel_tol=1e-6)
        and math.isclose(position[1], 0.0, abs_tol=1e-6)
    ):
        raise RuntimeError(f"RVO2 did not advance as expected: {position}, {elapsed=}")
    return {
        "position_m": position,
        "elapsed_s": elapsed,
        "api": signatures(
            PyRVOSimulator=module.PyRVOSimulator,
            addAgent=module.PyRVOSimulator.addAgent,
            setAgentPrefVelocity=module.PyRVOSimulator.setAgentPrefVelocity,
            doStep=module.PyRVOSimulator.doStep,
        ),
    }


def run_check(module_name, distribution, smoke, strict, expected=None):
    result = {"module": module_name, "distribution": distribution, "version": None}
    try:
        try:
            dist = metadata.distribution(distribution)
        except metadata.PackageNotFoundError:
            dist = None
        if dist is not None:
            result["version"] = dist.version
            result["wheel_tags"] = [
                line.removeprefix("Tag: ")
                for line in (dist.read_text("WHEEL") or "").splitlines()
                if line.startswith("Tag: ")
            ]
        module = import_module(module_name)
        result["origin"] = module.__file__
        result.update(smoke(module))

        if module_name == "mujoco":
            major = int(result["runtime_version"].split(".", 1)[0])
            if major < 4:
                raise RuntimeError("Native MuJoCo must satisfy sim extra (mujoco>=4.0)")
            if strict and (dist is None or dist.version != result["runtime_version"]):
                raise RuntimeError(
                    "MuJoCo distribution version must match the native runtime"
                )
        elif strict and (dist is None or dist.version != expected):
            raise RuntimeError(
                f"Expected {distribution}=={expected}, found {result['version']}"
            )
    # Keep collecting evidence when a native binding or one of its imports raises.
    except Exception as error:  # noqa: BLE001
        return {**result, "status": "FAIL", "error": f"{type(error).__name__}: {error}"}
    return {**result, "status": "PASS"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true", help="emit a JSON report")
    parser.add_argument(
        "--strict",
        action="store_true",
        help="also verify distribution version metadata",
    )
    args = parser.parse_args()
    checks = {"mujoco": run_check("mujoco", "mujoco", smoke_mujoco, args.strict)}
    for label, smoke in (
        ("sfm", partial(smoke_jupedsim, model_name="SocialForceModel")),
        (
            "avm",
            partial(smoke_jupedsim, model_name="AnticipationVelocityModel", rng_seed=0),
        ),
        ("csm", partial(smoke_jupedsim, model_name="CollisionFreeSpeedModelV2")),
    ):
        checks[label] = run_check("jupedsim", "jupedsim", smoke, args.strict, "1.4.2")
    checks["orca"] = run_check("rvo2", "pyrvo2", smoke_rvo2, args.strict, "0.0.0")
    passed = all(check["status"] == "PASS" for check in checks.values())
    report = {
        "executable": sys.executable,
        "python": platform.python_version(),
        "platform": platform.platform(),
        "soabi": sysconfig.get_config_var("SOABI"),
        "strict": args.strict,
        "status": "PASS" if passed else "FAIL",
        "checks": checks,
    }
    if args.json:
        print(json.dumps(report, indent=2, allow_nan=False))
    else:
        print(
            f"{report['status']} | {sys.executable} | Python {report['python']} | {report['soabi']}"
        )
        for label, check in checks.items():
            detail = (
                check.get("error") or f"{check['distribution']}=={check['version']}"
            )
            print(f"{check['status']} {label}: {detail}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
