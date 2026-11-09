"""Record one MuJoCo episode with a scheduled D435i depth stream."""

from argparse import ArgumentParser
from dataclasses import asdict
from json import dumps
from os import environ
from pathlib import Path
from platform import platform
from tempfile import TemporaryDirectory
from typing import TextIO

import numpy as np
from omegaconf import DictConfig

from narrow_nav.core.actions import RobotAction
from narrow_nav.core.observations import Observation, SensorObservation, SensorPayload
from narrow_nav.core.protocols import SimulationStepResult
from narrow_nav.core.types import Twist2D

from .configuration import compose_config
from .depth_config import DEPTH_SENSOR_ID
from .depth_report import write_depth_report
from .mujoco_engine import MuJoCoEngine

_STOP = Twist2D(0, 0, 0)


def _write_depth(path: Path, payload: SensorPayload, **metadata: object) -> None:
    np.savez_compressed(
        path,
        depth_m=np.asarray(payload.values, dtype=np.float32).reshape(payload.shape),
        valid_mask=np.asarray(payload.valid_mask, dtype=np.bool_).reshape(
            payload.shape
        ),
        **metadata,
    )


def _write_boundary(
    run: Path,
    observations: TextIO,
    ground_truth: TextIO,
    actions: TextIO,
    engine: MuJoCoEngine,
    observation: Observation,
    samples: tuple[SensorObservation, ...],
    step: SimulationStepResult | None = None,
) -> None:
    stamp = observation.stamp
    header = {
        "schema_version": stamp.schema_version,
        "episode_id": stamp.episode_id,
    }
    sample_indices = []
    for sample in samples:
        if sample.payload is None:
            raise ValueError(
                "record_depth requires a payload for every scheduled depth sample"
            )
        name = f"depth_{sample.sample_index:08d}.npz"
        _write_depth(
            run / "observations" / name,
            sample.payload,
            **header,
            sensor_id=sample.sensor_id,
            source=sample.source,
            frame_id=sample.frame_id,
            sample_index=sample.sample_index,
            capture_time_ns=sample.capture_time_ns,
            available_time_ns=sample.available_time_ns,
        )
        sample_indices.append(sample.sample_index)
    clean_indices = []
    for clean in engine.get_clean_depth():
        _write_depth(
            run / "ground_truth" / f"depth_{clean.sample_index:08d}.npz",
            clean.payload,
            **header,
            frame_id=clean.frame_id,
            sample_index=clean.sample_index,
            capture_time_ns=clean.capture_time_ns,
        )
        clean_indices.append(clean.sample_index)
    tick = stamp.timestamp.tick
    boundary = {
        **header,
        "tick": tick,
        "time_ns": stamp.timestamp.time_ns,
        "record": "reset" if step is None else "step",
    }
    observations.write(
        dumps(
            {
                **boundary,
                "sensors": [
                    {
                        "sensor_id": slot.sensor_id,
                        "status": slot.status,
                        "reason": slot.reason,
                        "sample_index": slot.sample.sample_index
                        if slot.sample
                        else None,
                    }
                    for slot in observation.sensors
                ],
                "new_depth_indices": sample_indices,
            }
        )
        + "\n"
    )
    truth = engine.get_ground_truth()
    privileged = {
        **boundary,
        "robot": truth.robot.to_dict(),
        "humans": [human.to_dict() for human in truth.humans],
        "clean_depth_indices": clean_indices,
    }
    if step is not None:
        interval = {
            **boundary,
            "step_index": tick - 1,
            "start_time_ns": step.requested.stamp.timestamp.time_ns,
            "end_time_ns": stamp.timestamp.time_ns,
        }
        actions.write(
            dumps(
                {
                    **interval,
                    "requested": asdict(step.requested),
                    "limited": asdict(step.limited),
                }
            )
            + "\n"
        )
        privileged.update(
            **interval,
            executed=asdict(step.executed),
            contacts=[asdict(contact) for contact in step.contacts],
            terminated=step.terminated,
            truncated=step.truncated,
            end_reason=step.end_reason,
        )
    ground_truth.write(dumps(privileged) + "\n")


def record_depth_episode(
    config: DictConfig,
    output_root: Path,
    command: Twist2D = _STOP,
    *,
    report: bool = False,
) -> Path:
    """Stream one episode to staging and publish all channels together."""
    if config.get("sensors") is None:
        raise ValueError("record_depth requires sensors=d435i")
    output_root.mkdir(parents=True, exist_ok=True)
    engine = MuJoCoEngine()
    try:
        engine.load(config)
        # The parent run appears only after every channel and manifest is complete.
        with TemporaryDirectory(prefix=".depth-", dir=output_root) as staging:
            run = Path(staging)
            (run / "observations").mkdir()
            (run / "ground_truth").mkdir()
            count = 0
            runtime_failure = None
            with (
                (run / "observations.jsonl").open("w") as observations,
                (run / "ground_truth.jsonl").open("w") as ground_truth,
                (run / "actions.jsonl").open("w") as actions,
            ):
                # Catch only runtime failures here. An I/O failure leaves staging unpublished.
                try:
                    observation = engine.reset()
                except Exception as exc:
                    if engine.get_metadata().status != "failed":
                        raise
                    runtime_failure = exc
                if runtime_failure is None:
                    sample = next(
                        slot.sample
                        for slot in observation.sensors
                        if slot.sensor_id == DEPTH_SENSOR_ID
                    )
                    if sample is None:
                        raise ValueError("reset did not publish a depth sample")
                    _write_boundary(
                        run,
                        observations,
                        ground_truth,
                        actions,
                        engine,
                        observation,
                        (sample,),
                    )
                    count += 1
                    while engine.get_metadata().status == "running":
                        robot = engine.get_ground_truth().robot
                        try:
                            result = engine.step(
                                RobotAction(robot.entity_id, command, robot.timestamp)
                            )
                        except Exception as exc:
                            if engine.get_metadata().status != "failed":
                                raise
                            runtime_failure = exc
                            break
                        _write_boundary(
                            run,
                            observations,
                            ground_truth,
                            actions,
                            engine,
                            result.observation,
                            result.sensor_samples,
                            result,
                        )
                        count += len(result.sensor_samples)

            metadata = engine.get_metadata()
            episode = metadata.episode
            manifest = asdict(metadata)
            manifest.pop("resolved_config")
            manifest.update(
                schema_version=episode.config.schema_version,
                episode_id=episode.episode_id,
                episode_ordinal=episode.episode_ordinal,
                run_id=episode.run_id,
                episode_seed=episode.config.episode_seed,
                scenario_id=episode.config.scenario_id,
                resolved_config_path="resolved.yaml",
                storage_format="narrow_nav_depth_npz_v1",
                step_count=metadata.last_valid_tick or 0,
                depth_frame_count=count,
                policy={"id": "constant_twist", "command": asdict(command)},
                platform=platform(),
                numpy_version=np.__version__,
                mujoco_gl=environ.get("MUJOCO_GL", "platform_default"),
            )
            (run / "resolved.yaml").write_bytes(metadata.resolved_config.encode())
            (run / "manifest.json").write_text(dumps(manifest, indent=2) + "\n")
            if report and count:
                write_depth_report(
                    run, count, metadata.depth_stream.profile.noise.max_depth_m
                )
            destination = output_root / episode.run_id
            run.rename(destination)
        if runtime_failure is not None:
            raise RuntimeError(
                f"capture failed; valid prefix recorded at {destination}"
            ) from runtime_failure
        return destination
    finally:
        engine.close()


def main() -> None:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--config-dir", type=Path, default=Path("configs"))
    parser.add_argument("--output-root", type=Path, default=Path("runs"))
    parser.add_argument(
        "--report", action="store_true", help="write privileged diagnostic SVG"
    )
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--wz", type=float, default=0.0)
    parser.add_argument("overrides", nargs="*")
    args = parser.parse_args()
    config = compose_config(args.config_dir, args.overrides)
    if config.get("sensors") is None:
        parser.error("set sensors=d435i robots=mecanum_robot")
    command = Twist2D(args.vx, args.vy, args.wz)
    print(record_depth_episode(config, args.output_root, command, report=args.report))


if __name__ == "__main__":
    main()
