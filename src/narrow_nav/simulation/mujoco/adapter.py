"""Native MuJoCo model, state extraction and contact sampling."""

from math import atan2, cos, sin
from random import Random

import mujoco
import numpy as np

from narrow_nav.core.actions import MotionBatch
from narrow_nav.core.geometry import wrap_yaw
from narrow_nav.core.metadata import EpisodeMetadata, SnapshotMetadata
from narrow_nav.core.protocols import (
    AdapterSnapshot,
    AdapterStepResult,
    ContactEvent,
    SensorSeed,
)
from narrow_nav.core.states import GroundTruthState, HumanState, RobotState
from narrow_nav.core.types import (
    NS_PER_SECOND,
    ROBOT_BASE_FRAME,
    WORLD_FRAME,
    Pose2D,
    Twist2D,
    Velocity2D,
)

from ..backend import SimulationError
from ..collision_monitor import contact_events
from ..depth_config import DEPTH_FRAME_ID, DEPTH_SENSOR_ID
from ..entity_registry import EntityRegistry
from ..robot_controller import DriveProfile
from ..world_builder import BuiltWorld
from ..world_geometry import WorldCollision
from .depth_camera import D435iCapture
from .geometry import collision_geometry
from .human_visuals import HumanVisuals

_BAD_WARNINGS = (
    mujoco.mjtWarning.mjWARN_BADQPOS,
    mujoco.mjtWarning.mjWARN_BADQVEL,
    mujoco.mjtWarning.mjWARN_BADQACC,
    mujoco.mjtWarning.mjWARN_BADCTRL,
)


class MuJoCoAdapter:
    """All native objects and indices are private to this adapter."""

    def __init__(self) -> None:
        self._model: mujoco.MjModel | None = None
        self._data: mujoco.MjData | None = None
        self._metadata: EpisodeMetadata | None = None
        self._rng: Random | None = None
        self._visuals: HumanVisuals | None = None
        self._depth: D435iCapture | None = None
        self._depth_context: mujoco.MjrContext | None = None

    @property
    def version(self) -> str:
        return mujoco.mj_versionString()

    def load(self, world: BuiltWorld, drive: DriveProfile) -> WorldCollision:
        model = mujoco.MjSpec.from_string(
            world.mjcf, assets=dict(world.visual_assets)
        ).compile()
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        visuals = HumanVisuals(model, world.skin_actors)
        scene = world.scene
        walls = tuple(
            model.geom(index).name
            for index in range(model.ngeom)
            if model.geom_bodyid[index] == 0 and model.geom_group[index] in (1, 3)
        )
        registry = EntityRegistry(
            scene.robot.entity_id,
            tuple(human.entity_id for human in scene.humans),
            walls,
        )
        body_ids = {
            model.body(name).id: name for name in (registry.robot, *registry.humans)
        }
        geom_entities = []
        for index, body_id in enumerate(model.geom_bodyid):
            entity = None
            if body_id == 0:
                name = model.geom(index).name
                entity = name if name in walls else None
            else:
                while body_id != 0 and body_id not in body_ids:
                    body_id = model.body_parentid[body_id]
                entity = body_ids.get(body_id)
            geom_entities.append(entity)
        actuator_ids = tuple(
            model.actuator(f"{registry.robot}/{name}_drive").id for name in drive.wheels
        )
        collision = collision_geometry(model, data)

        if self._depth is not None:
            self._depth.close()
            self._depth = None
        self._model, self._data = model, data
        self._drive, self._registry = drive, registry
        self._robot_body = model.body(registry.robot).id
        self._humans = tuple(
            (human, model.body(human.entity_id).id) for human in scene.humans
        )
        self._geom_entities = tuple(geom_entities)
        self._actuator_ids = list(actuator_ids)
        self._visuals = visuals
        self._metadata = self._rng = None
        return collision

    def _native(self) -> tuple[mujoco.MjModel, mujoco.MjData]:
        if self._model is None or self._data is None:
            raise SimulationError("MuJoCo model is not loaded")
        return self._model, self._data

    def reset(
        self,
        scene: BuiltWorld,
        metadata: EpisodeMetadata,
        engine_seed: int,
        sensor_seeds: tuple[SensorSeed, ...],
    ) -> AdapterSnapshot:
        model, data = self._native()
        mujoco.mj_resetData(model, data)
        self._rng = Random(engine_seed)
        mujoco.mj_forward(model, data)
        self._visuals.update(model, data, 0.0)
        self._check_state(data, 0)
        self._metadata = metadata
        truth = self._extract(metadata.stamp(0))
        if scene.depth_camera is None:
            return AdapterSnapshot(truth)
        if self._depth is None:
            self._depth = D435iCapture(
                model,
                f"{scene.scene.robot.entity_id}/depth_camera",
                scene.depth_camera,
                context=self._depth_context,
            )
        seed = next(
            stream.seed
            for stream in sensor_seeds
            if stream.sensor_id == DEPTH_SENSOR_ID
        )
        self._depth.noise.reset(seed)
        sample, clean = self._depth.capture(model, data, 0)
        return AdapterSnapshot(
            truth, (sample,), (WORLD_FRAME, ROBOT_BASE_FRAME, DEPTH_FRAME_ID), (clean,)
        )

    @staticmethod
    def _check_state(data: mujoco.MjData, time_ns: int) -> None:
        time_s = time_ns / NS_PER_SECOND
        if (
            not np.isfinite(data.time)
            or abs(data.time - time_s) > max(1e-9, time_s * 1e-10)
            or any(data.warning[warning].number for warning in _BAD_WARNINGS)
            or not all(
                np.isfinite(values).all()
                for values in (data.qpos, data.qvel, data.qacc)
            )
        ):
            raise SimulationError(f"invalid MuJoCo state or clock at {time_ns} ns")

    @staticmethod
    def _yaw(data: mujoco.MjData, body_id: int) -> float:
        orientation = data.xmat[body_id]
        return wrap_yaw(atan2(float(orientation[3]), float(orientation[0])))

    def _extract(self, stamp: SnapshotMetadata) -> GroundTruthState:
        model, data = self._native()
        velocity = np.empty(6, dtype=np.float64)

        def read(body_id: int) -> tuple[Pose2D, float, float, float]:
            x, y = map(float, data.xpos[body_id, :2])
            mujoco.mj_objectVelocity(
                model, data, mujoco.mjtObj.mjOBJ_XBODY, body_id, velocity, 0
            )
            return (
                Pose2D(x, y, self._yaw(data, body_id)),
                float(velocity[3]),
                float(velocity[4]),
                float(velocity[2]),
            )

        pose, vx, vy, wz = read(self._robot_body)
        c, s = cos(pose.yaw), sin(pose.yaw)
        robot = RobotState(
            self._registry.robot,
            pose,
            Twist2D(c * vx + s * vy, -s * vx + c * vy, wz),
            stamp.timestamp,
        )
        humans = []
        for spawn, body_id in self._humans:
            human_pose, x, y, _ = read(body_id)
            humans.append(
                HumanState(
                    spawn.entity_id,
                    human_pose,
                    Velocity2D(x, y),
                    spawn.goal,
                    stamp.timestamp,
                )
            )
        return GroundTruthState(stamp, robot, tuple(humans))

    def _contacts(self, data: mujoco.MjData, time_ns: int) -> tuple[ContactEvent, ...]:
        pairs: set[tuple[str, str]] = set()
        for contact in data.contact:
            if (
                contact.efc_address < 0
                or contact.dist > 0
                or contact.geom1 < 0
                or contact.geom2 < 0
            ):
                continue
            a, b = (
                self._geom_entities[contact.geom1],
                self._geom_entities[contact.geom2],
            )
            if a is not None and b is not None and a != b:
                first, second = sorted((a, b))
                pairs.add((first, second))
        return contact_events(pairs, self._registry, time_ns)

    def advance(self, limited: MotionBatch) -> AdapterStepResult:
        model, data = self._native()
        if self._metadata is None:
            raise SimulationError("reset must precede advance")
        episode = self._metadata.config
        start = limited.stamp.timestamp
        end = self._metadata.stamp(start.tick + 1)
        self._check_state(data, start.time_ns)
        data.ctrl[self._actuator_ids] = self._drive.wheel_speeds(limited.robot.velocity)
        previous_yaw = self._yaw(data, self._robot_body)
        delta_yaw = 0.0
        contacts = []
        samples, clean_frames = [], []
        depth = self._depth
        for offset in range(
            episode.physics_dt_ns, episode.control_dt_ns + 1, episode.physics_dt_ns
        ):
            time_ns = start.time_ns + offset
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            self._check_state(data, time_ns)
            yaw = self._yaw(data, self._robot_body)
            delta_yaw += wrap_yaw(yaw - previous_yaw)
            previous_yaw = yaw
            contacts.extend(self._contacts(data, time_ns))
            if depth is not None and time_ns % depth.profile.period_ns == 0:
                self._visuals.update(model, data, time_ns / NS_PER_SECOND)
                sample, clean = depth.capture(model, data, time_ns)
                samples.append(sample)
                clean_frames.append(clean)
        if not samples or samples[-1].capture_time_ns != end.timestamp.time_ns:
            self._visuals.update(model, data, end.timestamp.time_s)
        truth = self._extract(end)
        snapshot = (
            AdapterSnapshot(
                truth,
                tuple(samples),
                (WORLD_FRAME, ROBOT_BASE_FRAME, DEPTH_FRAME_ID),
                tuple(clean_frames),
            )
            if depth is not None
            else AdapterSnapshot(truth)
        )
        return AdapterStepResult(snapshot, delta_yaw, tuple(contacts))

    def close(self) -> None:
        if self._depth is not None:
            self._depth.close()
            self._depth = None
        self._data = self._model = self._metadata = self._rng = None
        self._visuals = None
