"""Native MuJoCo model, state extraction and contact sampling."""

from math import atan2, cos, hypot, sin
from random import Random

import mujoco
import numpy as np

from narrow_nav.core.actions import MotionBatch
from narrow_nav.core.errors import ConfigurationError, ContractError
from narrow_nav.core.geometry import wrap_yaw
from narrow_nav.core.metadata import EpisodeMetadata, SnapshotMetadata
from narrow_nav.core.protocols import (
    AdapterSnapshot,
    AdapterStepResult,
    SensorSeed,
)
from narrow_nav.core.states import GroundTruthState, HumanState, RobotState
from narrow_nav.core.types import NS_PER_SECOND, Pose2D, Twist2D, Velocity2D

from ..backend import SimulationError
from ..collision_monitor import contact_events
from ..entity_registry import EntityRegistry
from ..human_motion import heading_from_velocity
from ..robot_controller import DriveProfile
from ..world_builder import BuiltWorld
from ..world_geometry import GEOMETRY_TOLERANCE_M, WorldCollision
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

    not_observable_solver = ("human_wall", "human_human")

    def __init__(self, *, human_motion_enabled: bool = False) -> None:
        self._human_motion_enabled = human_motion_enabled
        self._model: mujoco.MjModel | None = None
        self._data: mujoco.MjData | None = None
        self._metadata: EpisodeMetadata | None = None
        self._rng: Random | None = None
        self._visuals: HumanVisuals | None = None

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
        humans = tuple(
            (human, model.body(human.entity_id).id) for human in scene.humans
        )
        human_body_ids = [body_id for _, body_id in humans]
        human_mocap_ids = model.body_mocapid[human_body_ids].tolist()
        if any(mocap_id < 0 for mocap_id in human_mocap_ids):
            raise ConfigurationError("human roots must be mocap bodies")

        self._model, self._data = model, data
        self._drive, self._registry = drive, registry
        self._robot_body = model.body(registry.robot).id
        self._humans = humans
        self._human_body_ids = human_body_ids
        self._human_mocap_ids = human_mocap_ids
        self._human_radii = tuple(spawn.radius_m for spawn, _ in humans)
        self._collision = collision
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
        self._metadata = None
        mujoco.mj_resetData(model, data)
        self._rng = Random(engine_seed)
        mujoco.mj_forward(model, data)
        self._expected_human_pos = model.body_pos[self._human_body_ids].copy()
        self._expected_human_quat = model.body_quat[self._human_body_ids].copy()
        self._human_velocities = np.zeros((len(self._humans), 2))
        if self._human_motion_enabled:
            self._visuals.reset_animation()
            self._visuals.update(
                model,
                data,
                0.0,
                speeds_mps={spawn.entity_id: 0.0 for spawn, _ in self._humans},
            )
            if any(
                not self._collision.contains(x, y, radius)
                for (x, y, _), radius in zip(
                    self._expected_human_pos, self._human_radii, strict=True
                )
            ):
                raise SimulationError(
                    "initial human footprint lies outside compiled free space"
                )
        else:
            self._visuals.update(model, data, 0.0)
        self._check_state(data, 0)
        self._check_human_pose(data)
        self._metadata = metadata
        return AdapterSnapshot(self._extract(metadata.stamp(0)))

    @staticmethod
    def _check_state(data: mujoco.MjData, time_ns: int) -> None:
        time_s = time_ns / NS_PER_SECOND
        if (
            not np.isfinite(data.time)
            or abs(data.time - time_s) > max(1e-9, time_s * 1e-10)
            or any(data.warning[warning].number for warning in _BAD_WARNINGS)
            or not all(
                np.isfinite(values).all()
                for values in (
                    data.qpos,
                    data.qvel,
                    data.qacc,
                    data.mocap_pos,
                    data.mocap_quat,
                )
            )
        ):
            raise SimulationError(f"invalid MuJoCo state or clock at {time_ns} ns")

    def _check_human_pose(self, data: mujoco.MjData) -> None:
        # Check only physical roots. Quaternius bones animate separately.
        if not (
            np.array_equal(
                data.mocap_pos[self._human_mocap_ids], self._expected_human_pos
            )
            and np.array_equal(
                data.mocap_quat[self._human_mocap_ids], self._expected_human_quat
            )
            and np.allclose(
                data.xpos[self._human_body_ids],
                self._expected_human_pos,
                atol=GEOMETRY_TOLERANCE_M,
                rtol=0,
            )
        ):
            mode = "kinematic" if self._human_motion_enabled else "stationary"
            raise SimulationError(
                f"{mode} human mocap pose changed outside its commanded motion"
            )

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
        for (spawn, body_id), (vx, vy) in zip(
            self._humans, self._human_velocities, strict=True
        ):
            x, y = map(float, data.xpos[body_id, :2])
            humans.append(
                HumanState(
                    spawn.entity_id,
                    Pose2D(x, y, self._yaw(data, body_id)),
                    Velocity2D(float(vx), float(vy)),
                    spawn.goal,
                    stamp.timestamp,
                )
            )
        return GroundTruthState(stamp, robot, tuple(humans))

    def _contact_pairs(self, data: mujoco.MjData) -> set[tuple[str, str]]:
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
        return pairs

    def advance(self, limited: MotionBatch) -> AdapterStepResult:
        model, data = self._native()
        if self._metadata is None:
            raise SimulationError("reset must precede advance")
        episode = self._metadata.config
        start = limited.stamp.timestamp
        if limited.stamp != self._metadata.stamp(start.tick):
            raise ContractError("adapter commands must match the current episode stamp")
        if limited.entity_ids != (self._registry.robot, *self._registry.humans):
            raise ContractError("adapter commands must match the configured actor IDs")
        if not self._human_motion_enabled and any(
            motion.velocity.linear_x != 0 or motion.velocity.linear_y != 0
            for motion in limited.humans
        ):
            raise ContractError("stationary adapter requires zero human velocity")
        end = self._metadata.stamp(start.tick + 1)
        self._check_state(data, start.time_ns)
        self._check_human_pose(data)
        data.ctrl[self._actuator_ids] = self._drive.wheel_speeds(limited.robot.velocity)
        physics_dt_s = episode.physics_dt_ns / NS_PER_SECOND
        deltas = (
            np.asarray(
                [
                    (motion.velocity.linear_x, motion.velocity.linear_y)
                    for motion in limited.humans
                ],
                dtype=float,
            ).reshape(-1, 2)
            * physics_dt_s
        )
        moving = bool(np.any(deltas))
        target_quat = self._expected_human_quat.copy()
        if moving:
            for index, ((_, body_id), motion) in enumerate(
                zip(self._humans, limited.humans, strict=True)
            ):
                if np.any(deltas[index]):
                    half = (
                        heading_from_velocity(motion.velocity, self._yaw(data, body_id))
                        / 2
                    )
                    target_quat[index] = (cos(half), 0, 0, sin(half))
        initial_xy = data.xpos[self._human_body_ids, :2].copy()
        before_xy = initial_xy
        previous_yaw = self._yaw(data, self._robot_body)
        delta_yaw = 0.0
        contacts = []
        blocked = 0
        for offset in range(
            episode.physics_dt_ns, episode.control_dt_ns + 1, episode.physics_dt_ns
        ):
            time_ns = start.time_ns + offset
            if moving:
                positions = self._expected_human_pos.copy()
                positions[:, :2] += deltas
                if self._collision.human_paths_clear(
                    tuple(map(tuple, before_xy)),
                    tuple(map(tuple, positions[:, :2])),
                    self._human_radii,
                ):
                    data.mocap_pos[self._human_mocap_ids] = positions
                    data.mocap_quat[self._human_mocap_ids] = target_quat
                    self._expected_human_pos = positions
                    self._expected_human_quat = target_quat
                else:
                    blocked += 1
            mujoco.mj_step(model, data)
            pairs = self._contact_pairs(data)
            mujoco.mj_forward(model, data)
            self._check_state(data, time_ns)
            if self._human_motion_enabled:
                self._check_human_pose(data)
                after_xy = data.xpos[self._human_body_ids, :2]
                self._human_velocities = (after_xy - before_xy) / physics_dt_s
                before_xy = after_xy
            yaw = self._yaw(data, self._robot_body)
            delta_yaw += wrap_yaw(yaw - previous_yaw)
            previous_yaw = yaw
            contacts.extend(
                contact_events(
                    pairs | self._contact_pairs(data), self._registry, time_ns
                )
            )
        if self._human_motion_enabled:
            dt_s = episode.control_dt_ns / NS_PER_SECOND
            speeds = {
                spawn.entity_id: hypot(dx, dy) / dt_s
                for (spawn, _), (dx, dy) in zip(
                    self._humans, before_xy - initial_xy, strict=True
                )
            }
            self._visuals.update(
                model, data, end.timestamp.time_s, speeds_mps=speeds, dt_s=dt_s
            )
        else:
            self._visuals.update(model, data, end.timestamp.time_s)
        self._check_human_pose(data)
        return AdapterStepResult(
            AdapterSnapshot(self._extract(end)), delta_yaw, tuple(contacts), blocked
        )

    def close(self) -> None:
        self._data = self._model = self._metadata = self._rng = None
        self._visuals = None
