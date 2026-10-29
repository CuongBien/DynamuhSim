"""Opt-in SocNavGym group ORCA correction; individual safety projection is unchanged.

The vendored group solver restarts RVO each step with zero velocity and uses
interaction radius + full human diameter. Use the actual member envelope and
measured mean velocity instead. Individual agents, obstacles, native goals,
turn limits, lifecycle and RNG remain owned by the existing backend.
"""
import json
import math
from pathlib import Path

PROFILE = {"schema": "dynamuhsim_human_motion/1", "profile": "group_orca_v1"}


def validate_profile(value):
    if value != PROFILE:
        raise ValueError("Unsupported human motion profile")
    return dict(PROFILE)


def group_state(group, velocities):
    if not group.humans:
        raise ValueError("Moving group has no members")
    radius = max(math.hypot(h.x - group.x, h.y - group.y) + h.width / 2 for h in group.humans)
    velocity = tuple(sum(velocities.get(int(h.id), (0., 0.))[axis] for h in group.humans) / len(group.humans) for axis in (0, 1))
    if radius <= 0 or not all(math.isfinite(x) for x in (radius, *velocity)):
        raise ValueError("Nonfinite or nonpositive group state")
    return radius, velocity


class GroupORCA:
    def __init__(self, kernel, simulator=None):
        self.kernel = kernel
        if simulator is None:
            import rvo2
            simulator = rvo2.PyRVOSimulator
        self.simulator = simulator
        self.last = []

    def velocities(self):
        # Keep the native individual-agent delayed prediction and obstacle branches.
        import numpy as np
        e = self.kernel.env
        sim = self.simulator(e.TIMESTEP, e.orca_neighborDist, e.total_humans,
                             e.orca_timeHorizon, e.orca_timeHorizonObst,
                             e.HUMAN_DIAMETER / 2, e.orca_maxSpeed)
        def preferred(agent, x, y):
            v = np.array([x, y], dtype=np.float32)
            norm = np.linalg.norm(v)
            if norm != 0:
                v /= norm
            v *= e.MAX_ADVANCE_HUMAN
            sim.setAgentPrefVelocity(agent, tuple(v))
        for human in e.static_humans + e.dynamic_humans:
            x, y = human.x, human.y
            if len(human.position_history) >= 3:
                x, y, theta, _ = human.position_history[-3]
                x += human.speed * np.cos(theta) * .05
                y += human.speed * np.sin(theta) * .05
                x += human.speed * np.cos(theta) * .05
                y += human.speed * np.sin(theta) * .05
            agent = sim.addAgent((x, y))
            preferred(agent, human.goal_x - human.x, human.goal_y - human.y)
        for obj in e.tables + e.chairs + e.laptops + e.plants + e.walls:
            sim.addObstacle(e.get_obstacle_corners(obj))
        for group in e.static_interactions + e.h_l_interactions:
            if group.name == "human-laptop-interaction":
                sim.addObstacle(e.get_obstacle_corners(group.human))
            elif group.name == "human-human-interaction" and group.type == "stationary":
                sim.addObstacle(e.get_obstacle_corners(group))
        agents, diagnostics = [], []
        for group in e.moving_interactions:
            agent = sim.addAgent((group.x, group.y))
            radius, velocity = group_state(group, self.kernel.motion.velocities)
            sim.setAgentRadius(agent, radius)
            sim.setAgentNeighborDist(agent, 2 * radius)
            sim.setAgentVelocity(agent, velocity)
            preferred(agent, group.goal_x - group.x, group.goal_y - group.y)
            agents.append(agent)
            diagnostics.append(dict(members=[int(h.id) for h in group.humans], radius_m=radius, measured_velocity=list(velocity)))
        sim.processObstacles()
        sim.doStep()
        result = [sim.getAgentVelocity(agent) for agent in agents]
        for row, velocity in zip(diagnostics, result):
            row["orca_velocity"] = list(velocity)
        self.last = diagnostics
        return result


def enable_profile(kernel, value):
    validate_profile(value)
    adapter = GroupORCA(kernel)
    kernel.env.compute_orca_interaction_velocities = adapter.velocities
    return adapter


def load_profile(kernel, runtime):
    path = Path(runtime) / "human_motion.json"
    if not path.exists():
        return None
    return enable_profile(kernel, json.loads(path.read_text()))
