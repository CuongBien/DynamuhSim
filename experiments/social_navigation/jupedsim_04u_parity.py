"""Emit a native trace for cross-build parity without coupling inputs."""

import json

import jupedsim
from shapely.geometry import box

MODELS = {
    "sfm_jupedsim": ("SocialForceModel", {}),
    "avm_jupedsim": ("AnticipationVelocityModel", {"rng_seed": 11}),
    "csm_jupedsim": ("CollisionFreeSpeedModelV2", {}),
}


def simulation(backend, geometry):
    model, options = MODELS[backend]
    return jupedsim.Simulation(
        model=getattr(jupedsim, model)(**options), geometry=geometry, dt=0.01
    )


def add_agent(sim, backend, start, goal):
    model, _ = MODELS[backend]
    stage = sim.add_waypoint_stage(goal, 0.1)
    journey = sim.add_journey(jupedsim.JourneyDescription([stage]))
    return sim.add_agent(
        getattr(jupedsim, f"{model}AgentParameters")(
            position=start, journey_id=journey, stage_id=stage, radius=0.25
        )
    )


def trace(backend):
    sim = simulation(backend, box(-5, -2, 5, 2))
    agent_id = add_agent(sim, backend, (-3, 0.1), (3, 0.1))
    result = []
    for _ in range(300):
        sim.iterate()
        agent = sim.agent(agent_id)
        result.append(
            (sim.iteration_count(), agent.position, agent.orientation, agent.stage_id)
        )
    return result


if __name__ == "__main__":
    print(json.dumps({key: trace(key) for key in MODELS}, allow_nan=False))
