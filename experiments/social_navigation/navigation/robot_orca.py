"""Opt-in robot-aware group ORCA for the benchmark's single walking group.
This is an adapter policy, not upstream SocNavGym behavior. It retains native
ORCA group setup and adds a robot agent with measured world-frame velocity.
The robot solution is discarded: ROS/Nav2 remains the only robot controller.
"""
import math
from types import MethodType
import numpy as np
import rvo2

class RobotAwareGroups:
    def __init__(self,kernel):
        self.kernel=kernel
        self.original=kernel.env.compute_orca_interaction_velocities
        self.last=None
        kernel.env.compute_orca_interaction_velocities=MethodType(self.compute,kernel.env)

    def compute(self,env):
        # Explicit scope guard instead of silently dropping other populations.
        if env.static_humans or env.dynamic_humans or env.static_interactions or env.h_l_interactions or len(env.moving_interactions)!=1:
            raise ValueError('robot_orca supports exactly one moving group, no other humans/interactions')
        sim=rvo2.PyRVOSimulator(env.TIMESTEP,env.orca_neighborDist,env.total_humans,
            env.orca_timeHorizon,env.orca_timeHorizonObst,env.HUMAN_DIAMETER/2,env.orca_maxSpeed)
        for obj in env.tables+env.chairs+env.laptops+env.plants+env.walls:
            sim.addObstacle(env.get_obstacle_corners(obj))
        indices=[]
        for group in env.moving_interactions:
            i=sim.addAgent((group.x,group.y))
            sim.setAgentRadius(i,env.INTERACTION_RADIUS+env.HUMAN_DIAMETER)
            sim.setAgentNeighborDist(i,2*(env.INTERACTION_RADIUS+env.HUMAN_DIAMETER))
            preferred=np.array([group.goal_x-group.x,group.goal_y-group.y],dtype=np.float32)
            if np.linalg.norm(preferred)!=0:preferred/=np.linalg.norm(preferred)
            preferred*=env.MAX_ADVANCE_HUMAN
            sim.setAgentPrefVelocity(i,(preferred[0],preferred[1]));indices.append(i)
        robot=env.robot;c,s=math.cos(robot.orientation),math.sin(robot.orientation)
        velocity=(c*robot.vel_x-s*robot.vel_y,s*robot.vel_x+c*robot.vel_y)
        rid=sim.addAgent((robot.x,robot.y))
        sim.setAgentRadius(rid,robot.radius)
        sim.setAgentMaxSpeed(rid,max(env.MAX_ADVANCE_ROBOT,math.hypot(*velocity)))
        sim.setAgentVelocity(rid,velocity)
        sim.setAgentPrefVelocity(rid,velocity)
        sim.processObstacles();sim.doStep()
        velocities=[sim.getAgentVelocity(i) for i in indices]
        self.last=dict(robot_xy=[robot.x,robot.y],measured_robot_world_velocity=list(velocity),
            solved_robot_velocity_ignored=list(sim.getAgentVelocity(rid)),
            group_velocity=[list(v) for v in velocities],
            group_robot_neighbor_seen=any(sim.getAgentAgentNeighbor(indices[0],j)==rid for j in range(sim.getAgentNumAgentNeighbors(indices[0]))))
        return velocities

    def restore(self):
        self.kernel.env.compute_orca_interaction_velocities=self.original


def behavior_description(kernel,enabled):
    e=kernel.env
    return dict(policy='robot_orca' if enabled else 'native',modified_group_solver=enabled,
        robot_in_group_orca=enabled,applies_to='walking group only' if enabled else 'native modes',
        robot_state_source='measured Gazebo pose and finite-difference world velocity',
        robot_radius=e.robot.radius,group_orca_radius=e.INTERACTION_RADIUS+e.HUMAN_DIAMETER,
        group_neighbor_distance=2*(e.INTERACTION_RADIUS+e.HUMAN_DIAMETER),
        time_horizon=e.orca_timeHorizon,time_horizon_obstacles=e.orca_timeHorizonObst,
        group_current_velocity='native zero initialization per solver call',
        group_smoothing='native 0.9 previous + 0.1 new',
        responsibility='RVO2 reciprocal; solved robot velocity is discarded, actual robot controlled by Nav2',
        new_swept_guard=False,new_human_pair_separation=False,stationary_group_reaction_changed=False)
