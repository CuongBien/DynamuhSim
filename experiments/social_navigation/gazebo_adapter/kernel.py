"""External scenario adapter around unmodified SocNavGym group behavior methods.
No robot motion, reward, RL observations, automatic episode reset, or formation logic.
"""
from pathlib import Path
import math
import random
import gymnasium as gym
import numpy as np
import socnavgym
from socnavgym.envs.utils.wall import Wall

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent

class GroupKernel:
    def __init__(self, config, layout, mode, seed):
        random.seed(seed)
        np.random.seed(seed)
        self.wrapper = gym.make('SocNavGym-v2', config=str(config))
        self.env = self.wrapper.unwrapped
        obs, _ = self.wrapper.reset(seed=seed)
        self.initial_observation_in_space = self.wrapper.observation_space.contains(obs)
        self.mode = mode
        self.steps = 0
        e = self.env
        self.group = (e.moving_interactions if mode == 'walking' else e.static_interactions)[0]
        g = self.group
        g.x, g.y = -24., 0.
        g.arrange_humans()
        for number, h in enumerate(g.humans, 1):
            h.id = number
            h.position_history.clear()
            if mode == 'walking':
                h.orientation = h.initial_orientation = 0.
            h.speed = 0.
        g.set_goal(-15., 0.)
        self.target = 0
        self.route = [(-15., 0.), (-24., 0.)]
        e.walls = []
        for i, box in enumerate(layout['obstacles'], 100):
            sx, sy = box['sx'], box['sy']
            e.walls.append(Wall(id=i, x=box['x'], y=box['y'],
                orientation=0. if sx >= sy else math.pi/2,
                length=max(sx, sy), thickness=min(sx, sy)))
        e.objects = e.walls + [g, e.robot]
        e.robot.goal_x, e.robot.goal_y = 26., 0.
        self.actual_velocity={int(h.id):(0.,0.) for h in g.humans}
        self.initial = self.humans()

    def humans(self):
        return [dict(id=int(h.id), x=float(h.x), y=float(h.y), yaw=float(h.orientation),
                     speed=float(h.speed), radius=self.env.HUMAN_DIAMETER/2,group_id=1,
                     vx=self.actual_velocity[int(h.id)][0],vy=self.actual_velocity[int(h.id)][1])
                for h in self.group.humans]

    def advance(self, robot):
        e = self.env
        r = e.robot
        r.x, r.y, r.orientation = robot['x'], robot['y'], robot['yaw']
        c, s = math.cos(r.orientation), math.sin(r.orientation)
        r.vel_x = c*robot['vx'] + s*robot['vy']
        r.vel_y = -s*robot['vx'] + c*robot['vy']
        r.vel_a = robot['omega']
        # The native group ORCA function omits the robot. The native contact handler
        # still sees it. Preserve this limitation, rather than inventing a new policy.
        if self.mode == 'walking':
            velocities = e.compute_orca_interaction_velocities()
            self.group.update_speed(e.TIMESTEP, velocities[0], e.MAX_ROTATION_HUMAN)
            if self.group.has_reached_goal():
                self.target = 1-self.target
                self.group.set_goal(*self.route[self.target])
        before={int(h.id):(float(h.x),float(h.y)) for h in self.group.humans}
        e.handle_collision_and_update()
        self.actual_velocity={int(h.id):((float(h.x)-before[int(h.id)][0])/e.TIMESTEP,(float(h.y)-before[int(h.id)][1])/e.TIMESTEP) for h in self.group.humans}
        self.steps += 1

    def close(self):
        self.wrapper.close()
