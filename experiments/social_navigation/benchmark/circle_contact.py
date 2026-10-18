"""Opt-in geometry ablation around the native kernel; vendor classes remain untouched.
Only instance collision predicates and the post-reset nominal robot radius change.
No ORCA robot agent, swept guard, pose correction, interpolation, or new steering.
"""
import math
from types import MethodType


def circle_intersects(human,other):
    radius=human.width/2
    dx,dy=human.x-other.x,human.y-other.y
    if other.name in ('robot','plant','human'):
        other_radius=other.width/2 if other.name=='human' else other.radius
        return math.hypot(dx,dy)<=radius+other_radius
    if other.name in ('wall','table','laptop','chair'):
        c,s=math.cos(other.orientation),math.sin(other.orientation)
        x,y=c*dx+s*dy,-s*dx+c*dy
        width=other.thickness if other.name=='wall' else other.width
        return math.hypot(max(abs(x)-other.length/2,0),max(abs(y)-width/2,0))<=radius
    if other.name in ('human-human-interaction','human-laptop-interaction'):
        return other.collides(human)
    raise ValueError('Unsupported contact geometry: '+other.name)


class CircleContact:
    def __init__(self,kernel,robot_radius):
        self.originals=[]
        self.contact_ids=set()
        kernel.env.robot.radius=float(robot_radius)
        kernel.env.ROBOT_RADIUS=float(robot_radius)
        for human in kernel.env.all_humans:
            # Bind only these experimental instances. The unmodified native
            # handle_collision_and_update still decides what contact does.
            self.originals.append((human,human.collides))
            def collides(h,obj,owner=self):
                result=circle_intersects(h,obj)
                if obj.name=='robot' and result:owner.contact_ids.add(int(h.id))
                return result
            human.collides=MethodType(collides,human)

    def begin_step(self):
        self.contact_ids.clear()

    def restore(self):
        for human,original in self.originals:human.collides=original


def geometry_description(kernel,policy,original_radius):
    return dict(policy=policy,modified_native_behavior=policy!='native',
        human_contact_shape='circle' if policy!='native' else 'axis_aligned_ellipse',
        human_contact_radius=kernel.env.HUMAN_DIAMETER/2 if policy!='native' else None,
        native_ellipse_semiaxes=None if policy!='native' else [kernel.env.HUMAN_DIAMETER/6,kernel.env.HUMAN_DIAMETER/2],
        human_proxy_radius=kernel.env.HUMAN_DIAMETER/2,
        original_reset_robot_radius=original_radius,effective_robot_radius=kernel.env.robot.radius,
        native_robot_radius_fixed_after_reset=policy!='native',
        robot_radius_is_nominal_proxy_metric_not_full_robot_collision_body=True,
        contact_timing='native pre-update; no swept collision test',
        contact_response='native stop the entire moving group if any member speed is zero',
        human_pair_separation_guard=False,robot_in_group_orca=False,
        factory_contacts='exact circle versus oriented box predicate' if policy!='native' else 'native ellipse predicate',
        native_wall_response_unchanged=True)
