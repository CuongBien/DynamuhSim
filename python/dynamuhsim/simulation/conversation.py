"""Explicit persistent-conversation fixture using native non-dispersing groups."""
import copy
import json
from pathlib import Path

PROFILE = dict(schema="dynamuhsim_human_lifecycle/1", profile="persistent_conversation_v1",
               groups=[[1,2,3],[4,5,6]], can_disperse=False,
               clock_validation="standing_snapshot_v1")


def validate_profile(value):
    if value != PROFILE or value.get("can_disperse") is not False:
        raise ValueError("Unsupported conversation lifecycle contract")
    return copy.deepcopy(PROFILE)


class PersistentConversation:
    def __init__(self, kernel):
        self.kernel = kernel
        env = kernel.env
        groups = list(env.static_interactions)
        if env.moving_interactions or env.static_humans or env.dynamic_humans:
            raise ValueError("Persistent conversation requires only stationary triplets")
        if sorted(sorted(int(h.id) for h in g.humans) for g in groups) != PROFILE["groups"]:
            raise ValueError("Persistent conversation requires designated initial triplets")
        self.groups = groups
        self.members = {id(g): tuple(g.humans) for g in groups}
        for group in groups:
            group.can_disperse = False
        self.snapshot()

    def snapshot(self):
        env = self.kernel.env
        if env.moving_interactions or env.static_humans or env.dynamic_humans or {id(g) for g in env.static_interactions} != {id(g) for g in self.groups}:
            raise RuntimeError("Conversation population/lifecycle changed")
        state = []
        for group in self.groups:
            if group.can_disperse is not False or (len(group.humans) != len(self.members[id(group)]) or any(a is not b for a,b in zip(group.humans,self.members[id(group)]))):
                raise RuntimeError("Conversation membership/dispersal flag changed")
            state.append(dict(members=sorted(int(h.id) for h in group.humans), can_disperse=False))
        return dict(profile=PROFILE["profile"], groups=sorted(state, key=lambda g:g["members"]))


def load_conversation(kernel, runtime):
    path = Path(runtime)/"human_lifecycle.json"
    if not path.exists():
        return None
    validate_profile(json.loads(path.read_text()))
    return PersistentConversation(kernel)


def verify_conversation(rows):
    expected = sorted(PROFILE["groups"])
    samples = 0
    for row in rows:
        state = row.get("human_lifecycle", {})
        if state.get("profile") != PROFILE["profile"]:
            return False, dict(reason="missing_lifecycle_state")
        groups = state.get("groups", [])
        if len(groups) != 2 or sorted(g["members"] for g in groups) != expected or any(g.get("can_disperse") is not False for g in groups):
            return False, dict(reason="dispersal_flag_or_membership")
        for key in ["groups_before", "groups"]:
            groups = row[key]
            if len(groups) != 2 or sorted(g["members"] for g in groups) != expected or any(g["type"] != "standing" for g in groups):
                return False, dict(reason="group_type_or_membership")
        for key in ["before", "humans"]:
            if sorted(h["id"] for h in row[key]) != list(range(1,7)):
                return False, dict(reason="human_population")
        samples += 1
    return samples > 0, dict(samples=samples, native_non_dispersing=True)
