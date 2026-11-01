"""Opt-in native group solver telemetry with no vendor/global mutations.

Clone only the bound method's global namespace to substitute a forwarding RVO
factory. The simulator, call order, native return and RNG are unchanged. This
instrument applies to the legacy native method, not arbitrary motion profiles.
"""
import types


class GroupDiagnostics:
    def __init__(self, kernel):
        self.kernel = kernel
        self.last = []
        original = kernel.env.compute_orca_interaction_velocities
        function = original.__func__
        namespace = dict(function.__globals__)
        native_rvo = namespace['rvo2']
        def factory(*args, **kwargs):
            return _RecordedSimulator(native_rvo.PyRVOSimulator(*args, **kwargs), self)
        namespace['rvo2'] = types.SimpleNamespace(PyRVOSimulator=factory)
        cloned = types.FunctionType(function.__code__, namespace, function.__name__, function.__defaults__, function.__closure__)
        cloned.__kwdefaults__ = function.__kwdefaults__
        self.original = original
        kernel.env.compute_orca_interaction_velocities = types.MethodType(cloned, original.__self__)


class _RecordedSimulator:
    def __init__(self, simulator, owner):
        self.simulator = simulator
        self.owner = owner
        self.agents = []

    def __getattr__(self, name):
        return getattr(self.simulator, name)

    def addAgent(self, *args, **kwargs):
        agent = self.simulator.addAgent(*args, **kwargs)
        self.agents.append(agent)
        return agent

    def doStep(self):
        sim = self.simulator
        groups = self.owner.kernel.env.moving_interactions
        agents = self.agents[len(self.agents)-len(groups):] if groups else []
        rows = []
        for group,agent in zip(groups,agents):
            rows.append(dict(members=[int(h.id) for h in group.humans],goal=[float(group.goal_x),float(group.goal_y)],
                             center=list(sim.getAgentPosition(agent)),envelope_radius_m=float(sim.getAgentRadius(agent)),
                             solver_input_velocity=list(sim.getAgentVelocity(agent)),raw_preferred_velocity=list(sim.getAgentPrefVelocity(agent))))
        result = sim.doStep()
        for row,agent in zip(rows,agents):row['post_orca_velocity']=list(sim.getAgentVelocity(agent))
        self.owner.last = rows
        return result
