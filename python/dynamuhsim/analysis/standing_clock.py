"""Snapshot-only clock contract for explicitly persistent stationary triplets.

This does not exercise arrival/expiry queries and never replaces the historical
query verifier. Only an explicitly frozen conversation profile may select it.
"""
import math
from dynamuhsim.simulation.conversation import verify_conversation


def verify_snapshots(rows, gate, ready):
    rows = list(rows)
    okay, _ = verify_conversation(rows)
    if not okay or not math.isfinite(gate):
        return False, dict(reason="conversation_contract")
    targets = {hid:h["goal"] for hid,h in ready["humans"].items()}
    if set(targets) != {str(i) for i in range(1,7)}:
        return False, dict(reason="initial_population")
    activated = False
    last_end = None
    active = snapshots = 0
    def close(a, b):
        return type(a) in [float,int] and math.isfinite(a) and abs(a-b) <= 1e-8
    for row in rows:
        snap = row["goal_clock"]
        if snap["profile"] != "sim_v1" or snap["domain"] != "gazebo_sim" or set(snap["humans"]) != set(targets):
            return False, dict(reason="clock_identity")
        if not row["active_phase"]:
            if activated or snap["gate_start_sim_s"] is not None or snap["evaluation_time_sim_s"] is not None or snap["events"] or not close(snap["elapsed_active_sim_s"],0.):
                return False, dict(reason="inactive_clock")
            for hid,h in snap["humans"].items():
                if h["goal"] != targets[hid] or h["assigned_sim_s"] is not None or not close(h["age_sim_s"],0.):
                    return False, dict(reason="inactive_goal_age")
            continue
        if not all(type(t) in [float,int] and math.isfinite(t) for t in [row["time"],row["state_time"]]) or row["time"] < gate-1e-6 or not close(row["state_time"]-row["time"],.1):
            return False, dict(reason="step_time")
        if last_end is not None and abs(row["time"]-last_end)>1e-6:
            return False, dict(reason="noncontiguous_steps")
        expected_events = [] if activated else [dict(event="activated",time_sim_s=gate)]
        if snap["events"] != expected_events:
            return False, dict(reason="unexpected_assignment_or_query")
        activated = True
        if not close(snap["gate_start_sim_s"],gate) or not close(snap["evaluation_time_sim_s"],row["state_time"]) or not close(snap["elapsed_active_sim_s"],row["state_time"]-gate):
            return False, dict(reason="clock_gate_or_age")
        for hid,h in snap["humans"].items():
            if h["goal"] != targets[hid] or not close(h["assigned_sim_s"],gate) or not close(h["age_sim_s"],row["state_time"]-gate):
                return False, dict(reason="goal_target_or_age")
            snapshots += 1
        last_end = row["state_time"]
        active += 1
    return activated and active>0, dict(contract="standing_snapshot_v1",active_steps=active,human_age_snapshots=snapshots,
        evaluations=0,assignments=0,arrival_expiry_query_coverage="NOT_EXERCISED",last_state_time_sim_s=last_end)
