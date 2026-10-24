"""Bounded action execution, independently testable without ROS/Gazebo."""
import time

from dynamuhsim.analysis.geometry import pose_errors


def wait_for(predicate, seconds, *, spin_once, processes, monotonic=time.monotonic):
    deadline = monotonic() + seconds
    while monotonic() < deadline:
        if any(proc.poll() is not None for proc in processes):
            raise RuntimeError("Owned launch or worker exited")
        spin_once()
        if predicate():
            return True
    return False


def goal_metrics(robot, action_status):
    """The existing task goal is (-20, 0, 0); status alone never proves arrival."""
    xy, angle = pose_errors(robot, (-20., 0., 0.))
    return dict(final_world_pose=robot, goal_xy_error_world_m=xy, goal_yaw_error_world_rad=angle,
                actual_goal_reached=action_status == 4 and xy <= .2 and angle <= .25)


def execute_goal(client, goal, recording, runtime, result, wait, capture_tf, *, on_accepted=lambda handle: None, on_started=lambda start: None, monotonic=time.monotonic):
    future = client.send_goal_async(goal, feedback_callback=recording.feedback)
    if not wait(future.done, 10):
        raise RuntimeError("No goal response")
    handle = future.result()
    if not handle.accepted:
        raise RuntimeError("Goal rejected")
    # Expose the accepted handle immediately so outer cleanup can cancel even if recording fails.
    on_accepted(handle)
    start = recording.simtime()
    recording.phase = "active"
    on_started(start)
    (runtime / "start_gate").write_text(str(start))
    result["goal_start_sim"] = start
    pending = handle.get_result_async()
    wall_start = monotonic()
    deadline = wall_start + max(240, result["sim_budget_s"] * 6)
    last_tf = -1.
    timed_out = False
    while not pending.done():
        wait(pending.done, .04)
        if recording.simtime() - last_tf > .2:
            capture_tf()
            last_tf = recording.simtime()
        if recording.simtime() - start >= result["sim_budget_s"] or monotonic() > deadline:
            timed_out = True
            cancel = handle.cancel_goal_async()
            wait(cancel.done, 5)
            break
    result.update(action_end_sim=recording.simtime(), active_sim_seconds=recording.simtime() - start,
                  wall_seconds=monotonic() - wall_start, action_status=None if timed_out else pending.result().status)
    result["navigation_status"] = "TIMEOUT" if timed_out else {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}.get(result["action_status"], "UNKNOWN")
    if not timed_out:
        payload = pending.result().result
        result.update(action_error_code=getattr(payload, "error_code", None), action_error_msg=getattr(payload, "error_msg", None))
    return handle
