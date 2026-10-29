# Ground-truth bridge shutdown validation

The preserved crossing group_cohesion_v2 capture aborted in parameter_bridge after Gazebo server exit, during ROS launch SIGINT teardown (pthread_mutex_lock owner assertion, exit-6). Temporal evidence does not establish the upstream root cause.

New capture-only mitigation: after acknowledged stable pause and recorder closure, send exactly one SIGINT to the session-owned /common/ground_truth parameter_bridge while the Gazebo publisher is alive. Exact executable/topic/node arguments and session token are checked; a Linux pidfd pins process identity across the final check and signal. Wait for ROS launch child-watcher terminal record without reaping its child, then close Gazebo and remaining ROS launch normally. Failed receipt remains failed; capture audit adds an ordered bridge exit check and retains no_child_exit_errors. Defaults and installed ROS libraries are untouched.

plan.json declares two fresh seed45 cells before execution. No tuning, retry, overwrite, policy waiver, dataset promotion or claim of universal repair. Original crossing FAIL remains visible. New outcomes will be appended after execution.
