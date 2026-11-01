# ROS domain ownership

Each `GazeboROSBackend` reserves its domain before creating a ROS context.
Omit `--domain` (or pass `ros_domain_id=None`) for automatic allocation. An
explicit `--domain 61` requires that domain to be unowned; it does not fall back
to another domain. The selected domain is recorded in training `config.json`.
The inherited `ROS_DOMAIN_ID` does not override this explicit selection.

The backend uses nonblocking Linux `flock` on a global host-local file in `/tmp`.
The lock is authoritative, not its PID metadata. Lock files remain in place and
must not be deleted while workers may run: unlinking a locked inode would allow
two independent locks for the same domain. Creation protects against symlinks;
the file is shared by cooperating local users. The lease FD is not generally
inheritable, but is explicitly passed to the owned `ros2 launch` process.

The lease lasts for the backend's whole lifetime. `reset()` tears down the old
episode with `_stop_episode()`, retaining the same domain and lock. Final
`close()` stops the owned process group and ROS context/executor before closing
the lease. Initialization exceptions also close resources. Callers must still
use `finally: env.close()` or a context manager for runtime errors, as the
training/evaluation entrypoints do. A closed backend cannot be reset; create a
new worker instead. A worker/context must not be reused after `fork`.

Kernel FD cleanup prevents stale locks after process death. If a worker crashes
while its owned launch remains alive, the launch's inherited FD keeps the domain
reserved until that live launch exits; this is not stale ownership. If teardown
cannot establish that owned resources stopped, it fails with the lease retained
rather than making an occupied domain available. Only the owned process group
is signalled. No unrelated ROS/Gazebo process is terminated.

## Port policy

This implementation targets the project's Linux/ROS2 DDS environment. It reads
`/proc/sys/net/ipv4/ip_local_port_range` and reserves a full 250-port RTPS block
per domain, budgeting up to 120 participants. Blocks overlapping the actual
ephemeral range or exceeding UDP port 65535 are rejected. With the usual Linux
range `32768 60999`, configured domains are conservatively `0–100` and `215–231`.
Automatic allocation also avoids conventional domain 0. Boundary domains 101
and 232 are deliberately excluded because they have smaller participant
budgets. A missing/unreadable platform port policy fails closed.

The port formulas and boundary limits follow the
[official ROS2 domain-ID documentation](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Domain-ID.rst).
Do not run more than 120 participants per domain or customize DDS port mappings
without revisiting this policy. Lease allocation is not a socket reservation:
unrelated services can still occupy ports, in which case ROS initialization
must fail rather than silently sharing a worker domain.

## Scope of isolation

A local advisory lock coordinates **cooperating local workers only**. It cannot
prevent an unrelated local or remote ROS participant from using the domain.
Before the first simulator launch, a bounded discovery check refuses an
existing publisher on `/clock`, `/imu`, `/scan`, `/odom` or `/cmd_vel`. This is
best effort, not proof against late/remote discovery. For stronger cross-host
isolation, use network/DDS isolation or coordinated domain allocation.

Native Gazebo partitions remain unique per episode. Sensor timestamps,
freshness checks, WorldControl requests/fallback, observations, rewards and PPO
hyperparameters are unchanged. Domain ownership does not lock training output
paths: use a fresh `--run-dir` for each run.
