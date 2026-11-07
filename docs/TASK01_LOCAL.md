# Task 01 on local Ubuntu

From the repository root, run:

```bash
bash tools/run_task01.sh
```

This builds the existing small native helpers, refreshes the installed launch files,
and runs one `crowd_orca_plus` / `crossing` / seed `42` episode. Docker uses only the
local `dynamuhsim-crowdsimplus:round2` image, by content ID, without network access or
image pulls. No package installation is performed.

Gazebo GUI and RViz start together with the capture's ROS domain, Gazebo partition,
simulation clock, warehouse map and SDF-derived robot geometry. RViz is centered on
the robot's route. Orange cylinders show measured Gazebo human poses; the green
robot marker shows Gazebo ground truth for comparison with the localized RobotModel.
These markers never feed AMCL or MPPI.

On the local Mesa/Intel driver, RViz's indexed-map shader initially assigned two
different sampler types to texture unit zero. The capture creates a frozen resource
overlay with explicit sampler bindings for the RViz process only. System packages,
Gazebo GPU rendering and the map pixels are not modified. The particle-cloud display
also uses the native Jazzy plugin and best-effort QoS.

For a capture without viewer windows:

```bash
bash tools/run_task01.sh --no-gui --no-rviz
```

Each invocation allocates a new directory under:

```text
experiments/social_navigation/datasets/single_policy_v1/generated/pilot_orca_crossing_seed42/capture_<time>_<unique-id>/
```

Do not reuse `capture_02`. Its existing `episode/` is intentional: it is a completed
historical capture, not a failed build. `prepare` and `run` refuse to overwrite it.
The one-step command stops on any build or prepare failure, so later commands cannot
accidentally continue against an old freeze. Editing source invalidates an older
freeze for rerunning; its recorded MCAP and historical manifest remain unchanged.

Useful options:

```bash
# Build and freeze only; do not start Gazebo or move anything.
bash tools/run_task01.sh --prepare-only

# Choose another domain if 92 is already occupied. The runner probes and locks it.
bash tools/run_task01.sh --ros-domain-id 93
```

The synchronized profile is `warehouse_sync_v1`. It checks that all static obstacles
and every occupied map cell match the Gazebo world before starting. It uses the native
SDF to derive the URDF instead of displaying a differently sized robot. AMCL retains
the OmniMotionModel and receives a small, declared initial covariance from the known
stationary simulation spawn. Its motion noise is calibrated for this Gazebo wheel
odometry; likelihood-field beam skipping reduces interference from moving humans.
The goal checker requires simultaneous XY/yaw agreement with a 5 cm XY tolerance.
All MPPI parameters and the human ORCA+ algorithm are unchanged. These localization
settings are for the local simulation, not a claim of physical-robot calibration.

The runner checks live parameters, compares localization and Gazebo at identical
timestamps, records before opening the motion gate, flushes MCAP, and reads the closed
bag to verify required topics and timestamps. `manifest.json` reports PASS/FAIL;
the one-step command exits nonzero when physical arrival or another gate fails.
It never starts an automatic replacement episode.
Ctrl+C/SIGTERM enters the recorder flush and owned-process cleanup path; it does not
turn an interrupted run into a successful episode.

Artifacts include `episode/raw_bag/*.mcap`, `metadata.yaml`, `runtime/worker.log`,
`runtime/policy_audit.json`, `runtime/rviz.log`, `episode/launch.log`, frozen sensor/map
assets, runtime robot parameters and source hashes.
