# Native geometry / inflation physical comparison

The native Gazebo robot is unchanged. Recipe v4 generates a run-local URDF from that SDF, including collision primitives and fixed transforms, and declares a 0.17 m Nav2 circle with 0.01 m padding. Existing recipe v1-v3 defaults remain unchanged.

The predeclared arms in `plan.json` compare the old recorded baseline with corrected geometry at inflation 0.80 m and 0.45 m. Both new arms use standing seed 45 and headless NVIDIA rendering. No waypoint or artificial command forces passage through people.

This is a diagnostic single-seed comparison. It is not a multi-seed navigation benchmark, social-policy evaluation, or World Model training dataset release. Goal completion, narrow-gap crossing, and descriptive geometric clearance are reported separately. Original captures are preserved.

## Measured results

| Arm | Native goal status | Active sim time | Actual path length | Actual goal XY error | Original narrow gap A |
| --- | --- | ---: | ---: | ---: | --- |
| A: recorded old radius .25, inflation .80 | SUCCEEDED | 69.07 s | 10.754 m | .0701 m | Not crossed |
| B: aligned URDF/TF, radius .17, inflation .80 | SUCCEEDED | 68.50 s | 10.884 m | .0949 m | Not crossed |
| C: aligned URDF/TF, radius .17, inflation .45 | SUCCEEDED | 30.89 s | 9.054 m | .0674 m | Not crossed |

Both new native capture integrity audits PASS, including the complete live parameter contract and numeric static TF against the aligned URDF. Runtime commit: `a898fa4`. Unit suite: 239 tests, OK, 2 conditional skips. Source files and native physics/camera bytes are frozen per run; `evidence.json` records raw paths and hashes. Both new attempts completed without retry. B encountered a recoverable Nav2 progress failure before eventual success; it is retained in the original launch log.

The measured paths still pass north of both groups. The recorded global plans also choose the detour. This is evidence that a geometry correction alone is insufficient to make this configured planner choose the narrow opening. It does not prove the opening is impossible or attribute the choice to a specific MPPI critic. No forced waypoint or velocity command was introduced.

The .45 m result is shorter/faster in this single controlled seed. It is an experimental candidate, not the new default or a multi-seed performance claim. Camera video is re-encoded from the recorded native observer GIF, 4fps from one image per simulated second. Skeletal gait in the separate light replay is reconstructed from travel distance.

C minimum conservative swept robot-human gap: .3393 m. Static clearance lower bound: .9376 m against all 42 upright native obstacle boxes, excluding the supporting floor. Static bounds use endpoint distance minus half segment length and robot bounding radius; human relative motion is linearly swept. These are geometric diagnostics, not native contact-force truth. The maximum robot collision enclosing radius used for all three arms is .168062484748657 m.

## Reproduce

Run from `/home/cuongbien/DynamuhSim`. Use NEW run/output paths; existing captures and audits deliberately refuse overwrite.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
python3 tools/capture_episode.py prepare --run-dir experiments/social_navigation/benchmark/generated/NEW_GEOMETRY_RUN --config configs/capture/geometry_v4_inflation045.json --scenario standing --seed 45
python3 tools/capture_episode.py run --run-dir experiments/social_navigation/benchmark/generated/NEW_GEOMETRY_RUN
python3 tools/audit_episode.py --run-dir experiments/social_navigation/benchmark/generated/NEW_GEOMETRY_RUN --policy configs/audit/capture_native_geometry.json --source-root /home/cuongbien/DynamuhSim --output experiments/social_navigation/benchmark/generated/NEW_GEOMETRY_AUDIT.json
```

`compare.py` reads only preserved runs listed in `plan.json`, and regenerates `results.json` / `physical_comparison.png`. `render_media.py --arm C_aligned_045 --output NEW.webm` reproduces the camera encoding without overwriting an existing artifact. Source timestamps and hashes are recorded in `evidence.json`.

The next investigation is a separate, explicit global-planner comparison of a physically open narrow passage versus the detour, with path/clearance metrics and logged costs. Passing through a conversation group is not automatically a desirable social policy. No World Model or RL training has started here.
