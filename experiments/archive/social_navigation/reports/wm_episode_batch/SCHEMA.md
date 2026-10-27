# WM sensor/action corpus — schema 1

This batch records robot motion with the existing moderate Nav2 behavior policy. It prepares action-conditioned **actually observed future depth occupancy** supervision. It does not establish all social-navigation World Model tasks or an RL environment/reward. Robot input stays LiDAR + depth + IMU/odometry; external observer RGB is debugging media only and is excluded from the portable inputs.

## Directory layout

```text
dataset01/
  dataset.json
  capture_protocol.json, export_protocol.json, quarantine_review.json
  tools/loader.py
  episodes/<episode_id>/
    episode.json, inventory.json
    inputs/
      samples_SENSOR_ONLY.npz
      observations_SENSOR_ONLY.npz
      depth_SENSOR_ONLY.npy
      calibration_SENSOR_ONLY.json
      lidar_geometry_SENSOR_ONLY.json
      bev_anchor_estimated_SENSOR_ONLY.npz
    labels/
      future_trajectories_actions_LABEL_ONLY.npz
      future_observed_bev_LABEL_ONLY.npz
      human_states_LABEL_ONLY.jsonl
      task_masks_LABEL_ONLY.npz
      episode_end_LABEL_ONLY.json
    provenance/
      capture_result.json, capture_audit.json, raw_export_audit.json
      legacy_normalizer_manifest.json, legacy_normalizer_qa.json
```

Full raw MCAP, paired diagnostic arrays, runtime world/robot/config and observer media remain in `generated/runs/` and `generated/runtime/`, outside the portable observation branch. The failed standing47 and walking49 captures remain raw and has no portable episode directory. To back up the complete evidence, copy these folders too. To read the training arrays independently, copy the **entire dataset01 directory**, preserving all relative paths. Files are plain files/hardlinks, not symlinks to an external dependency; copying gives self-contained bytes.

## Sensor observations

At10Hz anchors, `samples` stores20queries from t−1.9s through t. Header stamp and recorded availability must both be ≤each history query. Availability=max(header, latest simulation-clock reception/bag time), a conservative logging contract, not wall-clock latency metrology. Latest eligible measurements are held; no new LiDAR scans are invented between native5Hz scans. Original age limits apply: scan250ms, depth200ms, IMU/odom/action150ms. IMU/odom/action require full history support; scan/depth ≥90% with valid last sample. The masks remain explicit.

| Payload | Shape per window | Meaning |
| --- | --- | --- |
| `history_query_ns` | [20] int64 | Query times; last is anchor. |
| `scan_ranges_m`, `scan_ray_valid` | [20,360] float32/bool | Native LiDAR, original0/2π duplicate endpoint retained in this corpus. Structured invalid rays are0 with validityfalse; original raw messages remain in MCAP. |
| `imu` | [20,10] | Quaternion xyzw, angular velocity xyz, linear acceleration xyz. |
| `odom_local_pose_and_twist` | [20,9] | Estimated XY/yaw relative to last causal odom, plus linear xyz and angular xyz. No GT correction. |
| `action` | [20,3] | Executed vx/vy/wz history. Native header base_footprint shares planar axes with base_link. |
| `depth_index` | [20] int32 | Indices into full float32 depth array [F,240,424]. NaN/Inf stay bitwise as recorded; use finite/range and history masks before neural input processing. |
| `goal_xy_from_amcl`, `goal_valid` | [2], scalar | Known fixed task goal expressed using causal AMCL; stale >1s becomes zero/maskfalse, never filled using GT. |
| `*_stamp_ns`, `*_available_ns`, `*_valid` | [20] | Per-modality timing and support. |

`Dataset(...).episode(id).observation(i)` returns only `history`, `depth_history`, `bev_anchor_estimated`, `bev_frame_valid`, and `calibration`. It never reads human/world GT, persona, backend IDs or future labels. Backend/seed/episode IDs exist for provenance and splits; do not append them to model features. `observations_SENSOR_ONLY.npz` retains full timestamped sensor sequences for audits; training code should use the causal window API rather than accessing later measurements directly.

Input BEV is depth projected through measured intrinsics/static extrinsics, then warped to the estimated anchor using odometry that was available at the source depth time and anchor. The BEV file retains source depth/odom and anchor odom stamps/availability. This is an estimate, with residual localization error; simulator pose never corrects the input. Depth clip0.28–5m and low camera pose remain unchanged.

## Action conditioning and future labels

`action_condition(i)` is a separate explicit accessor for30executed future commands at t…t+2.9s, with header/availability/query stamps. It is appropriate as the specified **action-conditioned WM training input**, while `.observation(i)` alone remains causal. At deployment/planning, a candidate action sequence would have to be supplied explicitly. This batch observes one Nav2 behavior-policy trajectory per situation; it is not measured counterfactual outcomes for all action choices or an optimal/expert policy dataset.

Future occupancy has shape[N,3,64,64], horizons+0.5/+1/+3s. A recorded future depth frame must be within50ms of its query, later than anchor, and supported by scene ego poses. GT ego SE2 warps its points and camera origin into true robot axes frozen at anchor. GT poses live **only in LABEL** files. Input estimated anchor and target true anchor can differ; this is retained uncertainty, not corrected by an oracle. Planar SE2 does not establish a full world3D calibration.

BEV x forward[0,6.4), y left[−3.2,3.2), cell0.1m. Grid row indexes x, column indexes y. Occupancy above relative base_link height0.03…1.8m is1; observed ray-free cells0; unobserved cells−1. Invalid/Inf/missing depth does not carve free space. `cell_loss_valid` masks unknown and unsupported target frames. These are **future actually observed occupancy targets**, not whole-world occupancy, unseen-area completion, safe Nav2 costmaps or learned social cost.

Legacy future human trajectories are[N,30,6,2] in true anchor axes; group modes/membership are simulator labels. Instance IDs are episode-local. `human_valid` means interpolation support, **not visibility/detection**. No sensor-track association is accepted, so track-aligned/persona supervision is masked. Never infer H1–H5 from standing/walking or the name of a simulator.

## Task masks and episode ends

- `wm_observed_bev_train_valid[N]`: capture passes strict movement/infrastructure gates, input BEV has causal pose support, all3target frames have temporal/pose support and observed cells. This is task-data validity, not permission to train on a test split. Respect `split` separately.
- `executed_action_condition_valid[N]`: raw command windows meet the normalizer contract.
- `m1_track_aligned_valid[N,30,6]` and `persona_valid[N,6]`: allfalse.
- `rl_reward_valid[N]`: allfalse, rewardNaN. Do not feed invalid NaN rewards into loss or pretend they are zero rewards.

Unknown occupancy should be selected out of loss (for example `loss(logits[cell_loss_valid], target[cell_loss_valid])`), not interpreted as an empty/negative target. Masks with false semantic/persona support are unavailable supervision, not absence of people/behavior.

`episode_end_LABEL_ONLY.json` records success vs truncation at the collection boundary: actual goal success terminates, time budget or controller stop truncates. This is episode bookkeeping; no validated contact-force truth, RL reward or Gym step/reset interface is provided. Samples stop3s before action end to keep the full future action/target horizon. Do not treat the last exported window as the actual terminal time.

## Splits, provenance and limitations

Five seeds45–49 were assigned before capture:45–47train,48validation,49test, keeping both initial modes of a seed together. Two failed raw captures (standing47 and walking49) are quarantined; no replacement/retry. Test episode access is blocked by default; `allow_test=True` is reserved for declared test use or automated integrity QA. No WM was fitted or scored, and no test imagery was viewed or published in this collection stage. The native recorder may create debug media/legacy BEV previews as raw artifacts automatically; these are excluded from portable inputs and remain uninspected for test scenes.

Same warehouse route and avatar appear across all splits; seed separation does not establish held-out environments/people/simulators. Native group lifecycle may disperse, so an initial standing fixture is not a promise that all six people converse throughout. SocNavGym is the only backend in this first batch; CrowdSimPlus diversity and HuNavSim external testing remain later work. HuNavSim is untouched. Collection does not require a perfect person classifier because observed-depth WM targets are sensor-derived and unsupported human/persona tasks stay masked.

The legacy normalizer retains historical `pilot_only`/overall-trainingfalse metadata; its code is snapshotted unchanged for causal sampling and raw audits. Canonical **new dataset split and per-task eligibility** are `dataset.json`, `episode.json` and task masks. Do not reinterpret old manifests as newly accepted semantic supervision. Capture, export and reviewed quarantine continuation each retain their own frozen source hashes; the original failed clean-exit gate remains false.

Final accepted split is5train/2validation/1test. Only standing49 remains as test; walking49 is excluded raw quarantine, so this batch cannot independently evaluate walking on held-out episodes.
