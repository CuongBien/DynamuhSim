# Portable depth perception corpus, schema 1

This is a small simulator-specific perception development pilot. It is not a World Model training corpus. RGB, H1–H5 behaviour labels, navigation actions and future trajectories are not model inputs.

## Bundle layout

```text
manifest.json
protocol.json
loader.py
inputs/
  calibration_SENSOR_ONLY.json
  p001/ ... p007/
    depth_SENSOR_ONLY.npz
    lidar_SENSOR_ONLY.npz
    lidar_geometry_SENSOR_ONLY.json
    proposals_SENSOR_ONLY.npz
labels/
  p001/ ... p007/
    instance_pixels_LABEL_ONLY.npz
    proposal_labels_LABEL_ONLY.npz
    proposal_details_LABEL_ONLY.json
    visibility_LABEL_ONLY.json
    source_provenance_LABEL_ONLY.json
```

`manifest.json` assigns entire scene families to train (A), validation (B) and test (C). Static scenes select the midpoint frame (raw index 2) once; moving scenes retain all five 1 Hz frames in the same split. These are correlated frames, not independent episodes or people. The test API is sealed by default. Test use in this milestone is automated integrity/label QA only; no classifier evaluation, visual preview or model selection.

## Input arrays

| File | Arrays | Meaning |
| --- | --- | --- |
| depth | `depth[F,240,424]` float32, `stamp_ns[F]`, `available_ns[F]` int64 | Native camera optical-axis depth, metres; nonfinite stays nonfinite. No actor visibility masks in inputs. |
| lidar | `ranges[L,360]` float32, `stamp_ns[L]`, `available_ns[L]` int64 | Native, noisy 2D LaserScan; includes original duplicate 0/2π endpoint. Not inferred from depth. |
| proposals | `points_xyz[P,3]` float32, `point_pixel_uv[P,2]` uint16, `point_offsets[N+1]` int64 | Ragged candidate points, base_link metres, and source pixels (u,v). |
| proposals | `bbox_uv[N,4]` int16, `frame_index[N]`, `proposal_in_frame[N]` int16 | Bounding box is [u_min,v_min,u_max_exclusive,v_max_exclusive]. Candidate order is unchanged. |

The full depth frame is provided so later methods can replace the geometry proposal generator. This corpus exports the unfiltered baseline proposals, retaining curved clutter that earlier shape filters could reject. An absent proposal is not an absence-of-person label.

The portable NumPy loader returns one current frame/proposal plus the latest eligible scan: scan stamp ≤ depth stamp, scan availability ≤ depth availability, and scan age ≤250 ms. Missing scan yields `scan_valid=False` and NaN ranges. It never returns later frames, world poses, pixel ownership or labels. `available_ns` is max(header stamp, latest simulation clock observed by the capture callback), not wall-clock latency or a guaranteed ordering of callbacks within one simulation tick. This robot is stationary during these captures.

`optical_to_base_link` and camera intrinsics are in the calibration file. Original low camera position, 424×240 resolution, 0.28–5 m range and robot dimensions remain unchanged. Depth observes the person mesh; the original LiDAR still observes protective proxy cylinders. Their positions are checked, but the surface shapes are intentionally different.

## Labels and masking

`person_instance[F,240,424]`: int16 IDs 1–6, environment 0, unknown −1. Person ownership requires the drop-one-person view to reveal a deeper/background ray and the person-only view to agree with full depth within 1 mm. Null-person counterfactual mismatches and overlapping/conflicting ownership become unknown. Co-timed full/control depth must be bitwise equal. These interventions belong only in the raw LABEL provenance, never the input branch.

`class_index[N]`: int8, environment 0, person 1, ambiguous −1. `label_valid[N]` bool. Valid person requires exactly one person ID, ≥5 person points, ≥90% point purity and no unknown points. Valid environment requires every candidate point to be labelled environment. Mixed people, small person fragments and unknown points are retained with validity false. Do not train on ambiguous rows.

`person_id[N]` is an experiment-local simulator instance label for valid person rows, not a detector identity or semantic re-identification result. `person_purity` and `unknown_point_count` expose label quality. `persona_index=-1` and `persona_valid=False` for every row; person instances and standing/walking poses do not establish H1–H5.

Visibility fractions divide visible pixels by the silhouette in a person-only in-view rendering. They are not whole-body visibility or a learned occlusion prediction. A person behind the camera or beyond range is not evidence of full occlusion. Raw full/null/drop/only arrays and scripted scene/ECM traces are retained under `generated/runs/<scene>/`, outside the portable input branch.

## Reading a sample

Run with Python 3 and NumPy after copying the entire exported bundle anywhere; ROS and Gazebo are not required:

```python
from loader import load_input, load_label
sample = load_input("/path/to/corpus01", "p003", 0)
supervision = load_label("/path/to/corpus01", "p003", 0)
assert supervision["label_valid"]
# Model features must be derived only from sample.
```

The two calls are deliberately separate. Explicit `allow_test=True` is reserved for the declared test stage or automated QA. It is not used to tune a model in this milestone.

## Limitations

One warehouse and one avatar mesh with different angles and animation phases; diagnostic scripted translations and gait phases, not autonomous social behaviour. Three layout families do not establish cross-simulator, cross-person or real-world generalisation. Do not interpret frame count or candidate count as independent training episodes. No learned classifier is validated here and no ROS/Nav2 default changes follow from label integrity alone. HuNavSim remains unused and held aside for later external testing.
