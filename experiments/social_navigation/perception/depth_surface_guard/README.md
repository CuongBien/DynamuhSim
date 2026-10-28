# Depth surface evidence guard — limited planar-object acceptance

**The depth stage withdraws eight false person-support rows on planar boxes, preserves every measured proposal and all prior correct person support. It still leaves a curved-object false positive in the new scene. Overall person semantics remain FAIL.** Existing LiDAR + depth only; no RGB, ROS/default change or World Model training.

[Report](../../../archive/social_navigation/reports/depth_surface_guard/SURFACE_GUARD_REPORT.md) · [acceptance](../../../archive/social_navigation/reports/depth_surface_guard/acceptance.json) · [audit](../../../archive/social_navigation/reports/depth_surface_guard/audit.json) · [old before](../../../archive/social_navigation/reports/depth_surface_guard/preview/old_before/semantic_review.png) / [after](../../../archive/social_navigation/reports/depth_surface_guard/preview/old_after/semantic_review.png) · [remaining new failure](../../../archive/social_navigation/reports/depth_surface_guard/preview/new_remaining_failure/semantic_review.png).

## Method

The unchanged depth projector/shape proposals supply measured 3D clusters. Each cluster with at least60points is tested for at most two vertical planes, from192deterministic point pairs per fit. Inlier tolerance5mm, each plane≥20points and≥25cm vertical span; two-plane normals must be80–100degrees apart. At least98% of cluster points must be explained before planar-surface evidence is reported. Sparse/short/curved/unexplained clusters abstain. No coordinates, actor identity, counterfactual images, labels, future frames or camera RGB enter this calculation.

This is geometric evidence, not a semantic object classifier. A strong planar finding can withdraw only the previous experimental LiDAR person support; it does not delete the cluster, position or prior confidence evidence. Nonplanarity is not labelled person. All semantic/association/training masks remain false. Thin/partially seen people and planar clothing in other poses still require validation.

Surface fitting runs in double precision on the existing CUDA venv from lidar_person_pilot; no additional installs. Seven behavior tests passed before regression. Source/thresholds were frozen in [frozen.json](frozen.json) before regression outputs, then a different fixture was pinned in [scene_frozen.json](scene_frozen.json) before capture. No threshold/config/geometry edits followed the results.

## Reproduce offline results

From the repository root, with existing CUDA runtime/checkpoints and inherited Gazebo plugins already available:

```bash
TASK=experiments/social_navigation/perception/depth_surface_guard
OLD=experiments/social_navigation/perception/lidar_person_pilot
bash "$TASK/run.sh" test
bash "$TASK/run.sh" infer --capture "$OLD/generated/runs/full01" \
 --prior "$OLD/generated/inference/full02" --output "$TASK/generated/manual_regression"
bash "$OLD/run.sh" evaluate --capture "$OLD/generated/runs/full01" \
 --inference "$TASK/generated/manual_regression" --output "$TASK/generated/manual_eval"
```

New paths are required; no overwrite. The recorded fresh fixture uses changed human poses, rotated boxes and a curved cylinder, same native camera/LiDAR and warehouse. Capture is headless on NVIDIA, domain91 locked, bounded, pause/stable/stop/cleanup scoped to owned processes:

```bash
bash "$TASK/run.sh" capture --condition full --output "$TASK/generated/runs/manual_new"
bash "$OLD/run.sh" infer --capture "$TASK/generated/runs/manual_new" \
 --output "$TASK/generated/manual_prior"
bash "$TASK/run.sh" infer --capture "$TASK/generated/runs/manual_new" \
 --prior "$TASK/generated/manual_prior" --output "$TASK/generated/manual_guard"
bash "$OLD/run.sh" evaluate --capture "$TASK/generated/runs/manual_new" \
 --inference "$TASK/generated/manual_guard" --output "$TASK/generated/manual_guard_eval"
# Audit this workspace's actual recorded regression/new-scene outputs:
bash "$TASK/run.sh" audit --output "$TASK/generated/manual_audit"
```

Scene/source manifests pin this measured version; changing a fixture requires a new manifest and validation. Audit uses the known local paths/checkpoint and full arrays, not an arbitrary portable checkout. Original DR-SPAAM inference math/checkpoint/confidence0.5 are reused unchanged; the previous stage's JSON serialization repair is already declared there.

## Results and limits

Across repeated five-frame static records: old full false support4→0, old no-human control3→0, new full2→1. All eight withdrawn rows belong to planar objects; no person-owned cluster is flagged, all40person-owned geometry rows and all75total proposals remain. Correct prior support1→1 overall; recall is still poor. The cylinder at t5s remains wrongly person-supported, and its failure view is explicitly selected as the first remaining false-support case. Other review images use a fixed midpoint t4s.

Surface-only gate PASS is limited to boxes in two layouts of one warehouse/native mesh profile. Original full semantic gates remain FAIL; the no-human gate alone becomes PASS. This is not a multi-simulator/dynamic/real-world validation or a person recognition solution. No HuNavSim, H1–H5/persona validity, navigation TIMEOUT repair, dataset batch or training acceptance.

Fresh capture75depth/20scan uses NVIDIA and exits cleanly. CPU replay agrees with CUDA on75proposals; saved-plane residuals independently checked on18270points. All old files and user index changes are preserved. Git keeps small scan/raw-depth representative bundles, LABEL ONLY views, predictions, logs/hashes and true measured plots; full counterfactual arrays remain local in generated/ and are not backed up by Git. Next: a small annotated depth perception corpus with curved clutter and moving/occluded people, to validate a semantic person-vs-object classifier before World Model training batches.

Cập nhật dọn 27/10/2026: đã gỡ các công cụ chỉ phục vụ cohort/report cũ: `preview.py`. [Code trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/87b0c6eea9b54794db7b4cb794fae57c78f11c70/experiments/social_navigation/benchmark/depth_surface_guard) giữ cách tái lập; report, engine, source freeze và outcome cũ không đổi.

Capture hiện tại dùng chung `python/dynamuhsim/runtime/sensor_capture.py`; file
`capture.py` của stage chỉ chọn fixture, protocol và provenance. Nhận ảnh/scan,
synchronization, giới hạn số frame và các gate static/moving/null giữ nguyên.
Source hash của lượt mới ghi thêm module dùng chung. Không append lượt mới vào
corpus/freeze cũ; tái lập kết quả lịch sử bằng source revision trong manifest.

World của profile được dựng bởi `python/dynamuhsim/simulation/sensor_fixture.py`.
Các file fixture tại stage chỉ chọn input; không gọi vòng qua fixture stage khác.
XML/URDF và metadata đã được đối chiếu byte với source trước khi gom.
