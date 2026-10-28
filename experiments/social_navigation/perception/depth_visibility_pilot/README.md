# Static depth visibility and person-vs-clutter diagnostic

A label-only Gazebo experiment now distinguishes which measured depth pixels belong to each native human mesh. **The geometry candidate pipeline is still not a validated person detector:** it keeps four person-owned proposals and two object proposals; the no-human control keeps the same two objects. No ROS default, tracker threshold, navigation profile, dynamic dataset labels or training mask changed.

[Report](../../../archive/social_navigation/reports/depth_visibility_pilot/VISIBILITY_REPORT.md) · [acceptance](../../../archive/social_navigation/reports/depth_visibility_pilot/acceptance.json) · [independent audit](../../../archive/social_navigation/reports/depth_visibility_pilot/audit.json) · [real depth review](../../../archive/social_navigation/reports/depth_visibility_pilot/visibility_review.png) · [evaluation](../../../archive/social_navigation/reports/depth_visibility_pilot/evaluation.json).

## What ran

One designed warehouse fixture, six frozen native mesh actors at animation time zero, original low camera calibration/clip, two clutter boxes. Fifteen co-located depth cameras render five synchronized timestamps: full and duplicate control, no-human control, six leave-one-person-out views, six person-only silhouettes. These are GPU-rendered depth arrays, not generated illustrations. Only full depth is used to form proposals; counterfactuals/instance ownership are evaluation labels. Null depth is a separate negative control.

Four people have eligible visible support; one is behind the camera and one beyond its 5 m range. The rear person is **partially** visible (11.92%), not fully occluded. This fixture does not supply a real fully-occluded-person acceptance case or dynamic identity/visibility supervision. Five repeated frames are one scene, not five independent episodes. Visibility ratio is within camera view/range, not full-body coverage.

## Run

Ubuntu24.04, ROS2 Jazzy, Python3.12.3, NumPy/SciPy/Matplotlib, Gazebo Harmonic8.15.0 and ROS vendor OgreNext2.3.3 were used. Hardware renderer is required; actual Ogre logs verify NVIDIA RTX4060 via WSL D3D12. No Python installation, vendor or system configuration was changed. Rendering uses the GPU; physics/transport/analysis still use CPU. Offline BLAS is limited to one thread.

```bash
TASK=experiments/social_navigation/perception/depth_visibility_pilot
bash "$TASK/run.sh" test
# Existing actor-freezer dependency; build only if absent:
cmake -S experiments/social_navigation/perception/camera_height_ab \
  -B experiments/social_navigation/perception/camera_height_ab/generated/build
cmake --build experiments/social_navigation/perception/camera_height_ab/generated/build -j1
bash "$TASK/run.sh" build
# New output name required. One bounded, headless capture; no automatic retry.
bash "$TASK/run.sh" capture --output "$TASK/generated/runs/manual01"
bash "$TASK/run.sh" analyze --capture "$TASK/generated/runs/manual01" --output "$TASK/generated/manual_analysis"
bash "$TASK/run.sh" audit --capture "$TASK/generated/runs/manual01" \
  --analysis "$TASK/generated/manual_analysis" --output "$TASK/generated/manual_audit"
```

Capture uses the existing domain91 lock and its own partition/process token; pause/stable/stop precedes bridge cleanup. Failed outputs are retained and refuse overwrite. The accepted local source is generated/runs/static05, analysis01 and audit01. Full raw stacks stay under generated/ and are not backed up by Git. Git retains exact representative NPZ arrays, LABEL ONLY instance masks, source/runtime hashes, native logs and measured review plots. Audit of these exact runs expects the original workspace/checkpoint and local arrays, not just a report checkout.

## Gates and limitations

Attempts01–03 failed camera-info wiring;02 was interrupted and its original finalizer raised after ROS auto-shutdown, so an explicitly marked recovery record replaces a missing final receipt. Attempt04 captured all views but failed strict inter-frame bitwise equality at up to four pixels (max1.66893e-6m). Its FAIL remains. Protocol2 was declared before fresh static05: no finite-mask changes and inter-frame error <= the already-used 1mm label tolerance; simultaneous full/control must still match bitwise. Static05 passes this narrower numeric gate; its strict inter-frame bitwise result remains false. This is a development fixture, not a held-out result.

Pixel ownership requires both a deeper/empty ray after removing the person and equal depth in that person-only image. Contradictory/overlapping ownership is unknown, not force-labelled. Nine contract/label tests and an independent scalar audit of57070 pixels passed. The unchanged geometry filter retains the two clutter clusters in both full and null scenes. The reported 4/6 precision and4/4 recall are a single-scene diagnostic **if every geometry proposal were called a person**, not a claimed semantic classifier performance score.

The one-Hz static capture is not a tracking benchmark: active TTL is0.5s, so the existing tracker cannot accumulate normal 10Hz continuity. Empty confirmed-nonperson fields do not prove false-positive rejection. Semantic, association, persona H1–H5 and training masks remain false. Existing seeds42/43/44 are not retrospectively annotated by this snapshot. HuNavSim remains unused for final testing; navigation TIMEOUT remains separate.

Next: add a sensor-only person-vs-object semantic stage, validate it on this fixed clutter/visibility regression, then a dynamic occlusion fixture and a new independent scene. Do not tune on HuNavSim or start a training batch from these labels.

Capture hiện tại dùng chung `python/dynamuhsim/runtime/sensor_capture.py`; file
`capture.py` của stage chỉ chọn fixture, protocol và provenance. Nhận ảnh/scan,
synchronization, giới hạn số frame và các gate static/moving/null giữ nguyên.
Source hash của lượt mới ghi thêm module dùng chung. Không append lượt mới vào
corpus/freeze cũ; tái lập kết quả lịch sử bằng source revision trong manifest.

World của profile được dựng bởi `python/dynamuhsim/simulation/sensor_fixture.py`.
Các file fixture tại stage chỉ chọn input; không gọi vòng qua fixture stage khác.
XML/URDF và metadata đã được đối chiếu byte với source trước khi gom.
