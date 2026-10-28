# Pretrained LiDAR person pilot — DR-SPAAM

**CUDA execution works, but detection quality FAILS on the current simulated sensor/profile.** This experiment does not promote DR-SPAAM to ROS or enable semantic/training masks. User confirmed keeping existing LiDAR + depth inputs; no RGB is added.

[Measured report](../../../archive/social_navigation/reports/lidar_person_pilot/LIDAR_PERSON_REPORT.md) · [acceptance](../../../archive/social_navigation/reports/lidar_person_pilot/acceptance.json) · [audit](../../../archive/social_navigation/reports/lidar_person_pilot/audit.json) · [full midpoint](../../../archive/social_navigation/reports/lidar_person_pilot/full01/midpoint/semantic_review.png) · [no-human midpoint](../../../archive/social_navigation/reports/lidar_person_pilot/null01/midpoint/semantic_review.png).

## Source and environment

Use the [authors' repository](https://github.com/RWTHVision/2D_lidar_person_detection), pinned commit d12e90225d52ba6c85d20061f1e7286d6a64aade, and their fully supervised JRDB checkpoint ckpt_jrdb_ann_dr_spaam_e20.pth. [upstream.json](upstream.json) records the official model-zoo link, checkpoint SHA and source hashes. Compatible inference source and its GPL-3.0 license remain in `runtime/`. The original source is retained at the recorded Git revision linked below. Only NumPy int-alias conversion and restricted CPU-mapped checkpoint loading change upstream code; [parity](../../../archive/social_navigation/reports/lidar_person_pilot/compatibility_parity.json) verifies identical cutouts, tensors and synthetic outputs.

A separate experiment venv uses Python3.12.3, PyTorch2.7.1+cu128 and CUDA12.8 on RTX4060. Ubuntu/ROS/default Python and existing SocNavGym environment are unchanged. The venv reuses system/user NumPy1.26.4/SciPy1.11.4/OpenCV4.6 read-only, so it is not a hermetic container. [Environment](../../../archive/social_navigation/reports/lidar_person_pilot/environment.json), [CUDA lock](requirements-cuda.lock.txt). Original upstream ROS node is ROS1; this trial uses plain Python offline, without a Jazzy node integration claim.

## Recreate setup in this existing workspace

No system apt installation is required. This machine lacks ensurepip for new venvs, so existing pip bootstraps only the new target environment:

```bash
TASK=experiments/social_navigation/benchmark/lidar_person_pilot
python3 -m venv --without-pip --system-site-packages "$TASK/generated/venv"
PIP_CACHE_DIR="$PWD/$TASK/generated/pip_cache" \
 experiments/social_navigation/.venv-socnavgym/bin/python -m pip \
 --python "$TASK/generated/venv" install -r "$TASK/requirements-cuda.lock.txt" \
 --index-url https://download.pytorch.org/whl/cu128
python3 "$TASK/download_checkpoint.py"
bash "$TASK/run.sh" test
```

The CUDA smoke script uses a fixed output name which refuses overwrite. The completed parity script and duplicate upstream tree have been retired from HEAD; their measured receipt remains in reports/. The inherited visibility/actor-freezer plugins must already be built with the ROS Jazzy vendor SDK, as in the preceding stage.

## Capture and inference

Use new output paths; keep failed runs. Each capture is headless on NVIDIA, bounded, domain91 locked, and cleans only its own partition/token. Native LiDAR definition is unchanged except topic:360rays,0–2pi inclusive,3.5m range,5Hz,1cm Gaussian noise, height0.12m above base. LiDAR still sees human collision cylinders; depth sees native mesh. Full and no-human controls preserve clutter and sensor definitions.

```bash
bash "$TASK/run.sh" capture --condition full --output "$TASK/generated/runs/manual_full"
bash "$TASK/run.sh" capture --condition null --output "$TASK/generated/runs/manual_null"
bash "$TASK/run.sh" infer --capture "$TASK/generated/runs/manual_full" --output "$TASK/generated/manual_infer"
bash "$TASK/run.sh" evaluate --capture "$TASK/generated/runs/manual_full" \
 --inference "$TASK/generated/manual_infer" --output "$TASK/generated/manual_eval"
# Audit this workspace's fixed recorded artifacts:
"$TASK/generated/venv/bin/python" "$TASK/audit.py" --output "$TASK/generated/manual_audit"
```

Inference reads only native scans, full depth, timestamps/receive-clock availability, calibration and frozen source/weights. No actor IDs, counterfactual depth or pixel labels enter the model. The duplicated final bearing is removed and existing rays cyclically ordered, with no interpolation; model FOV is358.9972degrees over359 unique rays. No-return handling follows the author's node. No y-flip or coordinate fit to GT is applied. The fixed confidence is0.5, panoramic=true, stride1. New model instance resets recurrence per condition.

Positive LiDAR detections can corroborate a depth proposal only with unique mutual spatial pairing (0.6m/0.15m margin), using a scan already available under the stored contract and no older than250ms. Every depth proposal is retained; lack of a detection means unknown. Even supported candidates keep semantic_valid=false pending acceptance. At t2s the first depth has no eligible scan; t3–6s have valid inputs. Receive-clock availability does not resolve callback ordering within an equal clock tick or measure wall latency.

## What failed

Across four usable repeated static frames, full scene has5 supported proposal rows:1 person-owned and4 object-owned. No-human control has3 false supported rows. Counts are repeated observations, not five independent people/episodes or a population precision estimate. All four visible people remain as geometry, mostly unknown. Both predeclared gates FAIL, including the no-human control; excluding cold start does not fix this. No confidence sweep/fine-tune or failure replacement was done.

Inference math/config were frozen before scan capture. First JSON write failed on a NumPy bool scalar. Only serialization was repaired after capture, before quality evaluation; original engine and [exact diff](../../../archive/social_navigation/reports/lidar_person_pilot/serialization_repair.patch) remain. [Audit](../../../archive/social_navigation/reports/lidar_person_pilot/audit.json) confirms the two changed captured files from the archived manifest/source and all unchanged math/config/weights. Original 9 tests, CUDA smoke and upstream parity passed. Sensor raw capture150images/40scans/cleanup passed; those infrastructure results do not turn semantic FAIL into PASS.

Git keeps source, small raw scan arrays, representative LABEL ONLY views, complete prediction JSONLs, reports and true measured plots. Full depth stacks/CUDA packages/weights remain local under generated/ and are not backed up by Git. No World Model training, dataset batch, HuNavSim, original ROS/tracker changes or navigation TIMEOUT repair occurred. Next is conservative depth surface evidence on the fixed regression and a new clutter fixture; this will remain separate from validated person semantics.

Cập nhật dọn 27/10/2026: đã gỡ các công cụ chỉ phục vụ cohort/report cũ: `preview_midpoint.py`. [Code trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/87b0c6eea9b54794db7b4cb794fae57c78f11c70/experiments/social_navigation/benchmark/lidar_person_pilot) giữ cách tái lập; report, engine, source freeze và outcome cũ không đổi.

Capture hiện tại dùng chung `python/dynamuhsim/runtime/sensor_capture.py`; file
`capture.py` của stage chỉ chọn fixture, protocol và provenance. Nhận ảnh/scan,
synchronization, giới hạn số frame và các gate static/moving/null giữ nguyên.
Source hash của lượt mới ghi thêm module dùng chung. Không append lượt mới vào
corpus/freeze cũ; tái lập kết quả lịch sử bằng source revision trong manifest.

World của profile được dựng bởi `python/dynamuhsim/simulation/sensor_fixture.py`.
Các file fixture tại stage chỉ chọn input; không gọi vòng qua fixture stage khác.
XML/URDF và metadata đã được đối chiếu byte với source trước khi gom.

Đã gỡ `upstream_original/` và script parity chỉ dùng một lần. Bản runtime,
checkpoint, protocol và mọi source pin của engine không đổi.
[Source upstream và parity trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/c28907172edf8afe6fe99f687c3ce5bce438de5f/experiments/social_navigation/benchmark/lidar_person_pilot)
vẫn đúng hash trong `upstream.json`. Auditor chỉ đọc byte của parity đã nghỉ
qua revision Git này để đối chiếu receipt cũ; không thực thi source lịch sử.
