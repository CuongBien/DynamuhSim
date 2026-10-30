# Bắt đầu bàn giao MuJoCo

**Nhánh port hiện tại: feat/mujoco-backend.** Người nhận bắt đầu từ [MUJOCO_HANDOVER.md](MUJOCO_HANDOVER.md); [preflight một lệnh](../tools/mujoco_handoff.py) và [workspace mới](../experiments/social_navigation/mujoco_backend/README.md) đã chuẩn bị. MuJoCo hiện NOT_IMPLEMENTED, Gazebo reference giữ ở dd7a2b0.

Đây là trang vào hiện hành. Baseline Gazebo/data đã chốt ở **121a727**, nhánh `test/gazebo-harmonic-benchmark`. Vòng chuẩn bị bàn giao chỉ thêm entrypoint/check/setup/docs;116 file runtime,math,reader và bằng chứng được guard bằng SHA256 trong `configs/handoff/baseline.json`.

## Chạy bốn kiểm tra đầu tiên

Từ gốc checkout, với Python3.12 + NumPy:

```bash
python3 tools/handoff.py baseline
python3 tools/handoff.py doctor --scope data
python3 tools/handoff.py smoke-data --archive /absolute/path/handoff_data_pilot01.zip --work-dir /absolute/path/NEW_RESTORE_DIR
python3 tools/handoff.py smoke-contract
```

`NEW_RESTORE_DIR` phải chưa tồn tại. Nếu đã có dataset giải nén:

```bash
python3 tools/handoff.py smoke-data --dataset /absolute/path/handoff_data_pilot01
```

Tool kiểm archive SHA256,inventory và toàn bộ file trước khi gọi inspector/loader đã nghiệm thu trong Python isolated mode. Không cần raw,ROS,Gazebo hoặc MuJoCo để đọc pilot. Receipt tùy chọn `--output /new/path/check.json` dùng file mới, không ghi đè. CLI trả exit1 nếu thiếu dependency,đổi source/hash,dữ liệu lỗi hoặc inspector/test thất bại.

- [Setup theo ba mức](HANDOFF_SETUP.md).
- [Ranh giới backend và ownership source](BACKEND_BOUNDARY.md).
- [Hợp đồng dữ liệu,frame/mask và checklist MuJoCo](HANDOFF_MUJOCO.md).
- [Pilot và ZIP receipt](demos/2026-10-29/handoff_pilot/README.md).
- [Baseline/profile/backend contract machine-readable](../configs/handoff/).

## Mốc đã có

Pilot gồm2episode,517window,910depth frame khớp raw,2068BEV grid tái tính;loader NumPy-only và observation độc lập nhãn PASS. ZIP khoảng31.7MB,bung khoảng379MB. Full ZIP/raw không nằm trong Git. `pilot_only`,training_authorized=false,fit_allowed=false;chưa train WM/RL. Seed đã xem trong kiểm chứng simulator không phải split train/validation/test.

Hai chế độ là persistent conversation và natural walking. Hành vi nhóm/clock đã có bằng chứng ba seed;hai lượt teardown cuối PASS đủ4bridge. Mọi FAIL/ERROR cũ được giữ. Clock đứng có snapshot-age coverage,arrival/expiry query NOT_EXERCISED;active sidestep cũng chưa được thực thi trong cohort giữ nhóm. Chi tiết trong tài liệu gốc, không nâng phạm vi nghiệm thu khi dọn repo.

## Xem lệnh tham chiếu Gazebo

Nếu cần đối chiếu backend cũ, source ROS/workspace như trang setup rồi kiểm tra:

```bash
python3 tools/handoff.py doctor --scope gazebo
python3 tools/handoff.py reference-command --mode conversation --run-dir "$PWD/experiments/social_navigation/benchmark/generated/handoff_reference/conversation01"
python3 tools/handoff.py reference-command --mode natural_group_walking --run-dir "$PWD/experiments/social_navigation/benchmark/generated/handoff_reference/walking01"
```

`reference-command` chỉ in argv/Bash,không tạo run hoặc mở Gazebo. Nó chọn rõ recipe v7 đã chốt và cờ mode, không dựa vào default v3 của API capture lịch sử. Khi thực sự chạy các lệnh đã in,mỗi run có freeze/audit riêng. Không cần thêm native run để hoàn tất vòng chuẩn bị bàn giao này vì protected runtime không đổi.

## Phạm vi vòng chuẩn bị

Tài liệu lịch sử nằm trong `docs/history/`,`experiments/archive/` và các báo cáo `docs/demos/`. Chúng vẫn cần cho provenance/đối chiếu; không chuyển/xóa những path đang có hash/reference. Build/install/log,venv,vendor và generated cần được quản lý theo setup/artifact receipt,không copy vào một Python package mới.

Người tiếp nhận bắt đầu MuJoCo ở boundary đã ghi,giữ contract đầu ra và kiểm lại hai cảnh/loader. Vòng này chưa cài MuJoCo,viết MJCF/adapter,thu batch lớn hoặc thay kiến trúc robot.

## Nghiệm thu vòng chuẩn bị

[Biên bản setup](verification/handoff_setup_2026-10-29/README.md):310unit tests OK (2skip),38contract smoke tests,doctor đủ3scope,fresh ZIP restore+loader PASS;116protected hashes và2capture freezes cũ giữ nguyên. Không mở simulator/cài software trong vòng này.

## Detector depth thử nghiệm bổ sung

Xem [DEPTH_PERSON_EXPERIMENT.md](DEPTH_PERSON_EXPERIMENT.md) cho phần merge detector của nhánh feat/depth-person-detector. ROS interface đã kiểm, nhưng detector mới không đạt gate hình học trên pilot hiện tại; giữ opt-in, không dùng để train hoặc thay default. Baseline/data package ở trên vẫn nguyên vẹn.
