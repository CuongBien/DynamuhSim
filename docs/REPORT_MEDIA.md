# Media report, model và trạng thái test

23 nhóm report còn giữ có dependency/runtime state cần bảo toàn. Đợt này chuyển riêng **214 media / 17.053.392 byte** khỏi HEAD; giữ nguyên byte **1.814 tệp / 53.322.406 byte** còn lại, gồm gate, config, snapshot, trọng số, review arrays, freeze và marker test.

[Media policy](artifacts/report_media_plan_2026-10-25.json) · [Manifest và coverage của 23 nhóm](artifacts/report_media_migration_2026-10-25.json) · [Biên bản](verification/report_media_2026-10-25.json).

## Vì sao giữ tệp điều khiển

`depth_person_classifier/evaluate.py` tạo `reports/test_opened.json` theo chế độ độc quyền trước khi đọc test split. Di chuyển nguyên report sẽ làm marker biến mất khỏi đường dẫn consumer. Đó là lý do giữ model, `selected_frozen.json`, `test_opened.json` và các kết quả đã đo tại chỗ, thay vì coi toàn bộ report là output có thể xóa.

Hai verifier engine cũng đọc freeze trong report và kiểm source/config bằng hash. Các snapshot, JSON/NPZ/PT và metadata liên quan vẫn ở vị trí cũ. Chỉ các đuôi media PNG/GIF/JPG/JPEG/MP4/WebM có tên và hash trong policy được hydrate bù; policy không được chọn JSON, model arrays hoặc code.

## Sử dụng

```bash
# Check chỉ đọc: exit1 nếu còn media thiếu; controls khác hoặc thiếu sẽ báo lỗi.
python3 tools/report_media.py check --all --output /tmp/media-check.json
# Phục hồi media classifier từ commit đã pin, không đụng model hoặc test seal.
python3 tools/report_media.py hydrate --group reports:depth_person_classifier \
  --source git --output /tmp/classifier-media.json
# Phục hồi toàn bộ media còn thiếu bằng archive local.
python3 tools/report_media.py hydrate --all --source archive \
  --store /PATH/TO/artifact_store --output /tmp/all-media.json
# Probe control với dữ liệu sensor tổng hợp, không train/test thật.
python3 tools/probe_report_controls.py --output /tmp/control-probe.json
```

Mỗi output phải mới. Probe cần Python/NumPy/SciPy; không cần ROS/Gazebo/CUDA. Git source cần commit lịch sử; không tự fetch. Archive source cần object và manifest. Media được stage, kiểm hash rồi link vào từng tên đích bằng thao tác không ghi đè. Nếu bị ngắt, có thể chạy lại để bù phần còn thiếu. Nếu có file khác, thiếu controls hoặc dữ liệu dư, công cụ dừng và giữ nguyên; dùng checkout replay riêng thay vì xóa control để ép restore.

`tools/hydrate_reports.py` mặc định tự bù approved media gaps trước khi kiểm nguyên nhóm, nên lệnh hydrate report hiện có vẫn dùng được. Với index/plan riêng, media policy không bị áp vào ngoài ý muốn; có thể cung cấp `--media-plan` riêng. API hydrate nguyên nhóm `hydrate_history` vẫn giữ hợp đồng strict: không merge một thư mục có sẵn khác inventory.

`--all-externalized` của hydrate_reports vẫn chỉ 24 nhóm đã externalize nguyên cây ở đợt trước; `report_media.py hydrate --all` chỉ phục hồi media của 23 nhóm còn giữ controls. Khi cần viewer cũ như `wm_dataset_replay/serve_recording.py`, hydrate media của nhóm đó trước; đợt này không mở server hoặc chạy replay.

## Kiểm chứng

**104 tests PASS.** Bao gồm Git/archive media restore, bảo toàn test seal/model, từ chối control bị thiếu/sửa, từ chối symlink và ghi đè, phục hồi sau khi một phần đã publish, wrapper report và index/plan riêng.

Native CLI phục hồi đủ 214 media từ archive và Git; mỗi file khớp hash. Wrapper report chỉ bù ba media classifier và không ghi lại model/seal. Các media thử đã được kiểm rồi dọn riêng; hiện checkout nhánh dọn dẹp chỉ giữ controls, media hydrate theo nhu cầu.

Probe được chạy trước chuyển media, khi media vắng mặt, sau archive, sau Git và ở cold state cuối: kết quả JSON giống byte trên cùng môi trường. Nó verify 10 file engine identity và sáu file refinement, kiểm binding của model/training/freeze/test seal, gọi inference NumPy với ba hàng sensor giả lập và tracker với bốn frame giả lập. Ngưỡng0.5, model hash và dấu test không đổi.

Audit hook ghi nhận 38 file read trong repository; chặn ghi, process/network và việc chạy snapshot trong reports, đồng thời cấm mở raw/corpus hoặc held-out prediction arrays trong probe. Có một lần thư viện thử gọi subprocess đã bị chặn; probe vẫn hoàn tất. Trace đầy đủ và coverage theo nhóm có trong biên bản/manifest. Đây là kiểm một tập API cụ thể, **không phải đã chạy đủ mọi consumer của 23 nhóm**, và không phải test accuracy hoặc lượt Gazebo/Nav2 mới.

Tất cả 1.814 control files đã được hash đối chiếu độc lập; hash review arrays chỉ dùng bảo toàn byte, không đưa vào model hoặc chọn tham số. Bảy gate, dataset01, labels/splits/freeze và checkout/index chính không đổi. Archive canonical giữ nguyên, chưa có bản remote replication được xác nhận.

## Dựng lại policy

```bash
python3 tools/report_media.py plan \
  --ref 03142a6efc849e347d84c10f5758d27eb60c2a2b \
  --output /tmp/rebuilt-media-policy.json
```

Lệnh được kiểm sau khi media vắng mặt và cho policy giống byte. Dùng mốc đã pin; HEAD mới không còn đủ media để tạo lại inventory gốc.

Sau ba đợt world/report/media, tổng **482.102.992 byte / 13.098 tệp payload** đã chuyển khỏi HEAD. Git history vẫn giữ dữ liệu cũ, nên không coi đây là giảm dung lượng lịch sử clone. Phần còn lại có vai trò điều khiển/reproducibility; trước khi dọn tiếp cần tách consumer khỏi các đường dẫn report này, không xóa marker/freeze để làm thư mục trông gọn.
