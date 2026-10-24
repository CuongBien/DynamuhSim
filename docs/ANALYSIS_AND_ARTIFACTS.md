# Analysis/audit và artifact dùng chung

Hai entrypoint hiện hành: `tools/audit_episode.py` và `tools/artifact_store.py`. Các audit trong experiment cũ được giữ để tái lập evidence đã freeze; không thay ngưỡng hoặc viết lại kết quả lịch sử.

## Helper chuẩn

| File trong `python/dynamuhsim/` | Trách nhiệm và nơi sử dụng |
|---|---|
| `analysis/evidence.py` | JSON/JSONL/gzip, streamed SHA256, source theo hash/commit. Dùng trong capture runtime, auditor và artifact store. |
| `analysis/timeline.py` | Nearest timestamp có tolerance, tie chọn mẫu trước; SE2 không ngoại suy, giới hạn gap và yaw đi theo cung ngắn. |
| `analysis/geometry.py` | Sai số pose và signed linear swept-disc gap; runtime goal và auditor dùng cùng measurement, policy giữ quyền quyết định PASS/FAIL. |
| `analysis/capture.py` | Audit capture mới, đọc MCAP qua dependency ROS lazy; không chạy lại mô phỏng hoặc thực thi source snapshot. |
| `artifacts/store.py` | Pack/verify/restore có SHA256 và dọn raw theo phạm vi hạn chế, sau round trip. |

Auditor có thể đọc source đúng hash từ root chỉ định hoặc Git commit đã ghi; không tự dùng HEAD thay thế. Snapshot trong artifact chỉ được hash/đọc, không import/exec. Producer freeze cũ vẫn thuộc revision đã ghi; auditor mới không đổi manifest đó thành source hiện tại.

## Hai policy có ý nghĩa khác nhau

- `configs/audit/capture_integrity.json`: kiểm dữ liệu/recorder/sensor/friction/TF/tail/provenance/cleanup. TIMEOUT được giữ và không tự khiến integrity FAIL; kết quả này không cấp quyền train WM.
- `configs/audit/navigation_goal.json`: cùng các kiểm tra, thêm bắt buộc action thành công và XY≤0,20m, yaw≤0,25rad thật. Một lượt TIMEOUT sẽ FAIL policy này.

Hai file chỉ khác tên và `require_actual_goal`. Ngưỡng pair/tail giữ bộ audit native cũ; bổ sung kiểm freeze identity, file ON/OFF thật có mặt và shape/encoding depth. Swept gap và odom alignment là mô tả, không tự thêm gate an toàn hoặc quy kết contact truth.

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/audit_episode.py --run-dir "$RUN_DIR" \
  --source-root "$PWD" --output "$AUDIT_DIR/integrity.json"
python3 tools/audit_episode.py --run-dir "$RUN_DIR" \
  --source-root "$PWD" --policy configs/audit/navigation_goal.json \
  --output "$AUDIT_DIR/navigation.json"
```

Mỗi output phải là file mới. Thiếu/corrupt raw hoặc source không khớp không được báo PASS. Nếu root vendor khác checkout, thêm `--source-root /absolute/path/to/original/checkout`; mọi candidate vẫn phải khớp SHA256.

## Lưu artifact và khôi phục

Chọn store được ignore và nằm ngoài thư mục run. Gói chứa toàn bộ capture, snapshot source đúng hash/commit, metadata và symlink log nội bộ chuyển thành đường dẫn tương đối; text link gốc vẫn lưu. File bytes và executable modes được giữ. Store là kho local, chưa phải bản sao trên Drive hoặc Git LFS.

```bash
STORE="$PWD/experiments/social_navigation/benchmark/generated/artifact_store"
python3 tools/artifact_store.py pack --store "$STORE" \
  --run-dir "$RUN_DIR" --source-root "$PWD"
# Lấy object ID SHA256 được lệnh pack in ra:
OBJECT_ID="<64-character-sha256>"
python3 tools/artifact_store.py verify --store "$STORE" --object-id "$OBJECT_ID"
python3 tools/artifact_store.py restore --store "$STORE" \
  --object-id "$OBJECT_ID" --output "$RESTORE_DIR"
python3 tools/audit_episode.py --run-dir "$RESTORE_DIR/run" \
  --snapshot-root "$RESTORE_DIR/source" --output "$AUDIT_DIR/restored.json"
```

Object nằm trong `objects/<id>.tar.gz`, manifest trong `manifests/<id>.json`. Pack lặp cùng nội dung không sinh object thứ hai. Verify kiểm hash tổng và từng member; restore từ chối output đã tồn tại, path traversal, link ra ngoài, member lặp và mismatch. Không có giải nén tar bằng extractall.

## Dọn raw đã archive

```bash
python3 tools/artifact_store.py evict-raw --store "$STORE" \
  --object-id "$OBJECT_ID" --run-dir "$RUN_DIR"
```

Lệnh này chỉ chạy sau pack/verify và một restore đã kiểm chứng. Nó đối chiếu lại toàn bộ run, từ chối file tracked, file không ignore, metadata/raw thay đổi hoặc capture chưa đóng recorder. Chỉ gỡ `.npy` trong `episode/paired_depth/{on,off}` và `.mcap` trong `episode/raw_bag`; giữ nguyên plan/freeze/result/index/log/preview/source. Không có recursive delete trong lệnh.

`artifact_ref.json` được ghi trước khi gỡ raw và lưu đủ danh sách/object ID để phục hồi nếu bị ngắt. Auditor báo rõ cần restore khi thấy ref; khôi phục vào thư mục mới rồi audit với `--snapshot-root`. Không ghi đè capture lịch sử. Dataset01, split/nhãn và các báo cáo tracked không thuộc đợt dọn này.

## Kiểm chứng

```bash
python3 -m unittest discover -s tests -p 'test_*.py' -v
```

Tests gồm timestamp nanosecond lớn, wrap yaw, overlap giữa hai mẫu, source Git pin, policy integrity/goal, depth corrupt/thiếu ON, artifact corruption, link/path/overwrite, concurrent writer, roundtrip và eviction bị chặn với tracked/changed/incomplete data. Báo cáo chạy trên capture thật được lưu trong `docs/verification/`.
