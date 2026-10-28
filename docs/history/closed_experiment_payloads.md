# Generated output và hai experiment đã đóng

Đã gỡ khỏi HEAD các payload dưới đây sau khi archive và khôi phục kiểm đầy đủ
SHA256, dung lượng và quyền file. Code generator, runtime, protocol và các
kết quả đã công bố không bị sửa để tạo outcome mới.

| Nhóm | Tệp | Byte gốc | Trạng thái hiện hành |
|---|---:|---:|---|
| Gazebo `configs/generated` | 1.505 | 11.266.397 | Output runtime bị ignore; 7 baseline đầu vào được giữ nguyên byte tại `configs/fixtures` |
| Persona oracle/observability | 24 | 22.560.383 | Không còn producer/evaluator; dùng source/report lịch sử |
| LiDAR Step1 verification | 5 | 205.876 | Harness đã nghỉ dùng; capture API mới giữ riêng |

[Index và hash archive](../artifacts/closed_payloads_2026-10-28.json).
Archive local ở thư mục evidence được ghi trong index; Git vẫn giữ đầy đủ
payload ở commit `d9cf082b2984b05b767d115d5505388f5afbb622`.

Đọc [persona report](https://github.com/CuongBien/DynamuhSim/tree/d9cf082b2984b05b767d115d5505388f5afbb622/experiments/pedestrian_persona_benchmark),
[LiDAR Step1 report](https://github.com/CuongBien/DynamuhSim/tree/d9cf082b2984b05b767d115d5505388f5afbb622/experiments/pipeline_verification)
và [generated snapshot](https://github.com/CuongBien/DynamuhSim/tree/d9cf082b2984b05b767d115d5505388f5afbb622/experiments/gazebo_benchmark/configs/generated).

## Khôi phục để tra cứu

Cần lịch sử chứa đúng commit trên. Chọn một thư mục mới, rỗng; ví dụ này không
khôi phục vào checkout hiện hành hoặc ghi đè output của lượt chạy mới:

```bash
git archive --format=tar --output=/tmp/dynamuhsim_closed_payloads.tar \
  d9cf082b2984b05b767d115d5505388f5afbb622 \
  experiments/gazebo_benchmark/configs/generated \
  experiments/pedestrian_persona_benchmark experiments/pipeline_verification
mkdir /tmp/dynamuhsim_closed_payloads_restored
tar -xf /tmp/dynamuhsim_closed_payloads.tar -C /tmp/dynamuhsim_closed_payloads_restored
```

Runtime hiện hành không cần khôi phục các payload này. Chín consumer baseline
đọc `configs/fixtures` và resolve root theo checkout đang chạy. MPPI, wall
clearance, A3.4 và matrix generator tiếp tục tự sinh output như trước; các SDF
và JSON cũ không được coi là nguồn đầu vào mặc định cho các lượt mới.
